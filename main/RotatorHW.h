#ifndef ROTATORHW_H
#define ROTATORHW_H

// #include "Arduino.h"
#include "AS5600.h"
#include "FastAccelStepper.h"
#include "TMC2209.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>

#define N_STEPS 400 // No of fullsteps for calibration
#define KMAX 4      // up to KMAX Fourier coefficients

class RotatorHW
{
public:
    // Access of the singleton
    static RotatorHW &getInstance();
    RotatorHW();

    void begin(); // call in setup()

    // Fast (single-sample) readout for live debug/UI use - unlike getPosition(),
    // this does not average 64 samples, so it is cheap enough to poll at a UI
    // refresh rate.
    struct SensorSnapshot
    {
        uint16_t rawSensor;
        double correctedSensor;
        int32_t stepPosition;
        bool hall; // true = magnet detected (sensor is active-low)
    };
    SensorSnapshot getSensorSnapshot();

    // AS5600 magnet/airgap health, straight off the chip's own AGC closed
    // loop (datasheet: "the gain value should be in the center of its
    // range... adjust the airgap to achieve this value"). Never checked in
    // this project before - a poorly seated magnet would show up here, and
    // no amount of downstream sensor-error fitting or filtering can correct
    // for a badly-conditioned raw measurement.
    struct SensorDiagnostics
    {
        uint8_t agc;         // 0-255 (5V mode); ideally near the middle of that range
        uint16_t magnitude;  // internal CORDIC magnitude, 0-4095
        bool magnetDetected;
        bool magnetTooStrong; // AGC pinned at its minimum-gain end
        bool magnetTooWeak;   // AGC pinned at its maximum-gain end
    };
    SensorDiagnostics getSensorDiagnostics();

    // For offline calibration/filter development against an external script
    // instead of a firmware rebuild+flash+wait cycle per iteration: jog the
    // motor by a raw, signed microstep count (bypassing every degree/offset
    // conversion the normal motion API applies) and read the sensor with a
    // caller-chosen averaging depth (1 = the same fast single sample
    // getSensorSnapshot() uses, higher = MEASURE_PRECISE_ANGLE_DOUBLE's
    // averaging, for when precision matters more than speed).
    void jogMicrosteps(int32_t microsteps);
    double measureRawAngle(int samples = 1);
    int32_t getStepPositionSafe();

    // Deliberately aggressive concurrent-load stress test for the still-open
    // calibrateAngleSensor() hang (memory/rotator_angle_cal_hang.md,
    // CALIBRATION_FINDINGS.md's "cross-task reliability issue"): repeats
    // that routine's core motion (full-step mode, forwardStep(), a brief
    // delay, an AS5600 read) `steps` times, while a dedicated auxiliary task
    // hammers AS5600 reads via i2cMutex far more often than the normal
    // ~650ms angle_producer_task cadence - trying to reproduce the same
    // freeze much faster than a full ~5-9 minute calibration sweep would.
    // `sampleCount` (vs. the real routine's fixed 64) trades measurement
    // precision, which this test does not care about, for more repetitions
    // per unit wall-clock time. Returns the number of steps actually
    // completed - less than `steps` only if this itself hangs, in which
    // case the caller never sees the return value either; onProgress is the
    // only signal to reach the client before that point.
    int32_t stressFullStepI2C(int32_t steps, int sampleCount, std::function<void(int)> onProgress);

    // A stepper motor only has FULLSTEPS_PER_ROTATION true mechanical
    // equilibrium positions - "microstep 137" is meaningless on its own, it
    // is only ever a position *within* a full step. This briefly switches
    // the driver to full-step mode, takes exactly one real full step (a true
    // mechanical equilibrium by construction, not an assumption), switches
    // back to 256 microsteps/step, and returns the resulting stepPosition -
    // an anchor a caller can use as "phase 0" for any within-full-step
    // analysis instead of assuming whatever stepPosition already happened to
    // be was full-step-aligned.
    int32_t alignToFullStep();

    // Quality metrics from a calibrateAngleSensor() run, in degrees of
    // AS5600 error. "Before" is measured against the coefficients that were
    // in effect when the run started, "after" against the freshly fitted
    // ones - both against the same fresh sweep, so they are directly
    // comparable.
    struct CalibrationResult
    {
        double residualBeforeDeg;
        double residualAfterDeg;
        double peakAfterDeg;
    };

    // external accessors
    bool getIsMoving()
    {
        return _isMoving;
    }
    bool getReverse()
    {
        return _isReverse;
    }
    void putReverse(bool reverse)
    {
        _isReverse = reverse;
    }
    // True if increasing Alpaca Position currently tracks physical clockwise
    // rotation - the configured Nominal Direction (_nominalClockwise, see
    // getNominalClockwise()/putNominalClockwise() below) XORed with Alpaca's
    // own Reverse property, exactly as Franz described it: two flags, the
    // mount-specific default and ASCOM's per-session override, combine to
    // say what the motor actually does. Also the sign getPosition()/put*
    // Position()/syncPosition() use for the mechanical<->Position mapping -
    // see their shared `dir` comment.
    bool getDirection()
    {
        return _isReverse != _nominalClockwise;
    }
    // Which physical rotation direction counts as "positive" (increasing
    // Position) by default for this telescope/mount, independent of Alpaca's
    // Reverse - see getDirection(). Persisted to LittleFS via Configuration
    // (application-specific, not a per-device calibration value), unlike
    // Reverse itself which Alpaca clients set per session and this firmware
    // never persists.
    bool getNominalClockwise()
    {
        return _nominalClockwise;
    }
    void putNominalClockwise(bool clockwise);
    double getPosition();
    double getTargetPosition()
    {
        return _targetPosition;
    }
    double getMechanicalPosition();
    // The smallest output-shaft angle this rotator can address (one
    // microstep, at the driver's full 256-microstep resolution) - what
    // Alpaca's Rotator.StepSize reports.
    double getStepSizeDegrees();
    void gotoMechanicalZero();
    int measureMechanicalZero(std::function<void(int)> onProgress);
    // Updates _zeroPosSensorValue (the corrected-sensor-value target at true
    // mechanical zero) both in memory and in NVS, so a fresh
    // measureMechanicalZero() result survives reboots without a firmware
    // rebuild - see the constructor's comment on why that used to be a
    // hardcoded constant.
    void setZeroPosSensorValue(int value);
    // void measureMechanicalZero(int noOfMeasures = 1);
    void findEdge(bool dir);
    // void calibrateAngleSensor(void);
    CalibrationResult calibrateAngleSensor(std::function<void(int)> onProgress);
    // Overwrites the AS5600 correction coefficients directly, updating both
    // the live in-memory values correctSensorReading() uses immediately and
    // NVS (so they survive a reboot) - the same two-step
    // calibrateAngleSensorFinalize() does for an on-device sweep, exposed
    // here for the camera-referenced offline pipeline instead
    // (scripts/camera_angle_analyze.py's fit_fourier_against_reference(),
    // which fits against an independent reference instead of
    // correctSensorReading()'s only available self-referential phase basis
    // - see CALIBRATION_FINDINGS.md's "self-referential correction bias").
    // a/b must each have exactly KMAX+1 entries (index 0 is C0's harmonic-
    // free sibling and is ignored, kept only for index alignment with A/B's
    // own 1-based convention elsewhere in this file).
    void setAngleCalCoefficients(double c0, const double a[KMAX + 1], const double b[KMAX + 1]);
    // A second, much finer correction layered on top of C0/A/B: a
    // zero-mean, N_STEPS-entry table indexed by which full motor step the
    // (already C0/A/B-corrected) reading estimates it is near, added in
    // correctSensorReading() after the smooth harmonic model. Exists
    // because that smooth (order<=KMAX) model structurally cannot
    // represent error that repeats once per FULL STEP rather than once per
    // motor revolution - live-measured 2026-09-11
    // (scripts/calibration_lab.py's fullstep-accuracy command): an order-6
    // fit still left RMS=1.30 deg/peak=3.56 deg (motor-shaft) at full-step
    // resolution, yet that residual correlates +1.000 between two separate
    // measured revolutions (diff RMS only 0.049 deg) - a large but almost
    // perfectly repeatable, and therefore almost perfectly correctable,
    // error a low harmonic order simply cannot reach. All-zero (a no-op)
    // until scripts/camera_fullstep_table.py's output has been uploaded via
    // setFullStepTable(); persisted the same way as C0/A/B (live values +
    // NVS) - see that function.
    void setFullStepTable(const float table[N_STEPS]);
    void getFullStepTable(float outTable[N_STEPS]);
    double correctSensorReading(double sensorReading);
    // void fitSinusoidalErrorFromSteps(const std::vector<double> &y, int N, double &out_amplitude, double &out_phase);
    void putHalt();
    // Each returns false, without moving the motor or updating
    // TargetPosition, if the requested move would take the mechanical
    // position further than the cable-wrap motion limit from mechanical
    // zero - see MOTION_LIMIT_DEG in RotatorHW.cpp.
    bool putRelativePosition(double position);
    bool putAbsolutePosition(double position);
    bool putMechanicalPosition(double position);
    void syncPosition(double position);

private:
    // The only synchronized access to as5600: angle_producer_task polls
    // getPosition() every 100 ms from its own (unpinned) task while the
    // homing and calibration routines below make hundreds of their own
    // readAngle() calls from whichever task called them. Wire's
    // beginTransmission/write/endTransmission/requestFrom/read sequence is
    // not atomic, so two of those interleaving from different cores corrupts
    // the transaction - this is what crashed gotoMechanicalZero() a few
    // minutes after every boot before this existed.
    uint16_t readAngleSafe();

    // Runs for the process lifetime once begin() starts it, at a fixed
    // 150ms cadence (matching refineToTarget()'s own iteration period):
    // while _holdActive is set (after a commanded move/homing settles - see
    // put*Position()/gotoMechanicalZero()), continuously runs the same
    // Kalman-filtered PI control law as refineToTarget() to correct for
    // drift off the last commanded target - e.g. cable tension or an
    // unbalanced camera slowly torquing the shaft between commands - rather
    // than reacting to occasional, individually noisy samples. Disengaged
    // by putHalt() and by any routine that leaves the step-position counter
    // temporarily untrustworthy (calibrateAngleSensor(),
    // measureMechanicalZero(), alignToFullStep(), stressFullStepI2C()) -
    // re-engaged only by the next successful gotoMechanicalZero() or
    // put*Position() call. Takes motionMutex only via a non-blocking try
    // each cycle, so it can never add latency to, or deadlock with, a
    // user-commanded operation already holding it.
    static void holdTask(void *arg);

    // Same idea, for FastAccelStepper's position counter: angle_producer_task
    // reads it every 100 ms via getPosition()/getMechanicalPosition()/
    // getSensorSnapshot() while homing/calibration call setCurrentPosition()
    // to redefine the reference from a different task. On this ESP32 target
    // FastAccelStepper backs the counter with a PCNT hardware register plus a
    // software overflow-extension word; setCurrentPosition() has to update
    // both, and a concurrent read caught mid-update can see an inconsistent
    // combination of the two. That is what turned a calibration run's
    // position rescale into a nonsense multi-million-step reading on the
    // bench - not the rescale arithmetic itself. getStepPositionSafe() is
    // public (declared above) since it is a plain, harmless read; this
    // setter stays private - it redefines the reference and is only safe in
    // the few call sites that already reason carefully about the consequence.
    void setStepPositionSafe(int32_t newPosition);

    // Closed-loop refinement run after every MOVETO_WAIT in
    // putRelativePosition()/putAbsolutePosition()/putMechanicalPosition():
    // the open-loop move gets close, this measures (Kalman-filtered AS5600
    // reading) and issues small PI-controlled corrective jogs until within
    // tolerance or out of iterations. Ported from the live-tested Python
    // prototype in scripts/pi_position_control.py - see
    // CALIBRATION_FINDINGS.md for why (closed-loop measurement beat every
    // motor/gearbox model this project tried) and for the anti-windup
    // lesson (conditional integration - see the .cpp - is required, a plain
    // PI overshot badly without it). Adds up to ~20 extra small moves worth
    // of latency to a Move/MoveAbsolute/MoveMechanical call in the worst
    // case (typically 1-3, a few hundred ms) - not yet verified against
    // real hardware from this change.
    void refineToTarget(long targetMotorSteps);

    // Sweeps in `direction`, `chunk` microsteps at a time (up to `budget`
    // total), until the Hall GPIO's active/inactive state differs from
    // whatever it was when this was called - confirmed by continuing
    // `debounceSteps` further in the same direction and checking it still
    // differs, not just a brief electrical glitch (live-measured on this
    // GPIO - see CALIBRATION_FINDINGS.md). Returns false, with the caller
    // left wherever the sweep stopped, if `budget` is exhausted without a
    // confirmed change.
    bool sweepUntilHallChange(int32_t direction, int32_t chunk, int32_t budget, int32_t debounceSteps);

    // Finds the raw motor step target, among all mechanically-equivalent
    // candidates for the given wrapped mechanical angle (must be in
    // [0, 360)) - candidate, candidate-360, candidate+360 - that stays
    // within the cable-wrap motion limit, trying the shortest-path one
    // first. On a rotator limited to little more than one full turn
    // (MOTION_LIMIT_DEG), the shortest path is not always legal even when a
    // longer one reaching the exact same physical orientation is - see
    // RotatorHW.cpp. Returns false (leaving *outSteps at the shortest,
    // illegal candidate, for logging) if no candidate is legal.
    bool legalMotorStepsForAngle(double wrappedMechDeg, long *outSteps);
    // True if the given raw step count is within the cable-wrap motion
    // limit (MOTION_LIMIT_STEPS) of mechanical zero (raw step 0).
    bool withinMotionLimit(long targetSteps);

    // sensor calibration
    void calibrateAngleSensorInit(void);
    void calibrateAngleSensorStep(double sensor_raw);
    void calibrateAngleSensorFinalize(void);
    struct ResidualStats
    {
        double rmsDeg;
        double peakDeg;
    };
    // RMS/peak error of the *currently active* correction (whatever is in
    // C0/A/B right now) against a sweep of averaged raw readings indexed by
    // ideal step position.
    ResidualStats computeResidual(const std::vector<double> &avgRaw);

private:
    bool _isMoving;
    bool _isReverse;
    // Loaded from Configuration in the constructor, updated by
    // putNominalClockwise() - see that method and getDirection().
    bool _nominalClockwise;
    double _targetPosition;

    int16_t _zeroPosSensorValue;
    double _zeroPosSensorOffset;

    int sensorValueforMechanicalZero;
    double positionOffsetToMechanicalPosition;
    double sensorCorrectionAmplitude;
    double sensorCorrectionPhase;

    HardwareSerial &serial_stream;

    // Two-state motion model, per Franz's request: a commanded move (put*
    // Position()/gotoMechanicalZero()) drives _isMoving, and once it settles
    // on a target, continuous holding (holdTask() above) takes over to
    // correct for drift until the next commanded move, Halt, or maintenance
    // routine. _holdTargetSteps is a raw motor step count, same reference
    // frame as getStepPositionSafe() - unaffected by Reverse or Sync.
    volatile bool _holdActive;
    long _holdTargetSteps;
    // Serializes every routine that issues real motor commands (stepper->
    // move()/moveTo()/forwardStep()) against holdTask(), so continuous
    // holding never fights an in-progress commanded move, homing sweep, or
    // calibration sweep for the same stepper object. holdTask() only ever
    // *tries* to take this (never blocks), so it can never add latency to,
    // or deadlock with, a user-commanded operation.
    SemaphoreHandle_t motionMutex;

    // internal calibration values
    double sumC[KMAX + 1];
    double sumS[KMAX + 1];
    double sum0;
    double C0;
    double A[KMAX + 1];
    double B[KMAX + 1];
    int step_counter;
    // See setFullStepTable()'s comment.
    float _fullStepTable[N_STEPS];

    // Instantiate TMC2209
    TMC2209 stepper_driver;
    // Instantiate Rotary Sensor
    AS5600 as5600; //  use default Wire
    SemaphoreHandle_t i2cMutex;
    SemaphoreHandle_t stepperMutex;
    // Instantiate the AccelStepper Library and bind it to our TMC2209
    // AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    FastAccelStepperEngine engine;
    FastAccelStepper *stepper;
};

#endif // ROTATORHW_H