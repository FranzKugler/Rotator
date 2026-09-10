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
    bool getDirection()
    {
        return _isReverse != _isClockwise;
    }
    double getPosition();
    double getTargetPosition()
    {
        return _targetPosition;
    }
    double getMechanicalPosition();
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
    double correctSensorReading(double sensorReading);
    // void fitSinusoidalErrorFromSteps(const std::vector<double> &y, int N, double &out_amplitude, double &out_phase);
    void putHalt();
    void putRelativePosition(double position);
    void putAbsolutePosition(double position);
    void putMechanicalPosition(double position);
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
    bool _isClockwise;
    double _targetPosition;

    int16_t _zeroPosSensorValue;
    double _zeroPosSensorOffset;

    int sensorValueforMechanicalZero;
    double positionOffsetToMechanicalPosition;
    double sensorCorrectionAmplitude;
    double sensorCorrectionPhase;

    HardwareSerial &serial_stream;

    // internal calibration values
    double sumC[KMAX + 1];
    double sumS[KMAX + 1];
    double sum0;
    double C0;
    double A[KMAX + 1];
    double B[KMAX + 1];
    int step_counter;

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