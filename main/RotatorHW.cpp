#include "RotatorHW.h"
#include "Configuration.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include "freertos/task.h"

static const char *TAG = "rotator";

#define CW true
#define CCW false

#define WAIT_FOR_STOPPED_MOTOR                   \
    {                                            \
        while (stepper->isRunning())             \
            vTaskDelay(10 / portTICK_PERIOD_MS); \
    }
#define MOVE_WAIT(position) (    \
    {                            \
        stepper->move(position); \
        WAIT_FOR_STOPPED_MOTOR   \
    })
#define MOVETO_WAIT(position) (    \
    {                              \
        stepper->moveTo(position); \
        WAIT_FOR_STOPPED_MOTOR     \
    })
#define MEASURE_PRECISE_ANGLE(count) (           \
    {                                            \
        long int angle = 0;                      \
        for (int i = 0; i < count; i++)          \
        {                                        \
            angle += readAngleSafe();            \
            vTaskDelay(10 / portTICK_PERIOD_MS); \
        }                                        \
        angle / count;                           \
    })

#define MEASURE_PRECISE_ANGLE_DOUBLE(count) (                                  \
    {                                                                          \
        int32_t angle = readAngleSafe();                                       \
        int32_t start = angle;                                                 \
        for (int i = 1; i < count; i++)                                        \
        {                                                                      \
            vTaskDelay(10 / portTICK_PERIOD_MS);                               \
            int32_t delta = (readAngleSafe() - start + 6144) % 4096 - 2048;    \
            angle += start + delta;                                            \
        }                                                                      \
        (double)angle / (double)count;                                         \
    })

#define FMOD360(x) (fmod(fmod(x, 360) + 360, 360))
#define FMOD4096(x) (fmod(fmod(x, 4096) + 4096, 4096))
#define EXPECTED_SENSORVALUE(x) ((sensorValueforMechanicalZero + int(4096.0 * fmod(x, 36.0) / 36.0)) % 4096)
#define CORRECTED_SENSORVALUE(x) (correctSensorReading(x))

// Scalar linear Kalman filter over an unwrapped OUTPUT-shaft angle (degrees) -
// see RotatorHW::refineToTarget() and scripts/angle_filter_lab.py's
// AngleKalman1D, which this mirrors exactly (same predict/update maths).
// Kept local to this translation unit - only refineToTarget() needs it.
struct AngleKalman1D
{
    double x = 0.0;
    double P = 0.0;
    double Q = 0.0;
    double R = 0.0;

    void predict(double commandedDeltaOutputDeg)
    {
        x += commandedDeltaOutputDeg;
        P += Q;
    }

    void update(double measuredOutputDeg)
    {
        double y = measuredOutputDeg - x;
        double S = P + R;
        double K = (S > 0.0) ? P / S : 0.0;
        x += K * y;
        P *= (1.0 - K);
    }
};

// RAII guard for RotatorHW::motionMutex - every routine that issues real
// motor commands takes one of these as its first statement, so the mutex is
// released on every return path (including the early-return refusals in
// put*Position()/gotoMechanicalZero()) without repeating xSemaphoreGive() at
// each one.
namespace
{
struct MotionLock
{
    SemaphoreHandle_t m;
    explicit MotionLock(SemaphoreHandle_t m) : m(m) { xSemaphoreTake(m, portMAX_DELAY); }
    ~MotionLock() { xSemaphoreGive(m); }
};
} // namespace

// Pins on XIAO ESP32-S3 GPIO-Numbers
static constexpr gpio_num_t RX_PIN = GPIO_NUM_7;      // brown
static constexpr gpio_num_t TX_PIN = GPIO_NUM_8;      // white
static constexpr gpio_num_t ENABLE_PIN = GPIO_NUM_1;  // gray
static constexpr gpio_num_t STEP_PIN = GPIO_NUM_3;    // yellow
static constexpr gpio_num_t DIR_PIN = GPIO_NUM_4;     // purple
static constexpr gpio_num_t I2C_SDA = GPIO_NUM_5;     // white
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_6;     // brown
static constexpr gpio_num_t AS5600_DIR = GPIO_NUM_43; // gray
static constexpr gpio_num_t HALLSENSOR = GPIO_NUM_2;  // yellow

// Some HW Constants
const short unsigned int MICROSTEPS = 256;
const short unsigned int FULLSTEPS_PER_ROTATION = 400;
const short unsigned int SENSORCOUNT_STEPS_PER_ROTATION = 4096;
const short unsigned int STEPS_PER_SENSORCOUNT = (MICROSTEPS * FULLSTEPS_PER_ROTATION) / SENSORCOUNT_STEPS_PER_ROTATION;
const long unsigned int STEPS_PER_ROTATION = FULLSTEPS_PER_ROTATION * 10 * MICROSTEPS;
const double DEGREE_PER_STEP = 360.0 / FULLSTEPS_PER_ROTATION / 10 / MICROSTEPS;
const uint32_t NORMAL_MOTOR_SPEED = STEPS_PER_ROTATION / 10;
// const long unsigned int EDGE_STEPS = 256 * 256;

// Cable-wrap protection: once a camera is attached via a cable, the rotator
// must never be commanded further than this from mechanical zero (raw step
// 0, established by the last successful gotoMechanicalZero()) in either
// direction - Franz's number, sized for the planned cable with some margin.
// gotoMechanicalZero()'s boot-time sector search is bounded to the same
// figure (search up to this far one way, then return to the boot-time start
// and try the other way) for the same reason: two 190-degree sweeps in
// opposite directions from one starting point cover a full revolution with
// margin, and the sector must lie within it, since normal operation never
// lets a completed move end up further than this from zero either.
const double MOTION_LIMIT_DEG = 190.0;
const long MOTION_LIMIT_STEPS = (long)(MOTION_LIMIT_DEG / DEGREE_PER_STEP);

RotatorHW &RotatorHW::getInstance()
{
    static RotatorHW instance;
    return instance;
}

RotatorHW::RotatorHW()
    : _isMoving(false), _isReverse(false), _nominalClockwise(true), _targetPosition(0),
      serial_stream(Serial1), _holdActive(false), _holdTargetSteps(0)
{
    // Created here, before begin() brings up any task that could touch
    // as5600, so readAngleSafe() never sees a null handle.
    i2cMutex = xSemaphoreCreateMutex();
    stepperMutex = xSemaphoreCreateMutex();
    motionMutex = xSemaphoreCreateMutex();

    auto &cfg = Configuration::getInstance();

    // set Fourier coefficients from config
    for (int k = 0; k <= KMAX; k++)
    {
        A[k] = cfg.getA(k);
        B[k] = cfg.getB(k);
    }
    C0 = cfg.getC0();
    _nominalClockwise = cfg.getNominalClockwise();
    cfg.getFullStepTable(_fullStepTable);

    // The corrected-sensor-value target at true mechanical zero - see
    // measureMechanicalZero() (the calibration routine, binary-search edge
    // finding via findEdge()) and setZeroPosSensorValue() (what persists a
    // fresh measurement here). This used to be a hardcoded constant,
    // requiring a firmware rebuild+reflash any time the Hall window's
    // calibration drifted (e.g. after the config.json C0 update on
    // 2026-09-08 that made the old hardcoded 27 stop matching the real
    // window at all) - now loaded from NVS, falling back to 27 (that last
    // hand-measured value) only if nothing has been calibrated yet.
    _zeroPosSensorValue = 27;
    _zeroPosSensorOffset = 27.0;
    {
        nvs_handle_t nvs;
        if (nvs_open("homing", NVS_READONLY, &nvs) == ESP_OK)
        {
            int32_t stored = 0;
            if (nvs_get_i32(nvs, "zeroSensor", &stored) == ESP_OK)
                _zeroPosSensorValue = (int16_t)stored;
            nvs_close(nvs);
        }
    }

    sensorValueforMechanicalZero = 3747;
    positionOffsetToMechanicalPosition = 20.0;
    sensorCorrectionAmplitude = 0.003606;
    sensorCorrectionPhase = -0.609041;
}

void RotatorHW::begin()
{
    ESP_LOGI(TAG, "Intializing Rotator HW");
    // Setup the TMC2209 driver, 280mA RMS with Enable pin.
    //
    // Two calls used to run right after this: one that forced the run
    // current to the driver's absolute maximum (CS=31, ~980mA at this
    // Rsense/vsense) - discarding the CS that setRMSCurrent() had just
    // computed, while the hold current stayed at the small
    // setRMSCurrent()-derived value, a ~16:1 run:hold mismatch - and one that
    // enabled CoolStep. At this motor's 5V supply, 980mA is also far beyond
    // what the coil (17ohm/phase) can physically draw (~290mA ceiling), so
    // StealthChop's current regulator saturated its PWM duty cycle for most
    // of each microstep's sine/cosine wave, only regulating correctly near
    // the zero crossings. Removing the override alone (250mA target, CS=7)
    // measurably worsened wobble right after alignToFullStep() on the real
    // rotator: the old, distorted waveform apparently delivered more average
    // torque against cogging/friction than a clean-but-weak 245mA sine peak
    // does. 280mA (CS=8, ~276mA actual peak) stays safely under the ~290mA
    // physical ceiling - so the regulator still never saturates - while
    // recovering some of that torque headroom; this needs the same live
    // verification before it can be trusted. CoolStep still only pulls
    // current below the run setting whenever it estimates low load, fighting
    // positional stiffness - precision, not quietness or power draw, is what
    // this application needs, so it stays off.
    stepper_driver.setup(serial_stream, 250000, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
    stepper_driver.setHardwareEnablePin(ENABLE_PIN);
    stepper_driver.setRMSCurrent(280, 0.11, 0.6);
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.disableCoolStep();
    stepper_driver.enable();

    // Setup the AS5600 driver
    Wire.begin(I2C_SDA, I2C_SCL);
    as5600.begin(); //  set SW direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);

    // set input pin of Hall sensor, active low with pull-up
    pinMode(HALLSENSOR, INPUT_PULLUP);

    // init engine (sets up MCPWM + PCNT under the hood)
    engine.init();
    // connect one stepper to STEP_PIN
    stepper = engine.stepperConnectToPin(STEP_PIN);

    // configure direction & enable
    stepper->setDirectionPin(DIR_PIN, true);
    stepper->setEnablePin(ENABLE_PIN);
    stepper->enableOutputs();
    // stepper->setAutoEnable(true);

    // motion parameters
    // maximum speed in steps / sec. Assuming 10s for a full rotation...
    stepper->setSpeedInHz(NORMAL_MOTOR_SPEED);
    stepper->setAcceleration(100000); // 100000 steps/sec²

    //gotoMechanicalZero();
    //  measureMechanicalZero(10);
    // calibrateAngleSensor();

    // Continuous position holding - see holdTask()'s comment in the header
    // and RotatorHW.h's _holdActive/_holdTargetSteps. Low priority: it sleeps
    // almost all the time (see holdTask's idlePeriod) and only ever competes
    // for the motor via a non-blocking mutex try.
    xTaskCreate(holdTask, "hold", 4096, this, 1, nullptr);

    ESP_LOGI(TAG, "Finished intializing Rotator HW");
}

/**
 * The one place allowed to touch as5600. See the declaration in RotatorHW.h
 * for why: angle_producer_task and the homing/calibration routines below can
 * run on different cores at once, and Wire's multi-step transaction is not
 * safe to interleave.
 */
uint16_t RotatorHW::readAngleSafe()
{
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    uint16_t value = as5600.readAngle();
    xSemaphoreGive(i2cMutex);
    return value;
}

/**
 * The only synchronized access to the stepper's position counter. See the
 * declaration in RotatorHW.h for why: FastAccelStepper's ESP32 backend pairs
 * a PCNT hardware register with a software overflow-extension word, and
 * setCurrentPosition() has to update both - a getCurrentPosition() from
 * another task caught mid-update can read an inconsistent combination of the
 * two.
 */
int32_t RotatorHW::getStepPositionSafe()
{
    xSemaphoreTake(stepperMutex, portMAX_DELAY);
    int32_t position = stepper->getCurrentPosition();
    xSemaphoreGive(stepperMutex);
    return position;
}

void RotatorHW::setStepPositionSafe(int32_t newPosition)
{
    xSemaphoreTake(stepperMutex, portMAX_DELAY);
    stepper->setCurrentPosition(newPosition);
    xSemaphoreGive(stepperMutex);
}

bool RotatorHW::sweepUntilHallChange(int32_t direction, int32_t chunk, int32_t budget, int32_t debounceSteps)
{
    bool startState = digitalRead(HALLSENSOR);
    int32_t traveled = 0;
    while (traveled < budget)
    {
        MOVE_WAIT(direction * chunk);
        traveled += chunk;
        if (digitalRead(HALLSENSOR) != startState)
        {
            // Confirm over a further debounceSteps before trusting it - a
            // brief electrical glitch on this GPIO (live-measured, see
            // CALIBRATION_FINDINGS.md) would not survive this, only a real,
            // sustained state change will.
            MOVE_WAIT(direction * debounceSteps);
            traveled += debounceSteps;
            if (digitalRead(HALLSENSOR) != startState)
                return true;
            // false alarm - already moved past it, keep sweeping
        }
    }
    return false;
}

void RotatorHW::gotoMechanicalZero()
{
    MotionLock motionLock(motionMutex);
    ESP_LOGI(TAG, "Goto Mechanical Zero");
    _isMoving = true;
    // Untrustworthy until a fresh zero is established below - see holdTask()
    // and this function's success/failure paths.
    _holdActive = false;

    // ---- Which sector: find the Hall window, chunked + debounced ----
    //
    // This only needs to confirm we're SOMEWHERE inside the (once-per-
    // OUTPUT-revolution) Hall window - not pinpoint an edge. That matters:
    // an earlier version of this routine (see CALIBRATION_FINDINGS.md) spent
    // most of its complexity, and several bugs, precisely locating both
    // edges to derive "zero" from their midpoint. It doesn't need to -
    // _zeroPosSensorValue already records the AS5600 reading AT true zero
    // (from measureMechanicalZero(), which uses proper binary-search edge
    // finding via findEdge() and persists the result - see
    // setZeroPosSensorValue()). Once ANY point inside the window is
    // confirmed, the AS5600 reading there uniquely determines the offset to
    // that stored target (the window is narrower than one AS5600 revolution,
    // so there is no ambiguity once we know we're inside the one true
    // window, unlike matching an AS5600 value with no such confirmation
    // first - the original bug this whole rewrite exists to fix).
    const int32_t SECTOR_SEARCH_CHUNK = 4096;
    const int32_t DEBOUNCE_STEPS = 2048;
    // Bounded to the cable-wrap motion limit, not a full revolution: this
    // search runs before mechanical zero is even known, so it can only be
    // bounded relative to wherever boot happened to leave the rotator - see
    // MOTION_LIMIT_DEG's comment for why two such sweeps in opposite
    // directions, from the same start, are still guaranteed to cover the
    // sector regardless of where it is, as long as normal operation (also
    // limited to MOTION_LIMIT_DEG from zero) is what put the rotator here.
    const int32_t SECTOR_SEARCH_BUDGET = MOTION_LIMIT_STEPS;
    const int MAX_ATTEMPTS = 3;

    bool landedInWindow = false;
    for (int attempt = 1; attempt <= MAX_ATTEMPTS && !landedInWindow; ++attempt)
    {
        // Fixed per attempt, before either sweep moves anything - the CW
        // fallback below returns here first, rather than continuing on from
        // wherever the failed CCW sweep stopped, so the two sweeps' combined
        // travel never exceeds the cable-wrap limit even in the worst case.
        long searchStartSteps = getStepPositionSafe();

        // true = found via the CCW sweep, false = via the CW fallback - this
        // drives deltaSensorPos's sign below (which way "toward the
        // calibrated target" is), not whether the search succeeded (both
        // branches abort on failure, right where each is decided).
        //
        // sweepUntilHallChange() looks for the Hall state to change from
        // whatever it is *right now* - if we're already inside the window
        // (a real case: calling this again shortly after a previous
        // successful call, or landing inside it by chance) that would
        // detect *leaving* the window as "found", not entering it, and
        // everything after would then reason from a wrong starting
        // assumption. Live-measured: three consecutive calls, each already
        // starting inside the window, each took ~3m42s (the two full sweep
        // budgets on top of each other, tried MAX_ATTEMPTS times) and
        // returned the exact same, uncorrected position every time - this
        // check is what those calls were missing.
        bool foundViaCCW = true;
        if (digitalRead(HALLSENSOR)) // active-low: HIGH means NOT already inside
        {
            foundViaCCW = sweepUntilHallChange(-1, SECTOR_SEARCH_CHUNK, SECTOR_SEARCH_BUDGET, DEBOUNCE_STEPS);
            if (!foundViaCCW)
            {
                ESP_LOGW(TAG, "Index not found within %.0f deg CCW - returning to start, trying CW", MOTION_LIMIT_DEG);
                MOVETO_WAIT(searchStartSteps);
                if (!sweepUntilHallChange(1, SECTOR_SEARCH_CHUNK, SECTOR_SEARCH_BUDGET, DEBOUNCE_STEPS))
                {
                    ESP_LOGW(TAG, "Index not found within %.0f deg CW either - homing aborted, previous position kept", MOTION_LIMIT_DEG);
                    _isMoving = false;
                    return;
                }
            }
        }

        // ---- Exact zero: one direct move to the calibrated AS5600 target ----
        int32_t deltaSensorPos;
        double startPos = CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
        if (foundViaCCW)
            deltaSensorPos = -FMOD4096(startPos - _zeroPosSensorValue - 3) * STEPS_PER_SENSORCOUNT;
        else
            deltaSensorPos = FMOD4096(_zeroPosSensorValue - startPos - 3) * STEPS_PER_SENSORCOUNT;

        ESP_LOGI(TAG, "Inside Hall window, startPos=%.1f, moving %ld steps to calibrated zero (sensor target %d)",
                 startPos, (long)deltaSensorPos, _zeroPosSensorValue);
        MOVE_WAIT(deltaSensorPos);

        // Sanity check: the calibrated target sits well inside the Hall
        // window (not at its edge), so a genuinely correct landing should
        // still read active here. Live-measured (scripts/homing_repeatability.py
        // + camera verification, see CALIBRATION_FINDINGS.md): despite the
        // debouncing above, roughly 1 in 5 attempts still landed ~36 degrees
        // (one motor revolution) off, and every such case read Hall inactive
        // at this point, while every correct landing read active - a brief
        // electrical glitch on this GPIO occasionally still survives the
        // sector-search debounce, but this catches it before it's trusted as
        // "home".
        landedInWindow = !digitalRead(HALLSENSOR);
        if (!landedInWindow)
            ESP_LOGW(TAG, "Landed outside the Hall window after the direct move - attempt %d/%d, retrying",
                     attempt, MAX_ATTEMPTS);
    }
    if (!landedInWindow)
    {
        ESP_LOGW(TAG, "Still outside the Hall window after %d attempts - homing aborted, previous position kept", MAX_ATTEMPTS);
        _isMoving = false;
        return;
    }

    _zeroPosSensorOffset = CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(256));

    ESP_LOGI(TAG, "Sensor at mechanical Zero = %f, motor position before reset = %ld", _zeroPosSensorOffset, (long)getStepPositionSafe());
    setStepPositionSafe(0);
    _isMoving = false;
    // Zero is now trustworthy - hold it until the next commanded move.
    _holdTargetSteps = 0;
    _holdActive = true;

    /*
    // assuming that we're still in the Hallsensor "window", find the CCW magnetic edge
    findEdge(CCW);

    // read sensor - very precise
    int sensorAngle = MEASURE_PRECISE_ANGLE(64);
    int32_t stepperPosition1 = stepper->getCurrentPosition();
    ESP_LOGI(TAG, "Sensor after CCW edge: %d, Motor position %ld", sensorAngle, stepperPosition1);

        // int travelDistance = sensorValueforMechanicalZero - sensorAngle;
        // // we know we have to do a positve travel - so correct for overruns
        // if (travelDistance < 0)
        //     travelDistance += 4096;
        // MOVE_WAIT(travelDistance * STEPS_PER_SENSORCOUNT - 16);

        // // do the last 16 steps very slowly
        // for (int i = 0; i < 16; i++)
        //     MOVE_WAIT(1);


    findEdge(CW);
    sensorAngle = (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    int32_t stepperPosition2 = stepper->getCurrentPosition();
    ESP_LOGI(TAG, "Sensor after CW edge: %d, Motor position %ld", sensorAngle, stepperPosition2);

    // move to the middle position
    MOVETO_WAIT((stepperPosition1 + stepperPosition2) / 2);

    // we should be now at the mechanical zero position, so also reset the position in the driver
    stepper->setCurrentPosition(0);
    x = 0;
    _isMoving = false;

    sensorAngle = (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    ESP_LOGI(TAG, "Sensor Position is now %d and should be %d (delta = %d)", sensorAngle, sensorValueforMechanicalZero, abs(sensorAngle - sensorValueforMechanicalZero));
    ESP_LOGI(TAG, "Read and corrected Sensor position for Midpoint (mechanical 0) is %f", CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64)));

    // double pos = 0.0;
    // for (int i=0; i<16;i++){
    //     pos += 2.25;
    //     MOVETO_WAIT((long)(FMOD360(pos) / DEGREE_PER_STEP));
    //     ESP_LOGI(TAG, "Sensor read for position %.2f is %d", pos, (int)MEASURE_PRECISE_ANGLE(64));
    // }
    */
}

//void RotatorHW::measureMechanicalZero(int noOfMeasures)
int RotatorHW::measureMechanicalZero(std::function<void(int)> onProgress)
{
    MotionLock motionLock(motionMutex);
    unsigned long cwPosition = 0;
    unsigned long ccwPosition = 0;
    unsigned int sensorMechanicalZeroPosition;
    unsigned long preciseAngle;

    // we're about to move the motor
    _isMoving = true;
    // A fresh mechanical-zero measurement is a maintenance operation, not a
    // commanded move - disengage continuous holding until the next
    // gotoMechanicalZero()/put*Position() re-establishes a trusted target.
    _holdActive = false;

    // first to edge finding routines might not be so precisce - so skip from measurement
    findEdge(CW);
    findEdge(CCW);

    // loop through the number of requested measurements
    int noOfMeasures = 1;
    for (int i = 0; i < noOfMeasures; i++)
    {
        // find CW edge and store accumulated cwPosition read from Sensor
        findEdge(CW);
        preciseAngle = MEASURE_PRECISE_ANGLE(16);
        ESP_LOGI(TAG, "Sensor Position for CW edge is %ld, Stepper Position = %ld", preciseAngle, stepper->getCurrentPosition());
        cwPosition += preciseAngle;
        // find CCW edge and store accumulated cwPosition read from Sensor
        findEdge(CCW);
        preciseAngle = MEASURE_PRECISE_ANGLE(16);
        ESP_LOGI(TAG, "Sensor Position for CCW edge is %ld, Stepper Position = %ld", preciseAngle, stepper->getCurrentPosition());
        ccwPosition += preciseAngle;
    }
    // we're done with moving
    _isMoving = false;

    // average the positions
    cwPosition /= noOfMeasures;
    ccwPosition /= noOfMeasures;

    // correct cw position if we had an overrun
    if (cwPosition < ccwPosition)
        cwPosition += 4096;

    // calculate the sensor value for mechanical Zero
    sensorMechanicalZeroPosition = ((cwPosition + ccwPosition) / 2) % 4096;

    ESP_LOGI(TAG, "Averaged Sensor Position for CW edge is %ld", cwPosition);
    ESP_LOGI(TAG, "Averaged Sensor Position for CCW edge is %ld", ccwPosition);
    ESP_LOGI(TAG, "Sensor Value for Mechanical Zero Position is %d", sensorMechanicalZeroPosition);
    return sensorMechanicalZeroPosition;
}

void RotatorHW::setZeroPosSensorValue(int value)
{
    _zeroPosSensorValue = (int16_t)value;
    nvs_handle_t nvs;
    if (nvs_open("homing", NVS_READWRITE, &nvs) != ESP_OK)
    {
        ESP_LOGW(TAG, "Could not open NVS to persist zeroPosSensorValue=%d - kept in memory only", value);
        return;
    }
    esp_err_t err = nvs_set_i32(nvs, "zeroSensor", value);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Could not persist zeroPosSensorValue=%d to NVS: %s", value, esp_err_to_name(err));
    else
        ESP_LOGI(TAG, "Persisted zeroPosSensorValue=%d to NVS", value);
}

void RotatorHW::findEdge(bool dirCW)
{
    int32_t startPosition = stepper->getCurrentPosition();
    uint32_t probePosition = 0;
    for (int pos = 17; pos >= 0; pos--)
    {
        // probe the bit at position pos
        probePosition += (1 << pos);
        // direction of movement is determined by dirCW only - negative for CCW, positive for CW
        MOVETO_WAIT(startPosition);
        MOVETO_WAIT(startPosition + (dirCW ? 1 : -1) * probePosition);
        // ESP_LOGI(TAG, "probing position 0x%lX, Hallsensor=%d", probePosition, digitalRead(HALLSENSOR));
        //  if we're outside remove the bit - otherwise keep it
        if (digitalRead(HALLSENSOR))
            probePosition -= (1 << pos);
    }
    // ok, we found the coarse position, now make sure we're within the border by moving back in increments of 256
    // we want to do that slow - one Fullstep / sec
    /*
        stepper->setSpeedInHz(MICROSTEPS);
        while (digitalRead(HALLSENSOR))
            MOVE_WAIT((dirCW ? -256 : 256));

        // now in the other direction in increments of 1 until we hit the transition from 0 to 1 of the Hallsensor
        int count = 0;
        while (!digitalRead(HALLSENSOR))
        {
            MOVE_WAIT((dirCW ? 1 : -1));
            count++;
        }
        // restore normal speed
        stepper->setSpeedInHz(NORMAL_MOTOR_SPEED);
    */
    ESP_LOGI(TAG, "Probing position 0x%lX, Hallsensor=%d", startPosition - stepper->getCurrentPosition(), digitalRead(HALLSENSOR));
}

namespace
{
// Set true for the duration of a stressFullStepI2C() run; the auxiliary
// task below polls it instead of being vTaskDelete()'d from outside, so it
// always finishes its own current I2C read (and releases i2cMutex) before
// exiting, rather than being torn down mid-transaction.
volatile bool s_stressAuxRunning = false;

void stressAuxI2CTask(void *arg)
{
    RotatorHW *self = (RotatorHW *)arg;
    while (s_stressAuxRunning)
    {
        ESP_LOGI("StressTest", "aux: reading");
        self->measureRawAngle(1);
        ESP_LOGI("StressTest", "aux: read done");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}
} // namespace

int32_t RotatorHW::stressFullStepI2C(int32_t steps, int sampleCount, std::function<void(int)> onProgress)
{
    MotionLock motionLock(motionMutex);
    _holdActive = false; // full-step mode below leaves the reference untrustworthy
    ESP_LOGI(TAG, "Stress test starting: %ld full steps, %d-sample reads, aux I2C task every 1ms",
             (long)steps, sampleCount);

    s_stressAuxRunning = true;
    xTaskCreate(stressAuxI2CTask, "stressAux", 4096, this, 2, nullptr);

    stepper_driver.setMicrostepsPerStep(1);

    int32_t completed = 0;
    for (int32_t i = 0; i < steps; i++)
    {
        ESP_LOGI("StressTest", "step %ld/%ld: forwardStep", (long)(i + 1), (long)steps);
        stepper->forwardStep();
        ESP_LOGI("StressTest", "step %ld/%ld: forwardStep done, delay(20)", (long)(i + 1), (long)steps);
        delay(20);
        ESP_LOGI("StressTest", "step %ld/%ld: measuring", (long)(i + 1), (long)steps);
        double raw = MEASURE_PRECISE_ANGLE_DOUBLE(sampleCount);
        ESP_LOGI("StressTest", "step %ld/%ld: measured raw=%.1f", (long)(i + 1), (long)steps, raw);
        completed = i + 1;
        if (i % 10 == 0)
            onProgress(100 * i / steps);
    }

    s_stressAuxRunning = false;
    // Let the aux task see the flag and self-delete on its own next 1ms
    // wake, rather than racing its final measureRawAngle() against this
    // function returning and the caller possibly switching microsteps back.
    delay(50);

    stepper_driver.setMicrostepsPerStep(256);
    ESP_LOGI(TAG, "Stress test finished: %ld/%ld steps completed", (long)completed, (long)steps);
    return completed;
}

//void RotatorHW::calibrateAngleSensor(void)
RotatorHW::CalibrationResult RotatorHW::calibrateAngleSensor(std::function<void(int)> onProgress)
{
    // Sweep this many full motor revolutions and average the (wrapped) error
    // at each step index before fitting. This trades calibration time for
    // less read/motor noise in the fit; bump it if the reported residual
    // still looks noisy. Confirmed clean on the bench at 2 repeats (residual
    // RMS ~1 degree, consistent across two separate runs) with the full-step
    // motion below.
    //
    // Two other approaches were tried live and rejected - not because of
    // CAL_REPEATS, but because both replaced the full-step motion itself:
    // a variant that switched the driver to full-step mode as below but then
    // tried to rescale FastAccelStepper's position counter afterwards via one
    // large setCurrentPosition() jump corrupted that counter outright (its
    // internal 16-bit position reconstruction does not tolerate a jump that
    // size); a variant that avoided the rescale by moving MICROSTEPS pulses
    // at the normal driver resolution instead of switching to full-step mode
    // measured a materially worse fit even at a single repeat. Both are
    // symptoms of a deeper, still-unresolved cross-task reliability issue
    // between FastAccelStepper's move()/isRunning() and the 100 ms
    // angle_producer_task reading the same object - not something to keep
    // patching blind. The known, accepted cost of leaving that alone: this
    // full-step sweep runs the driver at a different microstep resolution
    // than FastAccelStepper's position counter assumes, so the counter
    // undercounts by up to 256x for the duration of the sweep and is left
    // that way afterwards - gotoMechanicalZero() must be re-run after any
    // calibration before trusting absolute position commands, which is
    // already required anyway (recalibrating shifts the sensor correction's
    // C0 offset, invalidating the stored mechanical-zero reference).
    constexpr int CAL_REPEATS = 2;

    MotionLock motionLock(motionMutex);
    _holdActive = false; // full-step sweep below leaves the reference untrustworthy

    // set driver to fullsteps
    stepper_driver.setMicrostepsPerStep(1);

    // Accumulate the *wrapped error against the ideal ramp* per step index,
    // not the raw counts themselves - averaging raw counts directly would
    // break near the 0/4096 wraparound whenever it falls inside the sweep.
    std::vector<double> avgErr(N_STEPS, 0.0);
    for (int r = 0; r < CAL_REPEATS; r++)
    {
        for (int i = 0; i < N_STEPS; i++)
        {
            // Fine-grained markers around each sub-operation, deliberately
            // at INFO (DEBUG is compiled out - CONFIG_LOG_MAXIMUM_LEVEL=3):
            // this loop is the one that has hung the device outright on
            // 4/4 live attempts (see memory/rotator_angle_cal_hang.md), at
            // different, unpredictable points each time, and the real
            // rotator has no UART console - the in-memory /log ring buffer
            // (LogBuffer.c, 200 lines) is the only way to see what the last
            // thing running was. Whichever of these markers is the last one
            // in the buffer after a hang narrows down which sub-operation
            // stopped returning.
            ESP_LOGI("Sensor Calibration", "step %d/%d: forwardStep", i + 1, N_STEPS);
            stepper->forwardStep();
            ESP_LOGI("Sensor Calibration", "step %d/%d: forwardStep done, delay(20)", i + 1, N_STEPS);
            delay(20);
            ESP_LOGI("Sensor Calibration", "step %d/%d: measuring (64-sample AS5600 average)", i + 1, N_STEPS);
            double raw = FMOD4096(MEASURE_PRECISE_ANGLE_DOUBLE(64));
            double ideal = 4096.0 * i / N_STEPS;
            double err = raw - ideal;
            err -= 4096.0 * std::round(err / 4096.0); // wrap to (-2048, 2048]
            avgErr[i] += err;
            ESP_LOGI("Sensor Calibration", "Rev %d/%d, Step: %3d, Sensor: %.1f", r + 1, CAL_REPEATS, i + 1, raw);
            onProgress(100 * (r * N_STEPS + i) / (CAL_REPEATS * N_STEPS));
            ESP_LOGI("Sensor Calibration", "step %d/%d: progress sent", i + 1, N_STEPS);
        }
    }
    std::vector<double> avgRaw(N_STEPS);
    for (int i = 0; i < N_STEPS; i++)
        avgRaw[i] = 4096.0 * i / N_STEPS + avgErr[i] / CAL_REPEATS;

    // how good were the coefficients already in effect, judged against this
    // fresh sweep? (uses C0/A/B as they stand before calibrateAngleSensorFinalize
    // overwrites them below)
    ResidualStats before = computeResidual(avgRaw);

    calibrateAngleSensorInit();
    for (int i = 0; i < N_STEPS; i++)
        calibrateAngleSensorStep(avgRaw[i]);
    calibrateAngleSensorFinalize();

    ResidualStats after = computeResidual(avgRaw);

    ESP_LOGI("Sensor Calibration", "C0 = %.4f", C0);
    for (int k = 1; k <= KMAX; k++)
    {
        ESP_LOGI("Sensor Calibration", "A%d = %.4f, B%d = %.4f", k, A[k], k, B[k]);
    }
    ESP_LOGI("Sensor Calibration", "Residual RMS: before=%.4f deg, after=%.4f deg (peak after=%.4f deg)",
             before.rmsDeg, after.rmsDeg, after.peakDeg);

    // store new values in configuration and persist them - previously these
    // were only written to the in-memory ConfigData (all setters called with
    // save=false and nothing ever called save() afterwards), so a
    // calibration run was silently lost on the next restart.
    auto &cfg = Configuration::getInstance();
    for (int k = 0; k <= KMAX; k++)
    {
        cfg.setA(k, A[k], false);
        cfg.setB(k, B[k], false);
    }
    cfg.setC0(C0, false);
    cfg.save();

    // set driver back to 256 microsteps
    stepper_driver.setMicrostepsPerStep(256);

    return {before.rmsDeg, after.rmsDeg, after.peakDeg};
}

void RotatorHW::calibrateAngleSensorInit(void)
{
    // reset all relevant parameters
    step_counter = 0;
    sum0 = 0;
    for (int k = 1; k <= KMAX; ++k)
    {
        sumC[k] = 0;
        sumS[k] = 0;
    }
}

void RotatorHW::calibrateAngleSensorStep(double sensor_raw)
{
    if (step_counter >= N_STEPS)
        return;

    // 1) ideal target value
    double ideal = 4096.0f * step_counter / (double)N_STEPS;

    // 2) error
    double e = sensor_raw - ideal;

    // 3) sum offsets
    sum0 += e;

    // 4) sum up harmonic parts, correlated against the *actually measured*
    // angle - not an angle accumulated from the step index. correctSensorReading()
    // evaluates cos(k*theta)/sin(k*theta) at the raw reading it is given, so the
    // fit has to use that same theta or the harmonics come out phase-rotated by
    // (this run's starting angle) * k as soon as a calibration run does not
    // happen to start at sensor_raw == 0. That is what produced a ~50 degree
    // "before" residual on a run started away from raw 0: the old fit's basis
    // was anchored to step 0 rather than to the sensor's own zero.
    double theta = 2.0 * M_PI * sensor_raw / 4096.0;
    for (int k = 1; k <= KMAX; ++k)
    {
        sumC[k] += e * cos(k * theta);
        sumS[k] += e * sin(k * theta);
    }

    step_counter++;
}

void RotatorHW::calibrateAngleSensorFinalize(void)
{
    if (step_counter < N_STEPS)
        return;

    // 1) Offset
    C0 = sum0 / (double)N_STEPS;

    // 2) Fourier-Koeffizienten (2/N-Normierung)
    for (int k = 1; k <= KMAX; k++)
    {
        A[k] = 2.0 * sumC[k] / N_STEPS;
        B[k] = 2.0 * sumS[k] / N_STEPS;
    }
}

void RotatorHW::setAngleCalCoefficients(double c0, const double a[KMAX + 1], const double b[KMAX + 1])
{
    C0 = c0;
    for (int k = 0; k <= KMAX; k++)
    {
        A[k] = a[k];
        B[k] = b[k];
    }

    auto &cfg = Configuration::getInstance();
    for (int k = 0; k <= KMAX; k++)
    {
        cfg.setA(k, A[k], false);
        cfg.setB(k, B[k], false);
    }
    cfg.setC0(C0, false);
    cfg.save();
}

RotatorHW::ResidualStats RotatorHW::computeResidual(const std::vector<double> &avgRaw)
{
    constexpr double DEG_PER_COUNT = 360.0 / 4096.0;
    double sumSq = 0.0;
    double peak = 0.0;
    for (int i = 0; i < N_STEPS; i++)
    {
        double ideal = 4096.0 * i / N_STEPS;
        double err = correctSensorReading(avgRaw[i]) - ideal;
        err -= 4096.0 * std::round(err / 4096.0); // wrap to (-2048, 2048]
        sumSq += err * err;
        peak = std::max(peak, std::fabs(err));
    }
    double rmsCounts = std::sqrt(sumSq / N_STEPS);
    return {rmsCounts * DEG_PER_COUNT, peak * DEG_PER_COUNT};
}

double RotatorHW::correctSensorReading(double sensorReading)
{
    double theta = 2.0f * M_PI * sensorReading / 4096.0f;
    double err = C0;
    for (int k = 1; k <= KMAX; ++k)
    {
        err += A[k] * cos(k * theta) + B[k] * sin(k * theta);
    }
    double smoothCorrected = sensorReading - err;

    // Full-step-resolution residual layered on top of the smooth model
    // above - see setFullStepTable()'s comment in the header for why this
    // exists. Indexed by which of the N_STEPS full steps smoothCorrected
    // estimates we are nearest - already a good-to-a-small-fraction-of-a-
    // full-step estimate, since it is exactly what refineToTarget()
    // converges position on - with linear interpolation between the two
    // neighbouring table entries for the fractional part. All-zero (a
    // no-op) until a table has been uploaded.
    const double countsPerStep = 4096.0 / N_STEPS;
    double bin = FMOD4096(smoothCorrected) / countsPerStep;
    int bin0 = ((int)bin) % N_STEPS;
    int bin1 = (bin0 + 1) % N_STEPS;
    double frac = bin - (int)bin;
    double fineErr = _fullStepTable[bin0] * (1.0 - frac) + _fullStepTable[bin1] * frac;

    return smoothCorrected - fineErr;
}

void RotatorHW::setFullStepTable(const float table[N_STEPS])
{
    for (int i = 0; i < N_STEPS; i++)
        _fullStepTable[i] = table[i];
    Configuration::getInstance().setFullStepTable(_fullStepTable);
}

void RotatorHW::getFullStepTable(float outTable[N_STEPS])
{
    for (int i = 0; i < N_STEPS; i++)
        outTable[i] = _fullStepTable[i];
}

RotatorHW::SensorSnapshot RotatorHW::getSensorSnapshot()
{
    uint16_t raw = readAngleSafe();
    return {
        raw,
        correctSensorReading(raw),
        getStepPositionSafe(),
        !digitalRead(HALLSENSOR), // active-low
    };
}

RotatorHW::SensorDiagnostics RotatorHW::getSensorDiagnostics()
{
    // Same i2cMutex as readAngleSafe() - see that function's comment for why
    // as5600 access must not interleave across tasks.
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    uint8_t agc = as5600.readAGC();
    uint16_t magnitude = as5600.readMagnitude();
    bool detected = as5600.detectMagnet();
    bool tooStrong = as5600.magnetTooStrong();
    bool tooWeak = as5600.magnetTooWeak();
    xSemaphoreGive(i2cMutex);
    return {agc, magnitude, detected, tooStrong, tooWeak};
}

void RotatorHW::jogMicrosteps(int32_t microsteps)
{
    if (microsteps == 0)
        return;
    // Bypasses _holdTargetSteps bookkeeping on purpose (this is a raw debug
    // jog, not a commanded move) - but it still issues a real stepper->move(),
    // so it needs the same motionMutex as every other such call, or it could
    // race holdTask()'s own stepper->move() for the same FastAccelStepper
    // object while holding is active. Disengaging holding here too (not just
    // locking against it) is not optional: a calibration script that issues
    // many jogMicrosteps() calls in a row (e.g. scripts/camera_angle_sweep.py)
    // never updates _holdTargetSteps, so a still-active hold task would keep
    // trying to correct back toward whatever target was active before the
    // sweep started, physically fighting the sweep's own motion the whole
    // time it runs - live-caught corrupting a full calibration sweep's data
    // this way (2026-09-11) before this fix.
    MotionLock motionLock(motionMutex);
    _holdActive = false;
    _isMoving = true;
    MOVE_WAIT(microsteps);
    _isMoving = false;
}

int32_t RotatorHW::alignToFullStep()
{
    // forwardStep()+delay() - the same choice calibrateAngleSensor() makes
    // for the identical microstep-mode switch, and for the same reason: a
    // queued move()+isRunning() poll measurably degraded results there when
    // tried, and this is the same driver-microstepping change applied to
    // the same live motor.
    MotionLock motionLock(motionMutex);
    _isMoving = true;
    _holdActive = false; // full-step switch below leaves the reference untrustworthy
    stepper_driver.setMicrostepsPerStep(1);
    stepper->forwardStep();
    delay(20);
    stepper_driver.setMicrostepsPerStep(256);
    _isMoving = false;
    return getStepPositionSafe();
}

double RotatorHW::measureRawAngle(int samples)
{
    if (samples <= 1)
        return readAngleSafe();
    return MEASURE_PRECISE_ANGLE_DOUBLE(samples);
}

void RotatorHW::refineToTarget(long targetMotorSteps)
{
    // Closed-loop refinement after the open-loop MOVETO_WAIT above already
    // got close - see RotatorHW.h's comment and CALIBRATION_FINDINGS.md.
    // Everything here is OUTPUT-shaft degrees; DEGREE_PER_STEP already
    // converts 1:1 between raw microsteps and output degrees (the 10:1
    // reduction is baked into its definition above).
    constexpr double kP = 0.9;
    constexpr double kI = 0.15;
    // Only accumulate the integral term while the error is already small -
    // without this, the large error from a long move winds up the integral
    // and then dominates the proportional term for many iterations after
    // the error is already small, causing sustained overshoot. Found live
    // with scripts/pi_position_control.py before this port; not a
    // hardware issue, standard conditional-integration anti-windup.
    constexpr double integralBandDeg = 0.05;
    constexpr double toleranceDeg = 0.01;
    constexpr int maxIterations = 20;
    constexpr int sensorSamples = 8;
    constexpr double periodDeg = 36.0; // one AS5600 revolution = 360/10 output degrees

    double targetOutputDeg = (double)targetMotorSteps * DEGREE_PER_STEP;

    // Seed the filter from a real measurement, wrapped to whichever
    // physical revolution is nearest the target - safe as long as the
    // open-loop move above landed within half an AS5600 revolution (18 deg
    // output) of the target, true under normal operation. A larger
    // open-loop miss (stall, lost steps) would be misread as a small error
    // in the wrong direction - the same blind spot every modulo-based
    // unwrap in this project has; nothing here can detect that case.
    double correctedCounts = correctSensorReading(MEASURE_PRECISE_ANGLE_DOUBLE(sensorSamples));
    double wrappedDeg = FMOD4096(correctedCounts) * periodDeg / 4096.0;
    double diff0 = wrappedDeg - targetOutputDeg;
    diff0 -= periodDeg * std::round(diff0 / periodDeg);

    AngleKalman1D kf;
    kf.x = targetOutputDeg + diff0;
    kf.P = 1e-4;
    kf.Q = 1e-7;
    kf.R = 1e-6;

    double integral = 0.0;
    int iterationsUsed = 0;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        double error = targetOutputDeg - kf.x;
        if (std::fabs(error) < toleranceDeg)
            break;
        iterationsUsed = iter + 1;

        if (std::fabs(error) < integralBandDeg)
            integral += error;
        double controlDeg = kP * error + kI * integral;
        int32_t controlMicrosteps = (int32_t)std::lround(controlDeg / DEGREE_PER_STEP);
        if (controlMicrosteps == 0)
            controlMicrosteps = (controlDeg > 0.0) ? 1 : -1;

        MOVE_WAIT(controlMicrosteps);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        double commandedDeg = controlMicrosteps * DEGREE_PER_STEP;
        kf.predict(commandedDeg);

        correctedCounts = correctSensorReading(MEASURE_PRECISE_ANGLE_DOUBLE(sensorSamples));
        wrappedDeg = FMOD4096(correctedCounts) * periodDeg / 4096.0;
        double diff = wrappedDeg - kf.x;
        diff -= periodDeg * std::round(diff / periodDeg);
        double measuredDeg = kf.x + diff;
        kf.update(measuredDeg);

        ESP_LOGI(TAG, "refineToTarget iter %d: control=%ld us (%.4f deg) measured=%.4f estimate=%.4f error=%.4f deg",
                 iter + 1, (long)controlMicrosteps, commandedDeg, measuredDeg, kf.x, targetOutputDeg - kf.x);
    }

    ESP_LOGI(TAG, "refineToTarget done after %d correction(s): estimate=%.4f deg (target %.4f, error %.4f deg)",
             iterationsUsed, kf.x, targetOutputDeg, targetOutputDeg - kf.x);
}

// Continuous counterpart to refineToTarget(): the same control law (an
// AngleKalman1D running estimate plus conditional-integration PI - see that
// function's comments for why each piece is there), but instead of iterating
// a bounded number of times right after an open-loop move and then stopping,
// this runs at the same fixed 150ms cadence for as long as the rotator sits
// at a settled target (potentially hours) - Franz's point: a PI controller
// needs an equidistant sample clock to mean anything, not an occasional
// poll-and-nudge. kf.x is deliberately persistent across cycles (not
// re-measured from scratch each time) so it keeps doing what a Kalman filter
// is for - averaging down AS5600 read noise (~0.05-0.1 deg, see
// CALIBRATION_FINDINGS.md) over many cycles rather than reacting to every
// single noisy sample. That, not an explicit deadband, is what keeps this
// from chattering: a lone noisy sample nudges kf.x only by the filter's own
// (tuned, small) Kalman gain, and the resulting commanded correction is only
// ever a whole number of microsteps - genuine, sustained drift eventually
// pushes kf.x far enough from target to round to a real correction; noise
// alone mostly rounds to zero and is simply absorbed into the estimate.
void RotatorHW::holdTask(void *arg)
{
    RotatorHW *self = (RotatorHW *)arg;

    // Same 10ms-per-sample averaging as MEASURE_PRECISE_ANGLE_DOUBLE, just
    // callable from a static member function (that macro references
    // readAngleSafe() unqualified, relying on an implicit `this` this
    // function doesn't have).
    auto measurePreciseAngle = [self](int count) -> double {
        int32_t angle = self->readAngleSafe();
        int32_t start = angle;
        for (int i = 1; i < count; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            int32_t delta = (self->readAngleSafe() - start + 6144) % 4096 - 2048;
            angle += start + delta;
        }
        return (double)angle / (double)count;
    };

    // Identical constants to refineToTarget() - this is the same controller,
    // just never told to stop; reusing its already-live-tuned gains rather
    // than inventing new ones for a "slow" variant.
    constexpr double kP = 0.9;
    constexpr double kI = 0.15;
    constexpr double integralBandDeg = 0.05;
    constexpr double periodDeg = 36.0;
    constexpr int sensorSamples = 8;
    const TickType_t cyclePeriod = pdMS_TO_TICKS(150);

    AngleKalman1D kf;
    double integral = 0.0;
    bool haveEstimate = false;
    long lastTargetSteps = 0;

    while (true)
    {
        vTaskDelay(cyclePeriod);

        if (!self->_holdActive)
        {
            // Reset so the next hold session always re-seeds from a fresh
            // measurement (below) instead of resuming a stale estimate.
            haveEstimate = false;
            continue;
        }

        // Never block: if a commanded move, homing sweep, or calibration
        // routine currently owns the motor, just skip this cycle - holding
        // is a background nicety, not worth adding latency to a
        // user-commanded operation for. The fixed cyclePeriod above still
        // elapses regardless, so the clock itself never skips a beat even
        // though an individual cycle's work sometimes does.
        if (xSemaphoreTake(self->motionMutex, 0) != pdTRUE)
            continue;

        if (!self->_holdActive) // could have been cleared while waiting for the tick above
        {
            xSemaphoreGive(self->motionMutex);
            haveEstimate = false;
            continue;
        }

        long targetSteps = self->_holdTargetSteps;
        double targetOutputDeg = (double)targetSteps * DEGREE_PER_STEP;

        if (!haveEstimate || targetSteps != lastTargetSteps)
        {
            // (Re)seed exactly like refineToTarget()'s startup: a real
            // measurement, wrapped to whichever AS5600 revolution is
            // nearest the target.
            double correctedCounts = self->correctSensorReading(measurePreciseAngle(sensorSamples));
            double wrappedDeg = FMOD4096(correctedCounts) * periodDeg / 4096.0;
            double diff0 = wrappedDeg - targetOutputDeg;
            diff0 -= periodDeg * std::round(diff0 / periodDeg);
            kf.x = targetOutputDeg + diff0;
            kf.P = 1e-4;
            kf.Q = 1e-7;
            kf.R = 1e-6;
            integral = 0.0;
            haveEstimate = true;
            lastTargetSteps = targetSteps;
            xSemaphoreGive(self->motionMutex);
            continue; // first real correction happens next cycle, against a seeded estimate
        }

        double error = targetOutputDeg - kf.x;
        if (std::fabs(error) < integralBandDeg)
            integral += error;
        double controlDeg = kP * error + kI * integral;

        // The PI state above (integral, and kf below) updates every cycle,
        // unconditionally - that is what makes this a continuously-running
        // controller rather than a periodic poll. Whether to actually move
        // the motor is a separate question: one microstep is only ~0.00035
        // deg, finer than the Kalman-filtered estimate's own steady-state
        // noise (live-measured: kf.x holds within roughly +/-0.001 deg once
        // settled) - so a naive "round to the nearest microstep and go"
        // rounds some nonzero correction on nearly every single cycle,
        // forever, chasing residual filter noise rather than real drift.
        // toleranceDeg (same figure as refineToTarget()'s own convergence
        // tolerance, confirmed comfortably above that noise floor) gates
        // actual actuation only; a cycle below it still updates kf/integral
        // and simply predicts a zero commanded step, so no correction is
        // ever lost - it just accumulates until it is large enough to be a
        // real, not illusory, correction.
        constexpr double toleranceDeg = 0.01;
        int32_t controlMicrosteps = (std::fabs(controlDeg) >= toleranceDeg)
                                         ? (int32_t)std::lround(controlDeg / DEGREE_PER_STEP)
                                         : 0;

        if (controlMicrosteps != 0)
        {
            long newSteps = self->getStepPositionSafe() + controlMicrosteps;
            if (self->withinMotionLimit(newSteps))
            {
                self->stepper->move(controlMicrosteps);
                while (self->stepper->isRunning())
                    vTaskDelay(pdMS_TO_TICKS(5));
            }
            else
            {
                ESP_LOGW(TAG, "hold: drift %.4f deg from target %.4f, correction would exceed the cable limit - skipped",
                         error, targetOutputDeg);
                controlMicrosteps = 0; // nothing actually moved
            }
        }

        double commandedDeg = controlMicrosteps * DEGREE_PER_STEP;
        kf.predict(commandedDeg);

        double correctedCounts = self->correctSensorReading(measurePreciseAngle(sensorSamples));
        double wrappedDeg = FMOD4096(correctedCounts) * periodDeg / 4096.0;
        double diff = wrappedDeg - kf.x;
        diff -= periodDeg * std::round(diff / periodDeg);
        double measuredDeg = kf.x + diff;
        kf.update(measuredDeg);

        if (controlMicrosteps != 0)
            ESP_LOGI(TAG, "hold: control=%ld us (%.4f deg) estimate=%.4f target=%.4f error=%.4f deg",
                     (long)controlMicrosteps, commandedDeg, kf.x, targetOutputDeg, error);

        xSemaphoreGive(self->motionMutex);
    }
}

void RotatorHW::putHalt()
{
    MotionLock motionLock(motionMutex);
    // Halt means "stop all rotator motion now" (Alpaca semantics), not just
    // abort an in-progress commanded move - so it also disengages continuous
    // holding (holdTask()) until the next commanded move or homing.
    _holdActive = false;
    // halt only if we're moving
    if (_isMoving)
    {
        // stop moving controlled and wait for stopped Motor
        stepper->stopMove();
        WAIT_FOR_STOPPED_MOTOR;
        // and mark as we're not moving anymore
        _isMoving = false;
    }
}

double RotatorHW::getPosition()
{
    // Combines the stepper's own reliable, unambiguous coarse position
    // (getMechanicalPosition(), no AS5600 involved) with a fresh AS5600
    // reading used only as a SMALL corrective nudge - not, as the previous
    // version of this function did, as an independently reconstructed
    // "which 36 degree bin am I in" value referenced against
    // _zeroPosSensorValue. That constant has no defined relationship to
    // getMechanicalPosition()'s own bin boundaries (stepPosition == 0), so
    // the two terms could disagree by anywhere up to nearly a full AS5600
    // revolution depending on exactly where the last homing happened to
    // land; live-measured discrepancy against an independent camera
    // reference was ~0.25-0.7 deg before this fix. The fine term below is
    // instead computed as the shortest signed distance from the AS5600
    // phase getMechanicalPosition() itself implies - the same
    // ideal-vs-measured, wrap-to-nearest pattern used throughout this file
    // (see MEASURE_PRECISE_ANGLE_DOUBLE's own "-start+6144)%4096-2048"
    // idiom) and in RotatorHW::refineToTarget() - so it can only ever
    // contribute a small correction, never a bin-sized jump.
    double mechanicalPos = getMechanicalPosition();
    double idealCounts = 4096.0 * fmod(mechanicalPos, 36.0) / 36.0;
    double correctedCounts = correctSensorReading(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    double fineCorrectionDeg = (FMOD4096(correctedCounts - idealCounts + 2048.0) - 2048.0) / 4096.0 * 36.0;
    // Reverse (Alpaca Rotator.Reverse) and the configured Nominal Direction
    // (Franz's per-telescope "which way is positive" setting) together flip
    // the sign of the mechanical-to-Position mapping, independent of the
    // Sync() offset - see getDirection()'s comment, and
    // putRelativePosition()/putAbsolutePosition()/putMechanicalPosition()/
    // syncPosition() for the matching inverse/forward transforms. dir is 1
    // exactly when getDirection() is true (both flags reduce to the
    // extensively-tested default case: Nominal Direction clockwise, Reverse
    // false).
    const double dir = getDirection() ? 1.0 : -1.0;
    return FMOD360(dir * (mechanicalPos + fineCorrectionDeg) + positionOffsetToMechanicalPosition);
}

double RotatorHW::getMechanicalPosition()
{
    // get a snapshot of the actual position based on the stepper position
    return (double)FMOD360(getStepPositionSafe() * DEGREE_PER_STEP);

}

double RotatorHW::getStepSizeDegrees()
{
    // Reported per Franz's judgment call: the practically meaningful "step
    // size" for a closed-loop rotator is the resolution of the sensor the
    // loop actually regulates against (refineToTarget()'s AS5600 feedback),
    // not the driver's raw microstep pitch, which is finer than anything
    // the control loop can distinguish in a single reading. One AS5600
    // count, at the motor shaft, is 1/10th of that on the output shaft.
    return 360.0 / SENSORCOUNT_STEPS_PER_ROTATION / 10.0;
}

void RotatorHW::putNominalClockwise(bool clockwise)
{
    _nominalClockwise = clockwise;
    // Application-specific (which way this particular telescope/mount calls
    // "positive"), not a device calibration value - see Configuration.hpp's
    // ConfigData::nominalClockwise and getDirection()'s comment. Unlike
    // Reverse, which Alpaca clients set per session and this firmware never
    // persists, this is meant to be set once per telescope and survive
    // reboots.
    Configuration::getInstance().setNominalClockwise(clockwise);
}

bool RotatorHW::legalMotorStepsForAngle(double wrappedMechDeg, long *outSteps)
{
    long currentSteps = getStepPositionSafe();
    double currentDeg = currentSteps * DEGREE_PER_STEP;
    double baseDelta = wrappedMechDeg - currentDeg;
    baseDelta -= 360.0 * std::round(baseDelta / 360.0); // shortest candidate, (-180, 180]

    // Try the shortest candidate first, then its two neighbors a full turn
    // either way, which land at the exact same physical orientation but
    // require more travel - on this rotator (MOTION_LIMIT_DEG barely more
    // than one full turn) that longer path can still be legal even when the
    // shortest one exceeds the limit, e.g. current=185 deg, target=0 deg:
    // shortest is +175 (to raw 360, illegal), but -185 (to raw 0) is legal
    // and physically identical. Live-caught during testing, not
    // hypothetical.
    const double candidateOffsetsDeg[] = {0.0, -360.0, 360.0};
    long shortest = currentSteps + std::lround(baseDelta / DEGREE_PER_STEP);
    for (double offsetDeg : candidateOffsetsDeg)
    {
        long candidate = currentSteps + std::lround((baseDelta + offsetDeg) / DEGREE_PER_STEP);
        if (withinMotionLimit(candidate))
        {
            *outSteps = candidate;
            return true;
        }
    }
    *outSteps = shortest;
    return false;
}

bool RotatorHW::withinMotionLimit(long targetSteps)
{
    return std::labs(targetSteps) <= MOTION_LIMIT_STEPS;
}

bool RotatorHW::putRelativePosition(double position)
{
    MotionLock motionLock(motionMutex);
    // See getPosition()'s comment on `dir` - Reverse flips the mechanical<->
    // Position mapping's sign, independent of the Sync() offset. No-op when
    // not reversed.
    const double dir = getDirection() ? 1.0 : -1.0; // see getPosition()'s comment on `dir`

    // calculate new positions
    double newTargetPosition = FMOD360(getPosition() + position);
    double desiredMechDeg = FMOD360(dir * (newTargetPosition - positionOffsetToMechanicalPosition));
    long targetMotorPosition;
    if (!legalMotorStepsForAngle(desiredMechDeg, &targetMotorPosition))
    {
        ESP_LOGW(TAG, "Refusing relative move by %f deg: would reach %f deg from mechanical zero, outside +/-%.0f deg cable limit",
                 position, targetMotorPosition * DEGREE_PER_STEP, MOTION_LIMIT_DEG);
        return false;
    }
    _targetPosition = newTargetPosition;

    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    refineToTarget(targetMotorPosition);
    _isMoving = false;
    // Target reached - hand off to continuous holding (holdTask()) until the
    // next commanded move, Halt, or maintenance routine.
    _holdTargetSteps = targetMotorPosition;
    _holdActive = true;
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE(64)));
    return true;
}

bool RotatorHW::putAbsolutePosition(double position)
{
    MotionLock motionLock(motionMutex);
    const double dir = getDirection() ? 1.0 : -1.0; // see getPosition()'s comment on `dir`

    // set new position
    double newTargetPosition = FMOD360(position);
    double desiredMechDeg = FMOD360(dir * (newTargetPosition - positionOffsetToMechanicalPosition));
    long targetMotorPosition;
    if (!legalMotorStepsForAngle(desiredMechDeg, &targetMotorPosition))
    {
        ESP_LOGW(TAG, "Refusing MoveAbsolute to %f deg: would reach %f deg from mechanical zero, outside +/-%.0f deg cable limit",
                 position, targetMotorPosition * DEGREE_PER_STEP, MOTION_LIMIT_DEG);
        return false;
    }
    _targetPosition = newTargetPosition;

    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    refineToTarget(targetMotorPosition);
    _isMoving = false;
    _holdTargetSteps = targetMotorPosition;
    _holdActive = true;
    int sensorPosition = (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d, Delta %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), sensorPosition, sensorPosition - (int)EXPECTED_SENSORVALUE(getMechanicalPosition()));
    return true;
}

bool RotatorHW::putMechanicalPosition(double position)
{
    MotionLock motionLock(motionMutex);
    const double dir = getDirection() ? 1.0 : -1.0; // see getPosition()'s comment on `dir`

    // MoveMechanical's argument is already a wrapped mechanical angle - no
    // dir/offset transform needed to find the motor target itself.
    long targetMotorPosition;
    if (!legalMotorStepsForAngle(FMOD360(position), &targetMotorPosition))
    {
        ESP_LOGW(TAG, "Refusing MoveMechanical to %f deg: would reach %f deg from mechanical zero, outside +/-%.0f deg cable limit",
                 position, targetMotorPosition * DEGREE_PER_STEP, MOTION_LIMIT_DEG);
        return false;
    }
    // _targetPosition (the Position-space equivalent TargetPosition
    // reports) needs the dir/offset transform even though the move itself
    // doesn't.
    _targetPosition = FMOD360(dir * position + positionOffsetToMechanicalPosition);
    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    refineToTarget(targetMotorPosition);
    _isMoving = false;
    _holdTargetSteps = targetMotorPosition;
    _holdActive = true;
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE(64)));
    return true;
}

void RotatorHW::syncPosition(double position)
{
    const double dir = getDirection() ? 1.0 : -1.0; // see getPosition()'s comment on `dir`
    positionOffsetToMechanicalPosition = position - dir * getMechanicalPosition();
}