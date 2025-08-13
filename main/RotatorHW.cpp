#include "RotatorHW.h"
#include "Configuration.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include <vector>
#include <cmath>

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
            angle += as5600.readAngle();         \
            vTaskDelay(10 / portTICK_PERIOD_MS); \
        }                                        \
        angle / count;                           \
    })

#define MEASURE_PRECISE_ANGLE_DOUBLE(count) (                                  \
    {                                                                          \
        int32_t angle = as5600.readAngle();                                    \
        int32_t start = angle;                                                 \
        for (int i = 1; i < count; i++)                                        \
        {                                                                      \
            vTaskDelay(10 / portTICK_PERIOD_MS);                               \
            int32_t delta = (as5600.readAngle() - start + 6144) % 4096 - 2048; \
            angle += start + delta;                                            \
        }                                                                      \
        (double)angle / (double)count;                                         \
    })

#define FMOD360(x) (fmod(fmod(x, 360) + 360, 360))
#define FMOD4096(x) (fmod(fmod(x, 4096) + 4096, 4096))
#define EXPECTED_SENSORVALUE(x) ((sensorValueforMechanicalZero + int(4096.0 * fmod(x, 36.0) / 36.0)) % 4096)
#define CORRECTED_SENSORVALUE(x) (correctSensorReading(x))

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
const double SCALE_S = 4096.0f / (2.0f * M_PI);
// const long unsigned int EDGE_STEPS = 256 * 256;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const unsigned char RUN_CURRENT_PERCENT = 100;

RotatorHW &RotatorHW::getInstance()
{
    static RotatorHW instance;
    return instance;
}

RotatorHW::RotatorHW()
    : _isMoving(false), _isReverse(false), _isClockwise(true), _targetPosition(0), 
      serial_stream(Serial1)
{
    auto &cfg = Configuration::getInstance();

    // set Fourier coefficients from config
    for (int k = 0; k <= KMAX; k++)
    {
        delta_phi[k] = 2.0 * M_PI * k / N_STEPS;
        A[k] = cfg.getA(k);
        B[k] = cfg.getB(k);
    }
    C0 = cfg.getC0();

    // EKF values - later from config file
    Q = 2e-6f;
    R = 1.0f;
    x = 0;
    P = 1.0f;

    _zeroPosSensorValue = 571;
    _zeroPosSensorOffset = 571.0;

    sensorValueforMechanicalZero = 3747;
    positionOffsetToMechanicalPosition = 20.0;
    sensorCorrectionAmplitude = 0.003606;
    sensorCorrectionPhase = -0.609041;
}

void RotatorHW::begin()
{
    ESP_LOGI(TAG, "Intializing Rotator HW");
    // Setup the TMC2209 driver, 250mA, Cool Step with Enable pin
    stepper_driver.setup(serial_stream, 250000, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
    stepper_driver.setHardwareEnablePin(ENABLE_PIN);
    stepper_driver.setRMSCurrent(250, 0.11, 0.2);
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver.enableCoolStep();
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

    ESP_LOGI(TAG, "Finished intializing Rotator HW");

    /*
        for (;;)
        {
            // double angle = 360.0 * esp_random() / UINT32_MAX;
            // putAbsolutePosition(angle);
            // delay(1000);
            int32_t absolutPosition = 4000.0 * 256.0 * (double)esp_random() / UINT32_MAX;
            x = 2.0 * M_PI * absolutPosition / 4000.0 / 256;
            P += Q;
            // predict sensor angle
            double s_pred = FMOD4096(_zeroPosSensorOffset + x * 10.0 * 4096.0 / 2.0 / M_PI);
            double err = C0;
            for (int k = 1; k <= KMAX; ++k)
                err += A[k] * cos(k * 10 * x) + B[k] * sin(k * 10 * x);

            double h = FMOD4096(s_pred + err);

            // Jakobian
            double H = 4096.0 / 2.0 / M_PI;
            for (int k = 1; k <= KMAX; ++k)
                H += -A[k] * sin(k * 10 * x) + B[k] * cos(k * 10 * x);

            // ekf_predict(absolutPosition - stepper->getCurrentPosition());

            MOVETO_WAIT(absolutPosition);

            for (int i=0; i<128; i++)
            {
                double y = as5600.readAngle() - h;
                //double y = MEASURE_PRECISE_ANGLE_DOUBLE(64) - h;
                double S = H * P * H + R;

                double K = P * H / S;
                x = x + K * y;
                P = (1.0 - K * H) * P;
            }

            //    double sensorValue = MEASURE_PRECISE_ANGLE_DOUBLE(64);
            // ekf_update(sensorValue);
            ESP_LOGI("ekf", "Stepper: %8.3fdeg, EKF: %8.3fdeg, Raw Sensor: %d, P: %.8f", 360.0 * absolutPosition / 4000.0 / 256, 360.0 * x / 2.0 / M_PI, as5600.readAngle(), P);
        }
        */
}

void RotatorHW::gotoMechanicalZero()
{
    ESP_LOGI(TAG, "Goto Mechanical Zero");
    // move one full rotation CCW
    _isMoving = true;
    stepper->moveTo(-STEPS_PER_ROTATION / 2);
    bool zeroFound = true;
    while (digitalRead(HALLSENSOR))
    {
        if (!stepper->isRunning())
        {
            ESP_LOGW(TAG, "Index not found on CCW move!");
            zeroFound = false;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (!zeroFound)
    {
        stepper->moveTo(STEPS_PER_ROTATION);
        while (digitalRead(HALLSENSOR))
        {
            if (!stepper->isRunning())
            {
                ESP_LOGW(TAG, "Index not found on CW move!");
                break;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    // stop moving and wait for stopped Motor
    int32_t moveBack = stepper->stepsToStop();
    stepper->stopMove();
    WAIT_FOR_STOPPED_MOTOR;
    if (zeroFound)
    {
        // found it on CCW move
        MOVE_WAIT(moveBack);
    }
    else
    {
        // we did a full turn CW, so move back
        MOVE_WAIT(-moveBack);
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    int16_t searchDirection;
    int32_t deltaSensorPos;
    // allright - we're close to the sensor position, calculate the difference in steps
    double startPos = CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    if (zeroFound)
    {
        searchDirection = -1;
        deltaSensorPos = -FMOD4096(startPos - _zeroPosSensorValue - 3) * STEPS_PER_SENSORCOUNT;
    }
    else
    {
        searchDirection = 1;
        deltaSensorPos = FMOD4096(_zeroPosSensorValue - startPos - 3) * STEPS_PER_SENSORCOUNT;
    }

    ESP_LOGI("TAG", "StartPos=%.1f, 571, deltaSensorPos = %ld", startPos, deltaSensorPos);
    MOVE_WAIT(deltaSensorPos);
    int32_t firstEdge = 0;
    int32_t secondEdge = 0;
    int status = 0;
    for (int i = 0; i < 300 && status < 2; i++)
    {
        int angle = CORRECTED_SENSORVALUE(as5600.readAngle());
        if (status == 0 && angle == _zeroPosSensorValue)
        {
            status = 1;
            firstEdge = stepper->getCurrentPosition();
            ESP_LOGI("TAG", "Step: %d Rising edge position = %ld, Sensor = %d", i, firstEdge, angle);
        }
        if (status == 1 && angle == _zeroPosSensorValue + searchDirection)
        {
            status = 2;
            secondEdge = stepper->getCurrentPosition();
            ESP_LOGI("TAG", "Step: %d Falling edge position = %ld, Sensor = %d", i, secondEdge, angle);
        }

        MOVE_WAIT(searchDirection);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI("TAG", "Step: %i - Angle: %i", i, angle);
    }
    MOVETO_WAIT((firstEdge + secondEdge) / 2);
    _zeroPosSensorOffset = CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(256));

    ESP_LOGI("TAG", "Sensor at mechanical Zero = %f, Motor position before reset = %ld", _zeroPosSensorOffset, stepper->getCurrentPosition());
    stepper->setCurrentPosition(0);

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
    unsigned long cwPosition = 0;
    unsigned long ccwPosition = 0;
    unsigned int sensorMechanicalZeroPosition;
    unsigned long preciseAngle;

    // we're about to move the motor
    _isMoving = true;

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

//void RotatorHW::calibrateAngleSensor(void)
void RotatorHW::calibrateAngleSensor(std::function<void(int)> onProgress)
{
    // set driver to fullsteps
    stepper_driver.setMicrostepsPerStep(1);

    calibrateAngleSensorInit();
    for (int i = 0; i < N_STEPS; i++)
    {
        // one fullstep, measure sensor angle and send it to our algorithm computing the Fouries coefficients
        stepper->forwardStep();
        delay(20);
        calibrateAngleSensorStep(MEASURE_PRECISE_ANGLE_DOUBLE(64));
        ESP_LOGI("Sensor Calibration", "Step: %3d, Sensor: %4d", i + 1, as5600.readAngle());
        onProgress(100 * i / N_STEPS);
    }
    calibrateAngleSensorFinalize();
    ESP_LOGI("Sensor Calibration", "C0 = %.4f", C0);
    for (int k = 1; k <= KMAX; k++)
    {
        ESP_LOGI("Sensor Calibration", "A%d = %.4f, B%d = %.4f", k, A[k], k, B[k]);
    }

    // store new values in configuration
    auto &cfg = Configuration::getInstance();
    for (int k = 0; k <= KMAX; k++)
    {
        cfg.setA(k, A[k], false);
        cfg.setB(k, B[k], false);
    }
    cfg.setC0(C0, false);

    // set driver back to 256 microsteps
    stepper_driver.setMicrostepsPerStep(256);
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
        phi[k] = 0;
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

    // 4) sum up harmonic parts 
    for (int k = 1; k <= KMAX; ++k)
    {
        double c = cos(phi[k]);
        double s = sin(phi[k]);
        sumC[k] += e * c;
        sumS[k] += e * s;
        phi[k] += delta_phi[k];
        if (phi[k] >= 2.0f * M_PI)
            phi[k] -= 2.0f * M_PI;
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

double RotatorHW::correctSensorReading(double sensorReading)
{
    double theta = 2.0f * M_PI * sensorReading / 4096.0f;
    double err = C0;
    for (int k = 1; k <= KMAX; ++k)
    {
        err += A[k] * cos(k * theta) + B[k] * sin(k * theta);
    }
    return sensorReading - err;
}

// 1) Forward model prediction
double RotatorHW::h_meas(double x)
{
    double x_phase = fmodf(x, 2.0f * M_PI);
    double s_pred = SCALE_S * x;
    double err = C0;
    for (int k = 1; k <= KMAX; ++k)
    {
        err += A[k] * cos(k * x) + B[k] * sin(k * x);
    }
    return s_pred + err;
}

// 2) Jacobian dh/dx
double RotatorHW::H_jacobian(double x)
{
    double H = SCALE_S;
    for (int k = 1; k <= KMAX; ++k)
    {
        H += -A[k] * k * sin(k * x) + B[k] * k * cos(k * x);
    }
    return H;
}

void RotatorHW::ekf_predict(int32_t delta_steps)
{
    // convert to rad
    double u = 2 * M_PI * delta_steps / (4000.0f * 256.0f);
    double sensorOffsetRad = 2.0f * M_PI * _zeroPosSensorOffset / 4096.0f;
    // update state
    x = fmod(x + u + sensorOffsetRad, 2.0f * M_PI);
    P = P + Q;
    ESP_LOGI("------", "x after prediction: %8.3f", 360.0 * x / 2.0 / M_PI);
}

void RotatorHW::ekf_update(double s_raw)
{
    // predict measurement & Jacobian
    // double h  = h_meas(x);              // in sensor readings
    double h = FMOD4096(409.6 * x / 2.0f / M_PI);
    double H = H_jacobian(x);
    // innovation and innovation covarianz
    double y = s_raw - h;
    double S = H * P * H + R;
    // Kalman-gain
    double K = P * H / S;
    // state and covariance update
    x = x + K * y;
    x = fmod(x, 2.0f * M_PI);
    P = (1.0f - K * H) * P;
    ESP_LOGI("------", "h (Sensor Digits):  %7.2f, y: %7f", h, y);
}

void RotatorHW::putHalt()
{
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
    // get a snapshot of the actual position based on the stepper position and convert it to actual position
    //return FMOD360(stepper->getCurrentPosition() * DEGREE_PER_STEP + positionOffsetToMechanicalPosition);
    return 36 * std::floor(FMOD360(stepper->getCurrentPosition() * DEGREE_PER_STEP) / 36 ) +  FMOD4096(correctSensorReading(MEASURE_PRECISE_ANGLE_DOUBLE(64))-_zeroPosSensorValue) / 4096.0 * 36.0 +  positionOffsetToMechanicalPosition;
}

double RotatorHW::getMechanicalPosition()
{
    // get a snapshot of the actual position based on the stepper position
    return (double)FMOD360(stepper->getCurrentPosition() * DEGREE_PER_STEP);
    
}

void RotatorHW::putRelativePosition(double position)
{
    // calculate new positions
    _targetPosition = FMOD360(getPosition() + position);
    long targetMotorPosition = FMOD360(_targetPosition - positionOffsetToMechanicalPosition) / DEGREE_PER_STEP;

    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    _isMoving = false;
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE(64)));
}

void RotatorHW::putAbsolutePosition(double position)
{
    // set new position
    _targetPosition = FMOD360(position);
    long targetMotorPosition = FMOD360(_targetPosition - positionOffsetToMechanicalPosition) / DEGREE_PER_STEP;

    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    _isMoving = false;
    int sensorPosition = (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE_DOUBLE(64));
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d, Delta %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), sensorPosition, sensorPosition - (int)EXPECTED_SENSORVALUE(getMechanicalPosition()));
}

void RotatorHW::putMechanicalPosition(double position)
{
    // calculate new position
    _targetPosition = FMOD360(position + positionOffsetToMechanicalPosition);
    long targetMotorPosition = FMOD360(position) / DEGREE_PER_STEP;
    // and move the motor
    _isMoving = true;
    MOVETO_WAIT(targetMotorPosition);
    _isMoving = false;
    ESP_LOGI(TAG, "For mechanical Position %f, Expected Sensor Value %d, Read Sensor Value %d", getMechanicalPosition(), (int)EXPECTED_SENSORVALUE(getMechanicalPosition()), (int)CORRECTED_SENSORVALUE(MEASURE_PRECISE_ANGLE(64)));
}

void RotatorHW::syncPosition(double position)
{
    positionOffsetToMechanicalPosition = position - getMechanicalPosition();
}