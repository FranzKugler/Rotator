#ifndef ROTATORHW_H
#define ROTATORHW_H

// #include "Arduino.h"
#include "AS5600.h"
#include "FastAccelStepper.h"
#include "TMC2209.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define N_STEPS 400 // No of fullsteps for calibration
#define KMAX 4      // up to KMAX Fourier coefficients

class RotatorHW
{
public:
    // Access of the singleton
    static RotatorHW &getInstance();
    RotatorHW();

    void begin(); // call in setup()

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
    // void measureMechanicalZero(int noOfMeasures = 1);
    void findEdge(bool dir);
    // void calibrateAngleSensor(void);
    void calibrateAngleSensor(std::function<void(int)> onProgress);
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

    // sensor calibration
    void calibrateAngleSensorInit(void);
    void calibrateAngleSensorStep(double sensor_raw);
    void calibrateAngleSensorFinalize(void);
    // EKF functions
    double h_meas(double x);
    double H_jacobian(double x);
    void ekf_predict(int32_t delta_steps);
    void ekf_update(double s_raw);

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
    double phi[KMAX + 1];
    double delta_phi[KMAX + 1];
    double C0;
    double A[KMAX + 1];
    double B[KMAX + 1];
    int step_counter;

    // internal Extended Kalman values
    // process and measurement noise
    double Q; // variance process model
    double R; // variance sensor measurement

    // Filter-Zustand
    double x; // initial angle
    double P; // initial variance

    // Instantiate TMC2209
    TMC2209 stepper_driver;
    // Instantiate Rotary Sensor
    AS5600 as5600; //  use default Wire
    SemaphoreHandle_t i2cMutex;
    // Instantiate the AccelStepper Library and bind it to our TMC2209
    // AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    FastAccelStepperEngine engine;
    FastAccelStepper *stepper;
};

#endif // ROTATORHW_H