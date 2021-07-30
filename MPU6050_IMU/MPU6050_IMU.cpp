#include <Adafruit_MPU6050.h>
#include <glm/vec3.hpp>
#include <glm/ext/scalar_constants.hpp>

#include <MPU6050_IMU.hpp>

const float G0_TO_MPS2 = 9.80665;                   // g to m/s^2 conversion factor
const float MPS2_TO_G0 = 1.0 / G0_TO_MPS2;          // m/s^2 to g conversion factor
const float RADS_TO_DEG = 180.0 / glm::pi<float>(); // radians to degrees conversion factor
const float DEG_TO_RADS = 1.0 / RADS_TO_DEG;        // degrees to radians conversion factor

MPU6050_IMU::MPU6050_IMU() {}
MPU6050_IMU::~MPU6050_IMU() {}

bool MPU6050_IMU::init()
{
    if (!_mpu6050.begin())
    {
        return false;
    }

    _mpu6050.setFilterBandwidth(MPU6050_BAND_260_HZ);  // DLPF band:  260 Hz (default), 184 Hz, 94 Hz, 44 Hz, 21 Hz, 10 Hz, 5 Hz
    _mpu6050.setAccelerometerRange(MPU6050_RANGE_2_G); // accl range: 16g, 8g, 4g, 2g (default)
    _mpu6050.setGyroRange(MPU6050_RANGE_250_DEG);      // gyro range: 2000 deg/s, 1000 deg/s, 500 deg/s, 250 deg/s (default)
    // _mpu6050.setCycleRate(MPU6050_CYCLE_20_HZ);        // cycle rate: 40 Hz, 20 Hz, 5 Hz, 1.25 Hz

    return true;
}

void MPU6050_IMU::_readAccel(glm::vec3 *result)
{
    // Library converts data to a standard Earth gravity of 9.80665 m/s^2
    sensors_event_t event;
    _mpu6050.getAccelerometerSensor()->getEvent(&event);
    sensors_vec_t acc = event.acceleration;
    result->x = (acc.x + _xAccelBiasMPS2) * _xAccelWeight;
    result->y = (acc.y + _yAccelBiasMPS2) * _yAccelWeight;
    result->z = (acc.z + _zAccelBiasMPS2) * _zAccelWeight;
}

void MPU6050_IMU::readAccel(glm::vec3 *result, IMU6DoF::accel_units units)
{
    _readAccel(result);
    switch (units)
    {
    case accel_units::MPS2:
        break;
    case accel_units::G0:
        result->x *= MPS2_TO_G0;
        result->y *= MPS2_TO_G0;
        result->z *= MPS2_TO_G0;
        break;
    }
}

void MPU6050_IMU::_readGyro(glm::vec3 *result)
{
    // Library converts data to radians
    sensors_event_t event;
    _mpu6050.getGyroSensor()->getEvent(&event);
    sensors_vec_t gyro = event.gyro;
    result->x = (gyro.x + _xGyroBiasRads);
    result->y = (gyro.y + _yGyroBiasRads);
    result->z = (gyro.z + _zGyroBiasRads);
}

void MPU6050_IMU::readGyro(glm::vec3 *result, IMU6DoF::gyro_units units)
{
    _readGyro(result);
    switch (units)
    {
    case gyro_units::RADS:
        break;
    case gyro_units::DPS:
        result->x *= RADS_TO_DEG;
        result->y *= RADS_TO_DEG;
        result->z *= RADS_TO_DEG;
        break;
    }
}

void MPU6050_IMU::setAccelBiases(float xBias, float yBias, float zBias, IMU6DoF::accel_units units)
{
    switch (units)
    {
    case IMU6DoF::accel_units::MPS2:
        break;
    case IMU6DoF::accel_units::G0:
        xBias *= G0_TO_MPS2;
        yBias *= G0_TO_MPS2;
        zBias *= G0_TO_MPS2;
        break;
    }
    _xAccelBiasMPS2 = xBias;
    _yAccelBiasMPS2 = yBias;
    _zAccelBiasMPS2 = zBias;
}

void MPU6050_IMU::setAccelBiases(glm::vec3 biases, IMU6DoF::accel_units units)
{
    setAccelBiases(biases.x, biases.y, biases.z, units);
}

void MPU6050_IMU::setAccelWeights(float xWeight, float yWeight, float zWeight)
{
    _xAccelWeight = xWeight;
    _yAccelWeight = yWeight;
    _zAccelWeight = zWeight;
}

void MPU6050_IMU::setAccelWeights(glm::vec3 weights)
{
    setAccelWeights(weights.x, weights.y, weights.z);
}

void MPU6050_IMU::setGyroBiases(float xBias, float yBias, float zBias, IMU6DoF::gyro_units units)
{
    switch (units)
    {
    case IMU6DoF::gyro_units::RADS:
        break;
    case IMU6DoF::gyro_units::DPS:
        xBias *= DEG_TO_RADS;
        yBias *= DEG_TO_RADS;
        zBias *= DEG_TO_RADS;
        break;
    }
    _xGyroBiasRads = xBias;
    _yGyroBiasRads = yBias;
    _zGyroBiasRads = zBias;
}

void MPU6050_IMU::setGyroBiases(glm::vec3 biases, IMU6DoF::gyro_units units)
{
    setGyroBiases(biases.x, biases.y, biases.z, units);
}
