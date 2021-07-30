#ifndef _MPU6050_IMU_H
#define _MPU6050_IMU_H

#include <Adafruit_MPU6050.h>
#include <glm/vec3.hpp>

#include <IMU6DoF.hpp>

class MPU6050_IMU : public IMU6DoF
{
private:
    void _readAccel(glm::vec3 *result);
    void _readGyro(glm::vec3 *result);

    Adafruit_MPU6050 _mpu6050;

    double _xAccelBiasMPS2 = 0.0, _yAccelBiasMPS2 = 0.0, _zAccelBiasMPS2 = 0.0;
    double _xAccelWeight = 1.0, _yAccelWeight = 1.0, _zAccelWeight = 1.0;

    double _xGyroBiasRads = 0.0, _yGyroBiasRads = 0.0, _zGyroBiasRads = 0.0;

public:
    MPU6050_IMU();
    ~MPU6050_IMU();

    bool init();

    void readAccel(glm::vec3 *result, IMU6DoF::accel_units units);
    void readGyro(glm::vec3 *result, IMU6DoF::gyro_units units);

    void setAccelBiases(float xBias, float yBias, float zBias, IMU6DoF::accel_units units);
    void setAccelBiases(glm::vec3 biases, IMU6DoF::accel_units units);
    void setAccelWeights(float xWeight, float yWeight, float zWeight);
    void setAccelWeights(glm::vec3 weights);

    void setGyroBiases(float xBias, float yBias, float zBias, IMU6DoF::gyro_units units);
    void setGyroBiases(glm::vec3 biases, IMU6DoF::gyro_units units);
};

#endif
