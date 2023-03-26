// Reference: https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino


#ifndef _QUATERIONION_FILTER_H_
#define _QUATERIONION_FILTER_H_

#include <stdint.h>

struct SensorData
{
  float acc_x, acc_y, acc_z;     // acceleration
  float gyro_x, gyro_y, gyro_z;  // angular rate from gyrosope
  int16_t temp; // temperature
};

struct Euler
{
  float roll, pitch, yaw;
};

struct Quaterion
{
  float x, y, z, w;
};

class QuaterionFilter
{
  public:
  QuaterionFilter();
  ~QuaterionFilter();
  Quaterion update(SensorData data, float sampleTime);

  private:
  Quaterion quaterion_;
  float beta_; 
  float zeta_;
};


// void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);

#endif  // _QUATERIONION_FILTER_H_