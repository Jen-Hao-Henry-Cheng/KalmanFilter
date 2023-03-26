// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

#include "quaterion_filter.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795

QuaterionFilter::QuaterionFilter() {
  quaterion_.x = 0;
  quaterion_.y = 0;
  quaterion_.z = 0;
  quaterion_.w = 1;

  float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  this->beta_ = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta_
  float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  this->zeta_ = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta_, the other free parameter in the Madgwick scheme usually set to a small or zero value
}

QuaterionFilter::~QuaterionFilter() {
}

Quaterion QuaterionFilter::update(SensorData data, float sampleTime) {
  // q1 = w, q2 = x, q3 = y, q4 = z
  float dt = sampleTime;
  float norm;                                                // vector norm
  float f1, f2, f3;                                          // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;  // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * quaterion_.w;
  float _halfq2 = 0.5f * quaterion_.x;
  float _halfq3 = 0.5f * quaterion_.y;
  float _halfq4 = 0.5f * quaterion_.z;
  float _2q1 = 2.0f * quaterion_.w;
  float _2q2 = 2.0f * quaterion_.x;
  float _2q3 = 2.0f * quaterion_.y;
  float _2q4 = 2.0f * quaterion_.z;
  float _2q1q3 = 2.0f * quaterion_.w * quaterion_.y;
  float _2q3q4 = 2.0f * quaterion_.y * quaterion_.z;

  // Normalise accelerometer measurement
  norm = sqrt(data.acc_x * data.acc_x + data.acc_y * data.acc_y + data.acc_z * data.acc_z);
  if (norm == 0.0f) return;  // handle NaN
  norm = 1.0f / norm;
  data.acc_x *= norm;
  data.acc_y *= norm;
  data.acc_z *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * quaterion_.z - _2q1 * quaterion_.y - data.acc_x;
  f2 = _2q1 * quaterion_.x + _2q3 * quaterion_.z - data.acc_y;
  f3 = 1.0f - _2q2 * quaterion_.x - _2q3 * quaterion_.y - data.acc_z;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * dt * zeta_;
  gbiasy += gerry * dt * zeta_;
  gbiasz += gerrz * dt * zeta_;
  data.gyro_x -= gbiasx;
  data.gyro_y -= gbiasy;
  data.gyro_z -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * data.gyro_x - _halfq3 * data.gyro_y - _halfq4 * data.gyro_z;
  qDot2 = _halfq1 * data.gyro_x + _halfq3 * data.gyro_z - _halfq4 * data.gyro_y;
  qDot3 = _halfq1 * data.gyro_y - _halfq2 * data.gyro_z + _halfq4 * data.gyro_x;
  qDot4 = _halfq1 * data.gyro_z + _halfq2 * data.gyro_y - _halfq3 * data.gyro_x;

  // Compute then integrate estimated quaternion derivative
  quaterion_.w += (qDot1 - (beta_ * hatDot1)) * dt;
  quaterion_.x += (qDot2 - (beta_ * hatDot2)) * dt;
  quaterion_.y += (qDot3 - (beta_ * hatDot3)) * dt;
  quaterion_.z += (qDot4 - (beta_ * hatDot4)) * dt;

  // Normalize the quaternion
  norm = sqrt(quaterion_.w * quaterion_.w
              + quaterion_.x * quaterion_.x
              + quaterion_.y * quaterion_.y
              + quaterion_.z * quaterion_.z);  // normalise quaternion
  norm = 1.0f / norm;
  quaterion_.w = quaterion_.w * norm;
  quaterion_.x = quaterion_.x * norm;
  quaterion_.y = quaterion_.y * norm;
  quaterion_.z = quaterion_.z * norm;

  return this->quaterion_;
}