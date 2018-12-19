/*
KalmanFilter.h KalmanFilter.cpp - Library from github
*/

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class KalmanFilter
{
public:
  KalmanFilter(void);
  ~KalmanFilter();
	void OneOrderFilter(float angle_m, float gyro_m, float dt, float K1);
	void Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
	void Angle_X(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
  void Angle_Y(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
  float Gyro_x, Gyro_y, Gyro_z;
  float accelz = 0;
  float angle, angle6;

private:
	float angle_err,q_bias;
	float Pdot[4] = { 0, 0, 0, 0};
	float P[2][2] = {{ 1, 0 }, { 0, 1 }};
	float  PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float angle_dot;
    float K1;
    float Q_angle, Q_gyro;
    float R_angle, C_0;
    float timeChange;
    float dt;
};
#endif
//  _KALMANFILTER_H_
