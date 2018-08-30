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
	void OneOrderFilter(float angle_m, float gyro_m,float dt,float K1);
	void Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0);
	void Angle_X(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,
									float R_angle,float C_0,float K1);
    void Angle_Y(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);
  float Gyro_x, Gyro_y, Gyro_z;
  float accelz = 0;
  float angle;
  float angle6;

private:
	float angle_err,q_bias;
	float Pdot[4] = { 0, 0, 0, 0};
	float P[2][2] = {{ 1, 0 }, { 0, 1 }};
	float  PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float angle_dot;
    float K1;              // 对加速度计取值的权重
    float Q_angle, Q_gyro; // 角度数据置信度,角速度数据置信度
    float R_angle, C_0;
    float timeChange;      // 滤波法采样时间间隔毫秒
    float dt;              // 注意：dt的取值为滤波器采样时间
};
#endif
//  _KALMANFILTER_H_
