/*
KalmanFilter.h KalmanFilter.cpp - Library from github
*/

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(void)
{
    K1 = 0.05;
    Q_angle = 0.001;
    Q_gyro = 0.005;
    R_angle = 0.5;
    C_0 = 1;
    timeChange = 5;
    dt = timeChange*0.001;
}

KalmanFilter::~KalmanFilter(void)
{
}

/*
  1-order low pass filter 
*/
void KalmanFilter::OneOrderFilter(float angle_m, float gyro_m,float dt,float K1)
{
    angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}


////////////////////////kalman/////////////////////////

void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0)
{
      angle += (gyro_m - q_bias) * dt;
      angle_err = angle_m - angle;
      Pdot[0] = Q_angle - P[0][1] - P[1][0];
      Pdot[1] = - P[1][1];
      Pdot[2] = - P[1][1];
      Pdot[3] = Q_gyro;
      P[0][0] += Pdot[0] * dt;
      P[0][1] += Pdot[1] * dt;
      P[1][0] += Pdot[2] * dt;
      P[1][1] += Pdot[3] * dt;
      PCt_0 = C_0 * P[0][0];
      PCt_1 = C_0 * P[1][0];
      E = R_angle + C_0 * PCt_0;
      K_0 = PCt_0 / E;
      K_1 = PCt_1 / E;
      t_0 = PCt_0;
      t_1 = C_0 * P[0][1];
      P[0][0] -= K_0 * t_0;
      P[0][1] -= K_0 * t_1;
      P[1][0] -= K_1 * t_0;
      P[1][1] -= K_1 * t_1;
      angle += K_0 * angle_err;
      q_bias += K_1 * angle_err;
      angle_dot = gyro_m - q_bias;
}

/*
    Get X inclination
*/
void KalmanFilter::Angle_X(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,
									float R_angle,float C_0,float K1)
{
    // int flag;
    float Angle = atan2(ay , az) * 57.3;
    Gyro_x = (gx - 128.1) / 131;
    Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro,R_angle,C_0);

    if (gz > 32768) gz -= 65536;
    Gyro_z = -gz / 131;
    accelz = az / 16.4;

    float angleAx = atan2(ax, az) * 180 / PI;
    Gyro_y = -gy / 131.00;
    OneOrderFilter(angleAx, Gyro_y, dt, K1);
}

/*
    Get Y inclination
*/
void KalmanFilter::Angle_Y(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{

    // int flag;
    float Angle = atan2(ax , az) * 57.3;
    Gyro_y = (gy - 128.1) / 131;
    //Gyro_y = gy / 131;
    Kalman_Filter(Angle, -Gyro_y, dt, Q_angle, Q_gyro, R_angle, C_0); 
    if (gz > 32768) gz -= 65536;
    Gyro_z = -gz / 131;
    accelz = az / 16.4;
    float angleAy = atan2(ay, az) * 57.3;
    Gyro_x = gx / 131.00;
    OneOrderFilter(angleAy, Gyro_x, dt, K1);
}
