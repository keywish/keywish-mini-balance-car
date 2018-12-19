#ifndef _BalanceCBC_H_
#define _BalanceCBC_H_

#define VERSION "AR-20180716"

#include <stdint.h>
#include "Arduino.h"
#include "SmartCar.h"
#include "IRremote.h"
#include "PS2X_lib.h"  //for v1.6
#include "Buzzer.h"
#include "RGBLed.h"
#include "mpu6050.h"
#include "ProtocolParser.h"
#include "KalmanFilter.h"

#define  AcceRatio 16384.0
#define  GyroRatio  131.0

#define BC_IR_PIN 8
#define BC_AIN1_PIN 3
#define BC_AIN2_PIN 11
#define BC_PWMA_PIN 5
#define BC_BIN1_PIN 4
#define BC_BIN2_PIN 2
#define BC_PWMB_PIN 6
#define BC_STANBY_PIN 7

#define BC_ENCODER_L 13
#define BC_ENCODER_R 10

#define BC_RGB_PIN A3
#define BC_BUZZER_PIN 9

#define BC_ECHO_PIN A1
#define BC_TRIG_PIN A0

#define BC_PS2X_CLK A5
#define BC_PS2X_CMD A1
#define BC_PS2X_CS  A2
#define BC_PS2X_DAT A0

typedef enum
{
    E_RGB_ALL = 0,
    E_RGB_RIGHT = 1,
    E_RGB_LEFT = 2
} E_RGB_INDEX;


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}ST_MPU6050_RAW;

typedef struct
{
    double p;
    double i;
    double d;
} ST_PID;

typedef struct
{
    ST_PID balace;
    ST_PID speed;
    ST_PID turn;
   // byte UltrasonicDistance;
    short int MotorRightPwm;
    short int MotorLeftPwm;
    float Temperature;
    float Inclination;
    float ax;   // g
    float ay;
    float az;
    float gx;   // dec/s
    float gy;
    float gz;
} ST_BALANCE_INFO;

class BalanceCar : public SmartCar {

private :
    uint8_t Ain1Pin, Ain2Pin, PwmaPin, Bin1Pin, Bin2Pin, PwmbPin, StandbyPin, EncoderLeftPinA, EncoderRightPinA;
    uint8_t EchoPin, TrigPin;
    uint8_t IrPin;      // Infrared remoter pin
    uint8_t BuzzerPin;  // Buzzer pin
    uint8_t RgbPin;     // Rgb pin
    uint8_t Ps2xClkPin, Ps2xCmdPin, Ps2xAttPin, Ps2xDatPin;    // for Ps2 remoter
    ProtocolParser *mProtocolPackage;
public :
    BalanceCar(ProtocolParser *Package, uint8_t ain1, uint8_t ain2, uint8_t pwma, uint8_t bin1, uint8_t bin2, uint8_t pwmb, uint8_t standby, uint8_t encoder_left, uint8_t encoder_right);
    ~BalanceCar();
    IRremote *mIrRecv;
    PS2X *mPs2x;
    Buzzer *mBuzzer;
    RGBLed *mRgb;
	  MPU6050 mpu;
    ST_MPU6050_RAW AccelZeroOffsent, GyroZeroOffsent;  // mpu6050 static
    float Temperature;
    float UltrasonicDistance;
    float Inclination;
    KalmanFilter mKalFilter;
    ST_MPU6050_RAW Accel, Gyro;
    ST_PID BalancePid, SpeedPid, TurnPid;
    double MotorRightPwm, MotorLeftPwm;
    int MotorRightPulse, MotorLeftPulse;

    void EncoderRightCount(void);
    void EncoderLeftCount(void);
    void CalculatePulse(void);
    void AutomaticBalance(void);
    void GoForward(void);
    void GoBack(void);
    void TurnLeft(byte mode);
    void TurnRight(byte mode);
    void KeepStop(void);
    void SetPwm(int angleoutput, int speedoutput, int rotationoutput);
    double BalancePwm(ST_PID pid,float Angle, float Gyro);
    double SpeedPwm(ST_PID pid, double p0);
    double TurnPwm(ST_PID pid,float Gyroz);
    void SetPID(ST_PID balance, ST_PID speed, ST_PID turn);
    bool IsPickUp(void);
    bool IsPutDown(void);

    void SetIrPin(uint8_t pin = BC_IR_PIN);
    unsigned int GetEncoderSpeed(uint8_t Motor);
    
    void SetRgbPin(uint8_t pin = BC_RGB_PIN);
    void LightOn(E_RGB_INDEX index = E_RGB_ALL, long Color = RGB_WHITE);
    void LightOff(E_RGB_INDEX index = E_RGB_ALL);
    void SetBuzzerPin(uint8_t pin = BC_BUZZER_PIN);
    void Sing(byte songName);
    float PianoSing(byte b[]);
    int SetPs2xPin(uint8_t clk = BC_PS2X_CLK, uint8_t cmd = BC_PS2X_CMD, uint8_t att = BC_PS2X_CS, uint8_t dat = BC_PS2X_DAT);
    void SetUltrasonicPin(uint8_t Trig_Pin = BC_TRIG_PIN, uint8_t Echo_Pin = BC_ECHO_PIN);
    float GetUltrasonicDistance();
    int ResetPs2xPin(void);
    void ReportAllInfo();
    void init(void);
};

#endif  /* _BalanceCBC_H_ */
