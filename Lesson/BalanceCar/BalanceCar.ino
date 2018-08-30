 #include "TimerOne.h"
#include "BalanceCar.h"
#include "ProtocolParser.h"
#include "KeyMap.h"
#include "Sounds.h"
#include "debug.h"

#define IR_PIN 8

#define AIN1_PIN 3
#define AIN2_PIN 11
#define PWMA_PIN 5

#define BIN1_PIN 4
#define BIN2_PIN 2
#define PWMB_PIN 6
#define STANBY_PIN 7

#define ENCODER_L 13
#define ENCODER_R 10

#define BUZZER_PIN 9
#define RGB_PIN 3

#define ECHO_PIN A1
#define TRIG_PIN A0

ProtocolParser *mProtocol = new ProtocolParser();
ST_PID BalancePid = {40, 0, -0.5 }, SpeedPid = {6.5, 0.61, 0}, TurnPid = {1, 0, 0};

BalanceCar mBalanceCar(mProtocol, AIN1_PIN, AIN2_PIN, PWMA_PIN, BIN1_PIN, BIN2_PIN, PWMB_PIN, STANBY_PIN, ENCODER_L, ENCODER_R);
byte Ps2xStatus, Ps2xType;
ST_PID balance, speed, turn;

void autoBalance(void)
{
    sei();
    static int count = 0;
    double balance_pwm = 0;
    static double speed_pwm = 0.0, turn_pwm = 0.0;
    mBalanceCar.CalculatePulse();
    mBalanceCar.mpu.getMotion6(&mBalanceCar.Accel.x, &mBalanceCar.Accel.y, &mBalanceCar.Accel.z, &mBalanceCar.Gyro.x, &mBalanceCar.Gyro.y, &mBalanceCar.Gyro.z);
    mBalanceCar.mKalFilter.Angle_Y(mBalanceCar.Accel.x, mBalanceCar.Accel.y, mBalanceCar.Accel.z, mBalanceCar.Gyro.x, mBalanceCar.Gyro.y, mBalanceCar.Gyro.z);
    // mBalanceCar.mKalFilter.Angle_Y(ax - mBalanceCar.AccelZeroOffsent.x, ay - mBalanceCar.AccelZeroOffsent.y, az-mBalanceCar.AccelZeroOffsent.z, gx - mBalanceCar.GyroZeroOffsent.x, gy - mBalanceCar.GyroZeroOffsent.y, gz - mBalanceCar.GyroZeroOffsent.z);
    balance_pwm = mBalanceCar.BalancePwm(mBalanceCar.BalancePid, mBalanceCar.mKalFilter.angle, mBalanceCar.mKalFilter.Gyro_y);
    count++;
    if (count == 4 || count == 8) {
        turn_pwm = mBalanceCar.TurnPwm(mBalanceCar.TurnPid, mBalanceCar.mKalFilter.Gyro_z);
    }
    if (count >= 8) {
        speed_pwm = mBalanceCar.SpeedPwm(mBalanceCar.SpeedPid, 0);
        count = 0;
    }
    mBalanceCar.SetPwm(balance_pwm, speed_pwm, turn_pwm);
}

void setup()
{
    Serial.begin(9600);
    mBalanceCar.SetPID(BalancePid, SpeedPid, TurnPid);
    mBalanceCar.SetControlMode(E_BLUETOOTH_CONTROL);
    mBalanceCar.SetStatus(E_RUNNING);
    mBalanceCar.init();
    mBalanceCar.SetBuzzerPin(BUZZER_PIN);
    mBalanceCar.SetIrPin(IR_PIN);
    mBalanceCar.SetRgbPin(RGB_PIN);
    mBalanceCar.SetUltrasonicPin(TRIG_PIN, ECHO_PIN);
    mBalanceCar.Sing(S_connection);
    Timer1.initialize(5000);
    Timer1.attachInterrupt(autoBalance); // 5ms attach the service routine here
}

void HandleBluetoothRemote()
{
    if (mProtocol->ParserPackage()) {
        switch (mProtocol->GetRobotControlFun()) {
            case E_INFO:
                mBalanceCar.ReportAllInfo();
                break;
            case E_ROBOT_CONTROL_DIRECTION:
                // Serial.println(mProtocol->GetRobotDirection());
                mBalanceCar.SetStatus(mProtocol->GetRobotDirection());
                break;
            case E_CONTROL_MODE:
                mBalanceCar.SetControlMode(mProtocol->GetControlMode());
                break;
            case E_VERSION:
                break;
        }
    }
}

void HandleInfaredRemote(byte irKeyCode)
{
    switch ((E_IR_KEYCODE)mBalanceCar.mIrRecv->getIrKey(irKeyCode))
    {
        case IR_KEYCODE_UP:
            // Serial.println("up");
            mBalanceCar.GoForward();
            break;
        case IR_KEYCODE_DOWN:
            // Serial.println("down");
            mBalanceCar.GoBack();
            break;
        case IR_KEYCODE_OK:
            // Serial.println("Stop");
            mBalanceCar.SetStatus(E_RUNNING);
            break;
        case IR_KEYCODE_LEFT:
            // Serial.println("left rotate");
            mBalanceCar.TurnLeft(E_LEFT_ROTATE);
            break;
        case IR_KEYCODE_RIGHT:
            // Serial.println("right rotate");
            mBalanceCar.TurnRight(E_RIGHT_ROTATE);
            break;
        case IR_KEYCODE_1:
            // Serial.println("left");
            mBalanceCar.TurnLeft(E_LEFT);
            break;
        case IR_KEYCODE_2:
            // Serial.println("right");
            mBalanceCar.TurnRight(E_RIGHT);
            break;
        default:
            break;
    }
}

void HandleUltrasoicAvoidance()
{
    float FrontDistance;
    FrontDistance = mBalanceCar.GetUltrasonicDistance();
    if (FrontDistance < 10) {
        mBalanceCar.GoBack();
        delay(200);
    }
    if (FrontDistance < 20) {
        mBalanceCar.TurnRight(E_RIGHT_ROTATE);
        delay(300);
    } else {
        mBalanceCar.GoForward();
    }
}

void loop() {
    mProtocol->RecevData();
    if (mBalanceCar.GetControlMode() !=  E_BLUETOOTH_CONTROL &&  mBalanceCar.GetControlMode() != E_PIANO_MODE) {
        if (mProtocol->ParserPackage()) {
            if (mProtocol->GetRobotControlFun() == E_CONTROL_MODE) {
                mBalanceCar.SetControlMode(mProtocol->GetControlMode());
            }
        }
    }
    switch (mBalanceCar.GetControlMode()) {
        case E_BLUETOOTH_CONTROL:
            HandleBluetoothRemote();
            break;
        case E_INFRARED_REMOTE_CONTROL:
            byte irKeyCode;
            if (irKeyCode = mBalanceCar.mIrRecv->getCode()) {
                  HandleInfaredRemote(irKeyCode);
                  delay(110);
            } else {
                if (mBalanceCar.GetStatus() != E_STOP ) {
                    mBalanceCar.SetStatus(E_RUNNING);
                }
            }
            break;
        case E_ULTRASONIC_AVOIDANCE:
            HandleUltrasoicAvoidance();
            break;
        case E_PIANO_MODE:
            if (mProtocol->ParserPackage()) {
                if (mProtocol->GetRobotControlFun() == E_BUZZER) {
                    mBalanceCar.PianoSing(mProtocol->GetPianoSing());
                } else if (mProtocol->GetRobotControlFun() == E_CONTROL_MODE) {
                    mBalanceCar.SetControlMode(mProtocol->GetControlMode());
                }
            }
            break;
        default:
            break;
    }

    switch (mBalanceCar.GetStatus()) {
        case E_FORWARD:
            mBalanceCar.LightOn();
            break;
        case E_LEFT:
        case E_LEFT_ROTATE:
            mBalanceCar.LightOff(E_RGB_RIGHT);
            mBalanceCar.LightOn(E_RGB_LEFT);
            break;
        case E_RIGHT:
        case E_RIGHT_ROTATE:
            mBalanceCar.LightOff(E_RGB_LEFT);
            mBalanceCar.LightOn(E_RGB_RIGHT);
            break;
        case E_BACK:
            mBalanceCar.LightOn(E_RGB_ALL, RGB_RED);
            break;
        case E_RUNNING:
        case E_STOP:
            mBalanceCar.LightOff();
            break;
        default:
            break;
    }
}
