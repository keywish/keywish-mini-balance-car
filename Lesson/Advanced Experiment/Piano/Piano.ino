#include "TimerOne.h"
#include "BalanceCar.h"
#include "ProtocolParser.h"
#include "KeyMap.h"
#include "Sounds.h"
#include "debug.h"


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

ProtocolParser *mProtocol = new ProtocolParser();


BalanceCar mBalanceCar(mProtocol, AIN1_PIN, AIN2_PIN, PWMA_PIN, BIN1_PIN, BIN2_PIN, PWMB_PIN, STANBY_PIN, ENCODER_L, ENCODER_R);
byte Ps2xStatus, Ps2xType;
ST_PID balance, speed, turn;

void setup()
{
    Serial.begin(9600);
    mBalanceCar.SetControlMode(E_PIANO_MODE);
    mBalanceCar.SetStatus(E_STOP);
    mBalanceCar.init();
    mBalanceCar.SetBuzzerPin(BUZZER_PIN);
    mBalanceCar.Sing(S_connection);
}

void HandleBluetoothRemote()
{
    if (mProtocol->ParserPackage()) {
        switch (mProtocol->GetRobotControlFun()) {
            case E_INFO:
                // mBalanceCar.GetUltrasonicDistance();
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
}
