#include "BalanceCar.h"
#include "ProtocolParser.h"
#include "Sounds.h"
#include "PinChangeInt.h"
#include "TimerOne.h"
//#define DEBUG_LEVEL  DEBUG_LEVEL_INFO
#include "debug.h"
static volatile int EncoderRightPulse = 0, EncoderLeftPulse = 0;

#if (EM_MOTOR_SHIELD_BOARD_VERSION < 3)
BalanceCar::BalanceCar(ProtocolParser *Package, uint8_t ain1, uint8_t ain2, uint8_t pwma, uint8_t bin1, uint8_t bin2, uint8_t pwmb, uint8_t standby, uint8_t encoder_left, uint8_t encoder_right):SmartCar("Balance-Car", E_BALANCE_CAR, 0x01, E_BLUETOOTH_CONTROL)
#else
BalanceCar::BalanceCar(ProtocolParser *Package, uint8_t ain1, uint8_t pwma, uint8_t bin1, uint8_t pwmb, uint8_t encoder_left, uint8_t encoder_right):SmartCar("Balance-Car", E_BALANCE_CAR, 0x01, E_BLUETOOTH_CONTROL)
#endif
{
	  this->Ain1Pin = ain1;
    this->PwmaPin = pwma;
    this->Bin1Pin = bin1;
    this->PwmbPin = pwmb;
    this->EncoderLeftPinA = encoder_left;
    this->EncoderRightPinA = encoder_right;
    EncoderRightPulse = 0;
    EncoderLeftPulse = 0;
    //ax = ay = az = gx = gy = gz = 0.0;
#if (EM_MOTOR_SHIELD_BOARD_VERSION < 3)
    this->Ain2Pin = ain2;
    this->Bin2Pin = bin2;
    this->StandbyPin = standby;
#endif
    SetStatus(E_STOP);
    mProtocolPackage = Package;
}

BalanceCar::~BalanceCar()
{
    delete mIrRecv;
    delete mBuzzer;
    // delete mpu;
    delete mPs2x;
    delete mRgb;
}

void BalanceCar::EncoderLeftCount(void)
{
    EncoderLeftPulse++;
}

void BalanceCar::EncoderRightCount(void)
{
    EncoderRightPulse++;
}

unsigned int BalanceCar::GetEncoderSpeed(uint8_t Motor)
{
    return EncoderRightPulse;
}

void BalanceCar::init(void)
{
    DEBUG_LOG(DEBUG_LEVEL_INFO, "initialize ......\n");
    //keep TB6612 AIN stop
#if (EM_MOTOR_SHIELD_BOARD_VERSION < 3)
    pinMode(Ain2Pin, OUTPUT);
    digitalWrite(Ain2Pin, LOW);
    //keep TB6612 BIN stop
    pinMode(Bin2Pin, OUTPUT);
    digitalWrite(Bin2Pin, LOW);
    //keep TB6612 Standby
    pinMode(StandbyPin, OUTPUT);
    digitalWrite(StandbyPin, LOW);
#endif
    //encoder pin input
    pinMode(Ain1Pin, OUTPUT);
    digitalWrite(Ain1Pin, LOW);
    pinMode(PwmaPin, OUTPUT);
    digitalWrite(PwmaPin, 0);
    pinMode(Bin1Pin, OUTPUT);
    digitalWrite(Bin1Pin, LOW);
    pinMode(PwmbPin, OUTPUT);
    digitalWrite(PwmbPin, 0);
    pinMode(EncoderLeftPinA, INPUT);
    pinMode(EncoderRightPinA, INPUT);
    
    Wire.begin();
    delay(100);
    int32_t ax_zero = 0, ay_zero = 0, az_zero = 0, gx_zero =0 ,gy_zero = 0,gz_zero = 0;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.initialize();
    delay(100);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    for (int i = 0; i < 200; i++) {
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
       ax_zero += ax;
       ay_zero += ay;
       az_zero += (az - 16384);
       gx_zero += gx;
       gy_zero += gy;
       gz_zero += gz;
    }
    AccelZeroOffsent.x = ax_zero/200;
    AccelZeroOffsent.y = ay_zero/200;
    AccelZeroOffsent.z = az_zero/200;
    GyroZeroOffsent.x = gx_zero/200;
    GyroZeroOffsent.y = gy_zero/200;
    GyroZeroOffsent.z = gz_zero/200;
 /*   Serial.println("AccelZeroOffsent :");
    Serial.println(AccelZeroOffsent.x);
    Serial.println(AccelZeroOffsent.y);
    Serial.println(AccelZeroOffsent.z);
    Serial.println("GyroZeroOffsent :");
    Serial.println(GyroZeroOffsent.x);
    Serial.println(GyroZeroOffsent.y);
    Serial.println(GyroZeroOffsent.z);
*/
    attachPinChangeInterrupt(EncoderLeftPinA, *((void (*)(void))(&EncoderLeftCount)), CHANGE);
    attachPinChangeInterrupt(EncoderRightPinA, *((void (*)(void))(&EncoderRightCount)), CHANGE);
}

void BalanceCar::GoForward(void)
{
    SetStatus(E_FORWARD);
}

void BalanceCar::GoBack(void)
{
    SetStatus(E_BACK);
}

void BalanceCar::KeepStop(void)
{
    DEBUG_LOG(DEBUG_LEVEL_INFO, "KeepStop\n");
#if (EM_MOTOR_SHIELD_BOARD_VERSION < 3)
    digitalWrite(Ain2Pin, HIGH);
    digitalWrite(Bin2Pin, HIGH);
    digitalWrite(StandbyPin, HIGH);
#endif
    digitalWrite(Ain1Pin, LOW);
    digitalWrite(PwmaPin, LOW);
    digitalWrite(Bin1Pin, LOW);
    digitalWrite(PwmbPin, LOW);
    SetStatus(E_STOP);
}

void BalanceCar::TurnLeft(byte mode)
{
    SetStatus(mode);
}

void BalanceCar::TurnRight(byte mode)
{
    SetStatus(mode);
}

void BalanceCar::SetPwm(int angleoutput, int speedoutput, int rotationoutput)
{
    E_SMARTCAR_STATUS status = GetStatus();
    if (E_RIGHT == status) {
        MotorRightPwm = -angleoutput - speedoutput + rotationoutput;
        MotorLeftPwm = -angleoutput - speedoutput + rotationoutput/3;
    } else if (E_LEFT == status) {
        MotorRightPwm = -angleoutput - speedoutput - rotationoutput/3 ;
        MotorLeftPwm = -angleoutput - speedoutput - rotationoutput;
    } else {
        MotorRightPwm = -angleoutput - speedoutput + rotationoutput;
        MotorLeftPwm = -angleoutput - speedoutput - rotationoutput;
    }
    if (MotorLeftPwm > 255) MotorLeftPwm = 255;
    if (MotorLeftPwm < -255) MotorLeftPwm = -255;
    if (MotorRightPwm > 255) MotorRightPwm = 255;
    if (MotorRightPwm < -255) MotorRightPwm = -255;
    if (mKalFilter.angle > 30 || mKalFilter.angle < -30) {
        MotorRightPwm = 0;
        MotorLeftPwm = 0;
        MotorRightPulse = 0;
        MotorLeftPulse = 0;
        KeepStop();
    } else {
        if (status == E_STOP ) {
            SetStatus(E_RUNNING);
        }
    }
    if (MotorLeftPwm >= 0) {
        digitalWrite(Ain1Pin, HIGH);
        analogWrite(PwmaPin, MotorLeftPwm);
    } else {
        digitalWrite(Ain1Pin, LOW);
        analogWrite(PwmaPin, -MotorLeftPwm);
    }

    if (MotorRightPwm >= 0) {
        digitalWrite(Bin1Pin, LOW);
        analogWrite(PwmbPin, MotorRightPwm);
    } else {
        digitalWrite(Bin1Pin, HIGH);
        analogWrite(PwmbPin, -MotorRightPwm);
    }
}

double BalanceCar::BalancePwm(ST_PID pid,float Angle, float Gyro)
{
    float Bias = 0;
    double balance;
    balance = pid.p * (Angle - Bias) + pid.d * Gyro;
    return balance;
}

double BalanceCar::SpeedPwm(ST_PID pid, double p0)
{
    static float speeds_filterold = 0, positions = 0.0;
    float speeds_filter;
    float speeds = (MotorRightPulse + MotorLeftPulse) * 1.0;
    MotorRightPulse = MotorLeftPulse = 0;
    speeds_filter = speeds_filterold * 0.7 + speeds * 0.3;
    speeds_filterold = speeds_filter;
    positions += speeds_filter;
    E_SMARTCAR_STATUS status = GetStatus();

    if (E_FORWARD == status) {
        positions = positions - 100;
    } else if (E_BACK == status) {
        positions = positions + 100;
    }
    positions = constrain(positions, -3550, 3550);
    double output = pid.i * (p0 - positions) + pid.p * (p0 - speeds_filter);

    if (status == E_STOP) {
        positions = 0;
        speeds_filterold = 0;
        output = 0;
    }
    return output;
}

double BalanceCar::TurnPwm(ST_PID pid, float Gyroz)
{
    float Bias = 0;
    double turnout_put;
    Bias = Gyroz - 0;
    E_SMARTCAR_STATUS status = GetStatus();
    turnout_put = pid.p* Gyroz;
    if (E_RIGHT_ROTATE == status || E_RIGHT == status) {
        turnout_put -= 350;
    } else if (E_LEFT_ROTATE == status || E_LEFT == status ){
        turnout_put += 350;
    }
    return turnout_put;
}

void BalanceCar::SetPID(ST_PID balance, ST_PID speedpid, ST_PID turn)
{
    BalancePid = balance;
    SpeedPid = speedpid;
    TurnPid = turn;
    // DEBUG_LOG(DEBUG_LEVEL_INFO, "BalancPid = %f %f %f\n", BalancPid.p, BalancPid.i, BalancPid.d);
}

void BalanceCar::CalculatePulse(void)
{
    int leftpluse = EncoderLeftPulse;
    int rightpluse  = EncoderRightPulse;

    EncoderLeftPulse = 0;
    EncoderRightPulse = 0;

    if (MotorRightPwm < 0 && MotorLeftPwm < 0) {
        rightpluse = -rightpluse;
        leftpluse = -leftpluse;
    } else if (MotorRightPwm > 0 && MotorLeftPwm > 0) {
        rightpluse = rightpluse;
        leftpluse = leftpluse;
    } else if (MotorRightPwm < 0 && MotorLeftPwm > 0) {
        rightpluse = -rightpluse;
        leftpluse = leftpluse;
    } else if (MotorRightPwm > 0 && MotorLeftPwm < 0) {
        rightpluse = rightpluse;
        leftpluse = -leftpluse;
    }
    MotorRightPulse += rightpluse;
    MotorLeftPulse += leftpluse;
}

bool BalanceCar::IsPickUp(void)
{
    return false;
}
bool BalanceCar::IsPutDown(void)
{
    return true;
}

void BalanceCar::SetIrPin(uint8_t pin = BC_IR_PIN)
{
    IrPin = pin;
    mIrRecv = new IRremote (IrPin);
    mIrRecv->begin();  // Initialize the infrared receiver
}

void BalanceCar::SetBuzzerPin(uint8_t pin = BC_BUZZER_PIN)
{
    BuzzerPin = pin;
    mBuzzer = new Buzzer();
    mBuzzer->setpin(BuzzerPin);
}

void BalanceCar::SetRgbPin(uint8_t pin = BC_RGB_PIN)
{
    RgbPin = pin;
    mRgb = new RGBLed(7, RgbPin);
    mRgb->setNumber(2);
    // mRgb->reset();
}

void BalanceCar::LightOn(E_RGB_INDEX index = E_RGB_ALL, long Color = RGB_WHITE)
{
    // mRgb->reset();
    if (index == E_RGB_ALL) {
        mRgb->setColor(0, Color);
    } else {
        mRgb->setColor(index, Color);
    }
    mRgb->show();
}
void BalanceCar::LightOff(E_RGB_INDEX index = E_RGB_ALL)
{
    //  mRgb->reset();
    if (index == E_RGB_ALL) {
        mRgb->setColor(0, 0, 0);
    } else {
        mRgb->setColor(index, 0);
    }
    mRgb->show();
}

int BalanceCar::SetPs2xPin(uint8_t clk = BC_PS2X_CLK, uint8_t cmd = BC_PS2X_CMD, uint8_t att = BC_PS2X_CS, uint8_t dat = BC_PS2X_DAT)
{
    static bool Ps2xInit = false;
    int error = 0 ;
    if (!Ps2xInit) {
        DEBUG_LOG(DEBUG_LEVEL_INFO, "SetPs2xPin\n");
        Ps2xClkPin = clk;
        Ps2xCmdPin = cmd;
        Ps2xAttPin = att;
        Ps2xDatPin = dat;
        mPs2x = new PS2X();
        //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
        //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
        error = mPs2x->config_gamepad(Ps2xClkPin, Ps2xCmdPin, Ps2xAttPin, Ps2xDatPin, false, false);
/*
        if (error == 1) {
            DEBUG_LOG(DEBUG_LEVEL_ERR, "No controller found, check wiring\n");
        } else if(error == 2) {
            DEBUG_LOG(DEBUG_LEVEL_ERR, "Controller found but not accepting commands\n");
        } else if(error == 3) {
            DEBUG_LOG(DEBUG_LEVEL_ERR, "Controller refusing to enter Pressures mode, may not support it\n");
        } else if(error == 0) {
            DEBUG_LOG(DEBUG_LEVEL_INFO, "Found Controller, configured successful\n");
        }
        Ps2xInit = true;
 */
    }
    return error;
}

int BalanceCar::ResetPs2xPin(void)
{
    int error = mPs2x->config_gamepad(Ps2xClkPin, Ps2xCmdPin, Ps2xAttPin, Ps2xDatPin, false, false);
    return error;
}

void BalanceCar::SetUltrasonicPin(uint8_t Trig_Pin = BC_TRIG_PIN, uint8_t Echo_Pin = BC_ECHO_PIN)
{
    EchoPin = Echo_Pin;
    TrigPin = Trig_Pin;
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);
}

float BalanceCar::GetUltrasonicDistance()
{
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    UltrasonicDistance = pulseIn(EchoPin, HIGH) / 58.00 ;
    return UltrasonicDistance;
}

float BalanceCar::PianoSing(byte b[])
{
    union result
    {
      float d;
      unsigned char data[4];
    }r1,r2;
    r2.data[0]=b[0];
    r2.data[1]=b[1];
    r2.data[2]=b[2];
    r2.data[3]=b[3];
    //mBuzzer->noTone(9);
    mBuzzer->_tone(r2.d,120, 2);
    return (r2.d);
}

void BalanceCar::Sing(byte songName)
{
    switch(songName) {
       case S_connection:
         mBuzzer->_tone(note_E5, 50, 30);
         mBuzzer->_tone(note_E6, 55, 25);
         mBuzzer->_tone(note_A6, 60, 10);
          //  _tone(9, 394);
       break;

       case S_disconnection:
         mBuzzer->_tone(note_E5,50,30);
         mBuzzer->_tone(note_A6,55,25);
         mBuzzer->_tone(note_E6,50,10);
       break;

       case S_buttonPushed:
         mBuzzer->bendTones (note_E6, note_G6, 1.03, 20, 2);
         delay(30);
         mBuzzer->bendTones (note_E6, note_D7, 1.04, 10, 2);
       break;

       case S_mode1:
          mBuzzer->bendTones (note_E6, note_A6, 1.02, 30, 10);  //1318.51 to 1760
       break;

       case S_mode2:
          mBuzzer->bendTones (note_G6, note_D7, 1.03, 30, 10);  //1567.98 to 2349.32
       break;

       case S_mode3:
         mBuzzer->_tone(note_E6,50,100); //D6
         mBuzzer->_tone(note_G6,50,80);  //E6
         mBuzzer->_tone(note_D7,300,0);  //G6
       break;

       case S_surprise:
         mBuzzer->bendTones(800, 2150, 1.02, 10, 1);
         mBuzzer->bendTones(2149, 800, 1.03, 7, 1);
       break;

       case S_OhOoh:
         mBuzzer->bendTones(880, 2000, 1.04, 8, 3); //A5 = 880
         delay(200);

         for (int i=880; i<2000; i=i*1.04) {
              mBuzzer->_tone(note_B5,5,10);
         }
       break;

       case S_OhOoh2:
          mBuzzer->bendTones(1880, 3000, 1.03, 8, 3);
         delay(200);

         for (int i=1880; i<3000; i=i*1.03) {
             mBuzzer->_tone(note_C6,10,10);
         }
       break;

       case S_cuddly:
          mBuzzer->bendTones(700, 900, 1.03, 16, 4);
          mBuzzer->bendTones(899, 650, 1.01, 18, 7);
       break;

       case S_sleeping:
          mBuzzer->bendTones(100, 500, 1.04, 10, 10);
          delay(500);
          mBuzzer->bendTones(400, 100, 1.04, 10, 1);
       break;

       case S_happy:
          mBuzzer->bendTones(1500, 2500, 1.05, 20, 8);
          mBuzzer->bendTones(2499, 1500, 1.05, 25, 8);
       break;

       case S_superHappy:
          mBuzzer->bendTones(2000, 6000, 1.05, 8, 3);
          delay(50);
          mBuzzer->bendTones(5999, 2000, 1.05, 13, 2);
       break;

       case S_happy_short:
          mBuzzer->bendTones(1500, 2000, 1.05, 15, 8);
         delay(100);
          mBuzzer->bendTones(1900, 2500, 1.05, 10, 8);
       break;

       case S_sad:
          mBuzzer->bendTones(880, 669, 1.02, 20, 200);
       break;

       case S_confused:
          mBuzzer->bendTones(1000, 1700, 1.03, 8, 2);
          mBuzzer->bendTones(1699, 500, 1.04, 8, 3);
          mBuzzer->bendTones(1000, 1700, 1.05, 9, 10);
       break;

       case S_fart1:
          mBuzzer->bendTones(1600, 3000, 1.02, 2, 15);
       break;

       case S_fart2:
          mBuzzer->bendTones(2000, 6000, 1.02, 2, 20);
       break;

       case S_fart3:
          mBuzzer->bendTones(1600, 4000, 1.02, 2, 20);
          mBuzzer->bendTones(4000, 3000, 1.02, 2, 20);
       break;
    }
}

void BalanceCar::ReportAllInfo()
{
    ST_PROTOCOL send_dat;
  //  GetUltrasonicDistance();
    ST_BALANCE_INFO info = {
        BalancePid,
        SpeedPid,
        TurnPid,
         // (byte)UltrasonicDistance,
        (short int)MotorRightPwm,
        (short int)MotorLeftPwm,
    };
    info.Temperature = mpu.getTemperature()/340 + 36.53;
    info.Inclination = mKalFilter.angle;
    info.ax = Accel.x / AcceRatio;
    info.ay = Accel.y / AcceRatio;
    info.az = Accel.z / AcceRatio;
    info.gx = Gyro.x / GyroRatio;
    info.gy = Gyro.y / GyroRatio;
    info.gz = Gyro.z / GyroRatio;
    send_dat.start_code = PROTOCOL_START_CODE;
    send_dat.len = sizeof(info) + 8;
    send_dat.type = 0x01;
    send_dat.addr = 0x01;
    send_dat.function = E_INFO;
    send_dat.data = (byte *)&info;    
    send_dat.end_code = PROTOCOL_END_CODE;
    mProtocolPackage->SendPackage(&send_dat, sizeof(info));
}
