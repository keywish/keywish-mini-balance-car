#include "Buzzer.h"
#include "RGBLed.h"
#include "Servo_test.h"
#include "IR_remote.h"
#include "Keymap.h"

#define MOTOR_L2   4 //Bin1
#define MOTOR_L1   2  //Bin2
#define PWMB   6  //PWMB
#define STBY   7  //STBY
#define SERVO_PIN  5

Servo ForwardServo;
Buzzer buzzer(9);
IRremote ir(8);
RGBLed rgbled_A3(7,A3);

double Speed = 100;
int BaseDegree = 100;
int i = 40;
unsigned char keycode;

void setup()
{
    Serial.begin(9600);
    ir.begin(); 
    pinMode(STBY, OUTPUT);
    pinMode(MOTOR_L1, OUTPUT);
    pinMode(MOTOR_L2, OUTPUT);
    ForwardServo.attach(SERVO_PIN);
    ForwardServo.write(90);
}

void move(int direction )
{
	if (direction == 1) {
		digitalWrite(MOTOR_L2,HIGH);
		digitalWrite(MOTOR_L1, LOW); //GoForward
		digitalWrite(STBY,HIGH);
	} else if(direction == 2) {
		digitalWrite(MOTOR_L2,LOW);
		digitalWrite(MOTOR_L1,HIGH); // GoBack
		digitalWrite(STBY,HIGH);
	} else if(direction == 3) {
		digitalWrite(MOTOR_L2,LOW);
		digitalWrite(MOTOR_L1,HIGH); //stop
		digitalWrite(STBY,HIGH);
	}    
}

void loop()
{
    if (keycode = ir.getCode()) {
      Serial.println(keycode, HEX);
        switch((E_IR_KEYCODE)ir.getIrKey(keycode)) {
         case  IR_KEYCODE_UP :
              Serial.println("up");
              ForwardServo.write(BaseDegree);
              move(1);
              analogWrite(PWMB,Speed);
              rgbled_A3.setColor(0,0,Speed-35,0);
              rgbled_A3.show(); 
              break;
         case IR_KEYCODE_DOWN :
              ForwardServo.write(BaseDegree);
              move(2);
              analogWrite(PWMB,Speed);
              rgbled_A3.setColor(0,Speed-35,Speed-35,Speed-35);
              rgbled_A3.show(); 
              break;
         case IR_KEYCODE_LEFT :
              ForwardServo.write(BaseDegree - 50);
              move(1);
              analogWrite(PWMB,Speed);
              rgbled_A3.setColor(2,25,15,0);
              rgbled_A3.setColor(1,0,0,0);
              rgbled_A3.show(); 
              break;
          case IR_KEYCODE_RIGHT :
              ForwardServo.write(BaseDegree + 40);
              move(1);
              analogWrite(PWMB,Speed);
              rgbled_A3.setColor(1,25,15,0);
              rgbled_A3.setColor(2,0,0,0);
              rgbled_A3.show(); 
              break;
          case IR_KEYCODE_STAR  :
              i = i + 10;                    
              if ( i >= 255) {
                i=255;
                buzzer.tone(2000, i);
              }
              Speed = i;
              Serial.println(Speed);//The hexadecimal line feed output code
              buzzer.tone(3000, i);
              break;
          case IR_KEYCODE_POUND :
                i = i - 10;                    
                if (i <= 40) {
                    i=40;
                    buzzer.tone(2000, i);
                }  
                Speed = i;
                Serial.println(Speed);//The hexadecimal line feed output code
                buzzer.tone(3000, i);
                break;

          } 
    } else {
        Serial.println("stop");
        ForwardServo.write(BaseDegree);
        move(3);
        analogWrite(PWMB,0);
        rgbled_A3.setColor(0,Speed-35,0,0);
        rgbled_A3.show(); 
    }
    delay(100);
}
