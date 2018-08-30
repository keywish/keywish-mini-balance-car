#include "PinChangeInt.h"
#define AIN1 3
#define AIN2 11
#define BIN1 4
#define BIN2 2
#define PWMA 5
#define PWMB 6
#define STBY 7

#define ENCODER_LEFT_A 13
#define ENCODER_RIGHT_A 10

int left_pluse = 0, right_pluse = 0;
void Code_left(void)
{
   left_pluse++;
   Serial.print("left_pluse ");
   Serial.println(left_pluse);
}

void Code_right(void)
{
   right_pluse++;
   Serial.print("right_pluse ");
   Serial.println(right_pluse);
}

void setup()
{
    Serial.begin(9600);
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
	pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
	pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    digitalWrite(STBY,HIGH);
    pinMode(ENCODER_LEFT_A, INPUT);
    
    pinMode(ENCODER_RIGHT_A, INPUT);
    attachPinChangeInterrupt(ENCODER_LEFT_A, Code_left, CHANGE);
    attachPinChangeInterrupt(ENCODER_RIGHT_A, Code_right, CHANGE);
}

void loop()
{
    delay(500);
}
