#include "PinChangeInt.h"
#define AIN1 3
#define BIN1 4
#define PWMA 5
#define PWMB 6

#define ENCODER_LEFT_A 7
#define ENCODER_RIGHT_A 2

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
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  attachPinChangeInterrupt(ENCODER_LEFT_A, Code_left, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A, Code_right, CHANGE);
}

void loop()
{
  delay(500);
}
