#define AIN1 3
#define BIN1 4
#define PWMA 5
#define PWMB 6

void setup()
{
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void loop()
{
  digitalWrite(AIN1,HIGH);
  digitalWrite(BIN1,LOW);
  analogWrite(PWMB,255); 
  analogWrite(PWMA,255); 
  delay(2000);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
  analogWrite(PWMB,255); 
  analogWrite(PWMA,255); 
  delay(2000);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
  analogWrite(PWMB,0); 
  analogWrite(PWMA,0); 
  delay(2000);
}
