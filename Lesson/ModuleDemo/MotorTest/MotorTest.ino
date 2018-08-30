#define AIN1 3
#define AIN2 11
#define BIN1 4
#define BIN2 2
#define PWMA 5
#define PWMB 6
#define STBY 7

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
}

void loop()
{
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2, LOW); 
    analogWrite(PWMA,255);
    delay(2000);
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2, HIGH); 
    analogWrite(PWMA,255);
    delay(2000);
    analogWrite(PWMA,0);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2, HIGH); 
    analogWrite(PWMB,255);
    delay(2000);
    digitalWrite(BIN2,LOW);
    digitalWrite(BIN1, HIGH); 
    analogWrite(PWMB,255);
    delay(2000);
    analogWrite(PWMB,0);
}
