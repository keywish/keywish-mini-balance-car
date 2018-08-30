void setup()
{
    pinMode(9,OUTPUT);
}

void loop()
{  
    for(int i = 200; i <= 800; i++)   // 200HZ ~ 800HZ
    {
        tone(9,i);
    }
    delay(1000);                    //Max Frequency hold 1s
    for(int i= 800; i >= 200; i--)   // 800HZ ~ 200HZ
    {  
        tone(9,i);
        delay(10);
    }

}
