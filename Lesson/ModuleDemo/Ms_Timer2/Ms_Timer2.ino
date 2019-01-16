#include "MsTimer2.h"

// Switch on LED on pin 13 each second


void flash() {

   static bool output = HIGH;
   Serial.print("flash : "); Serial.println(output);
   digitalWrite(13, output);
   output = !output;
}

void setup() {
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    Serial.print("start ");
    MsTimer2::set(500, flash); // 500ms period
    MsTimer2::start();
}

void loop() {
}
