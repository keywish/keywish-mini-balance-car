#include "RGBLed.h"
RGBLed rgbled_A3(7,A3);
void setup()
{
  rgbled_A3.setColor(1,255,255,255);
	rgbled_A3.setColor(2,10,10,10);
  rgbled_A3.show();
}

void loop()
{

}

