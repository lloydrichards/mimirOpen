// NeoPixelTest
// This example will cycle between showing four pixels as Red, Green, Blue, White
// and then showing those pixels as Black.
//
// Included but commented out are examples of configuring a NeoPixelBus for
// different color order including an extra white channel, different data speeds, and
// for Esp8266 different methods to send the data.
// NOTE: You will need to make sure to pick the one for your platform 
//
//
// There is serial output of the current state so you can confirm and follow along
//
#include <Arduino.h>
#include "mimirOpen.h"

mimirOpen mimir(115200);

void setup()
{
config config = mimir.initSPIFFS(); 
 mimir.initMicroSD();
 mimir.initSensors();

 envData data = mimir.readSensors();
 mimir.printSensors(data);

 mimir.logData(data);
 Serial.println("**FINISHED**");
}


void loop()
{

}