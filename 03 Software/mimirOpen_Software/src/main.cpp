#include <Arduino.h>
#include <mimirOpen.h>

mimirOpen mimir(115200);

void setup()
{
  mimir.initSPIFFS();
  mimir.i2cScanner();
  mimir.initMicroSD();
  mimir.initSensors();
  mimir.initWIFI();
  //mimir.initPixels();
}

void loop()
{
  mimir.readSensors();
  mimir.printSensors();
  mimir.logData(mimir.stringData());

  mimir.WiFi_ON();
  mimir.sendData("https://us-central1-mimirhome-app.cloudfunctions.net/sensorData/add");
  mimir.WiFi_OFF();
  //mimir.testPixels();
  delay(300000);
}