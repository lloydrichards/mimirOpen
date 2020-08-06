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
package sendData;

RTC_DATA_ATTR int count;

void setup()
{

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    config config = mimir.initSPIFFS();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        mimir.initWIFI(config);
    }
    mimir.initMicroSD("/testing.txt");
    mimir.initSensors();

    envData data = mimir.readSensors();
    mimir.printSensors(data);
    mimir.logData(data, "/testing.txt");

    sendData.auth.deviceId = config.deviceId;
    sendData.auth.userId = config.userId;
    sendData.auth.macAddress = WiFi.macAddress();
    sendData.status = mimir.getStatus();
    sendData.data = data;

    //Everything that happens with the server happens in here
    mimir.WiFi_ON();

    mimir.WiFi_OFF();

    mimir.saveToSPIFFS(config);
    count++;
    Serial.print("Boot Count: ");
    Serial.println(count);
    mimir.SLEEP(15);
}


void loop()
{

}