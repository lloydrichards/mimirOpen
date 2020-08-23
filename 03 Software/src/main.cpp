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
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

mimirOpen mimir(115200);
DataPackage sendData;
AuthPackage sendAuth;

RTC_DATA_ATTR int bootCount = 0;

void setup()
{

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    mimir.printBootReason();
    if (bootCount == 0)
    {
        config config = mimir.initSPIFFS();
        config.email = "tester@word.com";
        config.serialNumber = "test";
        config.deviceName = "Test";

        mimir.initWIFI(config);
        sendAuth.email = config.email;
        sendAuth.serialNumber = config.serialNumber;
        sendAuth.macAddress = WiFi.macAddress();
        mimir.WiFi_ON();
        mimir.sendAuth("https://us-central1-mimirhome-app.cloudfunctions.net/dataTransfer/auth", sendAuth, config);
        mimir.WiFi_OFF();
    }
    config config = mimir.initSPIFFS();
    mimir.initMicroSD("/testing.txt");
    mimir.initSensors();

    // Serial.println("RESETTING WIFI");
    // mimir.forceWIFI(config);
    // delay(1000);

    envData data = mimir.readSensors();
    mimir.printSensors(data);
    mimir.logData(data, "/testing.txt");

    sendData.auth.deviceId = config.deviceId;
    sendData.auth.userId = config.userId;
    sendData.auth.macAddress = WiFi.macAddress();
    sendData.status = mimir.getStatus();
    sendData.data = data;

    //Everything that happens with the server happens in here
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    delay(500);
    mimir.WiFi_ON();
    mimir.sendData("https://us-central1-mimirhome-app.cloudfunctions.net/dataTransfer/add", sendData);
    mimir.WiFi_OFF();
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector

    mimir.saveToSPIFFS(config);

    Serial.print("Boot Count: ");
    Serial.println(bootCount);
    bootCount++;
    mimir.SLEEP(15);
}

void loop()
{
}