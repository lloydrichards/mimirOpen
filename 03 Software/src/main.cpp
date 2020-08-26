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
#include <bsec.h>

mimirOpen mimir(115200);
DataPackage sendData;
AuthPackage sendAuth;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR uint8_t BSECState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
RTC_DATA_ATTR int64_t BSECTime = 0;

void setup()
{
    ///////////////////////////////////////////////////
    //////////////////////STARTUP//////////////////////
    ///////////////////////////////////////////////////
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    mimir.printBootReason();
    config config = mimir.initSPIFFS();
    mimir.initMicroSD("/testing.txt");
    mimir.initSensors(BSECState, BSECTime);
    envData data = mimir.readSensors(BSECState, BSECTime);
    mimir.checkBSECSensor();
    mimir.printSensors(data);
    mimir.logData(data, "/testing.txt");

    ///////////////////////////////////////////////////
    /////////////////////MODE CHANGE///////////////////
    ///////////////////////////////////////////////////
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("\nMonitor Button was pressed!");
        mimir.initPixels();
        mimir.changeMode(8000);
    }

    ///////////////////////////////////////////////////
    /////////////////////FORCED MODE///////////////////
    ///////////////////////////////////////////////////
    if (digitalRead(ENVIRO_PIN) == LOW && digitalRead(RADAR_PIN) == LOW)
    {
        Serial.println("\nEntering Forced Mode");
        mimir.forceWIFI(config);
    }

    ///////////////////////////////////////////////////
    ///////////////////////ON BOOT/////////////////////
    ///////////////////////////////////////////////////
    if (bootCount == 0)
    {
        mimir.i2cScanner();
        config.email = "tester@word.com";
        config.serialNumber = "wthsfty5vgu";
        config.deviceName = "Test";

        // mimir.initWIFI(config);
        // sendAuth.email = config.email;
        // sendAuth.serialNumber = config.serialNumber;
        // sendAuth.macAddress = WiFi.macAddress();
        // mimir.WiFi_ON();
        // mimir.sendAuth("https://us-central1-mimirhome-app.cloudfunctions.net/dataTransfer/auth", sendAuth, config);
        // mimir.WiFi_OFF();
    }
    ///////////////////////////////////////////////////
    //////////////////////SEND DATA////////////////////
    ///////////////////////////////////////////////////
    if (bootCount % 3 == 0)
    {
        sendData.auth.deviceId = config.deviceId;
        sendData.auth.userId = config.userId;
        sendData.auth.macAddress = WiFi.macAddress();
        sendData.status = mimir.getStatus();
        sendData.data = data;

        mimir.initPixels();
        mimir.testPixels(1, 100);
        //Everything that happens with the server happens in here
        // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
        // delay(500);
        // mimir.WiFi_ON();
        // mimir.sendData("https://us-central1-mimirhome-app.cloudfunctions.net/dataTransfer/add", sendData);
        // mimir.WiFi_OFF();
        // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector
    }
    ///////////////////////////////////////////////////
    /////////////////////SLEEP TIME////////////////////
    ///////////////////////////////////////////////////
    mimir.saveToSPIFFS(config);
    mimir.printBSECState("Current State", BSECState);
    Serial.print("Boot Count: ");
    Serial.println(bootCount);
    bootCount++;
    mimir.SLEEP(5);
}

void loop()
{
}