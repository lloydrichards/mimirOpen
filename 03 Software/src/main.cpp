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
#include <NeoPixelBus.h>

mimirOpen mimir(115200);
DataPackage sendData;

RgbColor RED(128, 0, 0);
RgbColor YELLOW(32, 32, 0);
RgbColor GREEN(0, 64, 0);
RgbColor BLACK(0, 0, 0);

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool offlineMode = false;
RTC_DATA_ATTR uint8_t BSECState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
RTC_DATA_ATTR int64_t BSECTime = 0;

void setup()
{
    ///////////////////////////////////////////////////
    //////////////////////STARTUP//////////////////////
    ///////////////////////////////////////////////////
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    // int wakeUpGPIONumber = int(log(esp_sleep_get_ext1_wakeup_status()) / log(2));
    // switch (wakeUpGPIONumber)
    // {
    // case ENVIRO_PIN:
    //     Serial.println("ENVIRO Button Pushed!");
    //     return;
    // case MONITOR_PIN:
    //     Serial.println("MONITOR Button Pushed!");
    //     return;
    // case RADAR_PIN:
    //     Serial.println("RADAR Button Pushed!");
    //     return;
    // default:
    //     mimir.printBootReason();
    //     return;
    // }
    if (digitalRead(ENVIRO_PIN) == LOW && digitalRead(MONITOR_PIN) == LOW)
    {
        offlineMode = !offlineMode;

        mimir.turnOFFPixels();
        if (offlineMode)
        {
            Serial.println("Offline Mode");
        }
        else
        {
            Serial.println("Online Mode");
        }
    }
    mimir.i2cScanner();
    String filename = "/" + mimir.dateToString() + ".txt";
    Serial.println(filename);
    config config = mimir.initSPIFFS();

    if (bootCount == 0)
    {
        mimir.initPixels();
        mimir.pixelBootUp();
        mimir.pixelSystemStatus(BATTERY, GREEN);
        mimir.pixelSystemStatus(MICROSD, YELLOW);
        mimir.initMicroSD(filename) ? mimir.pixelSystemStatus(MICROSD, GREEN) : mimir.pixelSystemStatus(MICROSD, RED);
        mimir.pixelSystemStatus(SENSORS, YELLOW);
        mimir.initSensors(BSECState, BSECTime) ? mimir.pixelSystemStatus(SENSORS, GREEN) : mimir.pixelSystemStatus(SENSORS, RED);
    }
    else
    {
        mimir.initSensors(BSECState, BSECTime);
        mimir.initMicroSD(filename);
    }

    envData data = mimir.readSensors(BSECState, BSECTime);
    mimir.printSensors(data);

    ///////////////////////////////////////////////////
    /////////////////////MODE CHANGE///////////////////
    ///////////////////////////////////////////////////
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        mimir.initPixels();
        Serial.println("\nMonitor Button was pressed!");
        mimir.changeMode(&config, 8000);
    }

    ///////////////////////////////////////////////////
    /////////////////////FORCED MODE///////////////////
    ///////////////////////////////////////////////////
    if (digitalRead(ENVIRO_PIN) == LOW && digitalRead(RADAR_PIN) == LOW)
    {
        mimir.initPixels();
        Serial.println("\nEntering Forced Mode");
        mimir.forceWIFI(&config);
    }

    ///////////////////////////////////////////////////
    ///////////////////////ON BOOT/////////////////////
    ///////////////////////////////////////////////////
    if (bootCount == 0 && !offlineMode)
    {
        mimir.pixelSystemStatus(WIFI, YELLOW);
        mimir.initWIFI(&config) ? mimir.pixelSystemStatus(WIFI, GREEN) : mimir.pixelSystemStatus(WIFI, RED);

        sendData.auth.deviceId = mimir.getDeviceId();
        sendData.auth.userId = config.userId;
        sendData.auth.email = config.email;
        sendData.auth.macAddress = WiFi.macAddress();
        sendData.status = mimir.getStatus();
        sendData.status.bootCount = bootCount;
        sendData.data = data;

        mimir.WiFi_OFF();
    }
    ///////////////////////////////////////////////////
    //////////////////////SEND DATA////////////////////
    ///////////////////////////////////////////////////
    if (bootCount % 3 == 0 && bootCount != 0 && !offlineMode)
    {
        sendData.auth.deviceId = mimir.getDeviceId();
        sendData.auth.userId = config.userId;
        sendData.auth.email = config.email;
        sendData.auth.macAddress = WiFi.macAddress();
        sendData.status = mimir.getStatus();
        sendData.status.bootCount = bootCount;
        sendData.data = data;
        //Everything that happens with the server happens in here
        mimir.WiFi_ON();
        mimir.sendData("https://us-central1-mimirhome-app.cloudfunctions.net/dataTransfer/add", sendData, &config);
        mimir.WiFi_OFF();
    }
    ///////////////////////////////////////////////////
    /////////////////////SLEEP TIME////////////////////
    ///////////////////////////////////////////////////

    mimir.logData(data, filename);
    mimir.saveToSPIFFS(config);
    Serial.println("\n-----------------");
    mimir.printTimeDate();
    Serial.print("Device ID: ");
    Serial.println(mimir.getDeviceId());
    Serial.print("Boot Count: ");
    Serial.println(bootCount);
    bootCount++;
    mimir.SLEEP(5);
}

void loop(){};