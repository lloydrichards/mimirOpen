#include "mimirOpen.h"
#include <FS.h> //this needs to be first, or it all crashes and burns...
#include "SPIFFS.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "time.h"

SPIClass spiSD(HSPI);

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#include <NeoPixelBrightnessBus.h>
#include "Adafruit_SHT31.h"                         //https://github.com/adafruit/Adafruit_SHT31
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h" //https://github.com/sparkfun/SparkFun_Ambient_Light_Sensor_Arduino_Library
#include "bsec.h"
#include <HSCDTD008A.h>

//Define Sensors
Adafruit_SHT31 SHT31 = Adafruit_SHT31();
SparkFun_Ambient_Light VEML6030(addrVEML6030);
Bsec BME680;
HSCDTD008A COMPAS(Wire, addrCompas);

//VEML6030 settings
float gain = .125;
int integTime = 100;

//Define Pixels
#define PIXEL_PIN 19
#define PIXEL_COUNT 5

NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> pixel(PIXEL_COUNT, PIXEL_PIN);

RgbColor red(50, 0, 0);
RgbColor green(0, 50, 0);
RgbColor blue(0, 0, 50);
RgbColor yellow(25, 25, 0);
RgbColor purple(25, 0, 25);
RgbColor lightBlue(0, 25, 25);
RgbColor white(128);
RgbColor black(0);

mimirOpen::mimirOpen(int baudRate)
{
    Serial.begin(baudRate);
    StartTime = millis();
}

void mimirOpen::initPixels(int brightness)
{
    Serial.println("Starting Pixels...");
    pixel.Begin();
    pixel.Show();
    pixel.SetBrightness(brightness);
}

void mimirOpen::initSensors()
{
    if (SHT31.begin(addrSHT31D)) {
        Serial.println("SHT31 Success!");
        STATUS_SHT31 = OKAY;
    }
    else {
        Serial.println("SHT31 Failed!");
        STATUS_SHT31 = ERROR_L;
    }
    if (VEML6030.begin())
    {
        VEML6030.setGain(gain);
        VEML6030.setIntegTime(integTime);
        VEML6030.setPowSavMode(2);
        VEML6030.enablePowSave();
        Serial.println("VEML6030 Success!");
        STATUS_VEML6030 = OKAY;
    }
    else {
        Serial.println("VEML6030 Failed!");
        STATUS_VEML6030 = ERROR_L;
    }
    BME680.begin(addrBME680, Wire);
    if (BME680.status == BSEC_OK) {
        STATUS_BME680 = OKAY;
        Serial.println("BME680 Success!");
    }
    else {
        STATUS_BME680 = ERROR_L;
        Serial.println("BME680 Failed!");
    }
    if (COMPAS.begin())
    {
        COMPAS.calibrate();
        Serial.println("Compass Success!");
        STATUS_COMPAS = OKAY;
    }
    else {
        Serial.println("Compass Failed!");
        STATUS_COMPAS = ERROR_L;
    }
}

void mimirOpen::initMicroSD(String filename)
{
    spiSD.begin(14, 2, 15, 13);
    Serial.println("Initializing SD card...");
    if (!SD.begin(13, spiSD))
    {
        Serial.println("ERROR - SD card initialization failed!");
        return; // init failed
    }
    // If the data.txt file doesn't exist
    // Create a file on the SD card and write the data labels
    File file = SD.open(filename.c_str());
    if (!file)
    {
        Serial.println("File doens't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename.c_str(), "Date,Time,Battery(%),Temperature(SHT31_L),Temperature(SHT31_H),Temperature(BMP280),Altitude(BMP280),Humidity(SHT31_L),Humidity(SHT31_H),Pressure(BMP280),Luminance(VEML6030),UVA(VEML6075),UVB(VEML6075),UVIndex(VEML6075),eCO2(CCS811),tVOC(CCS811),bearing(Compass); \r\n");
    }
    else
    {
        Serial.println("File already exists");
    }
    file.close();
}
void mimirOpen::initWIFI(config _config)
{
    config newConfig;

    WiFiManagerParameter custom_USER_NAME("User Name", "User Name", _config.userName.c_str(), 40);
    WiFiManagerParameter custom_PASSWORD("Password", "Password", "", 40);
    WiFiManagerParameter custom_DEVICE_NAME("Device Name", "Device Name", _config.deviceName.c_str(), 40);

    WiFiManager wifiManager;

    wifiManager.addParameter(&custom_USER_NAME);
    wifiManager.addParameter(&custom_PASSWORD);
    wifiManager.addParameter(&custom_DEVICE_NAME);

    wifiManager.setAPCallback(WiFiCallback);
    wifiManager.autoConnect("mimirOpen WIFI");

    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI();
        SetupTime();
    }
    //Update Device Info with Params
    newConfig.userName = custom_USER_NAME.getValue();
    newConfig.password = custom_PASSWORD.getValue();
    newConfig.deviceName = custom_DEVICE_NAME.getValue();

    saveToSPIFFS(newConfig);
    WiFi_OFF();
}

config mimirOpen::initSPIFFS()
{
    config newConfig;
    if (SPIFFS.begin())
    {
        Serial.println("mounted file system");
        if (SPIFFS.exists("/config.json"))
        {
            //file exists, reading and loading
            Serial.println("reading config file");
            fs::File configFile = SPIFFS.open("/config.json", "r");
            if (configFile)
            {
                Serial.println("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument configJson(1024);
                DeserializationError error = deserializeJson(configJson, buf.get());
                if (error)
                {
                    Serial.println("ERROR load json config");
                    return newConfig;
                }
                Serial.println("\nparsed json");
                serializeJson(configJson, Serial);

                newConfig.userName = configJson["userName"].as<String>();
                newConfig.password = configJson["password"].as<String>();
                newConfig.deviceName = configJson["deviceName"].as<String>();
                newConfig.userId = configJson["userId"].as<String>();
                newConfig.deviceId = configJson["deviceId"].as<String>();
            }
            else
            {
                Serial.println("failed to load json config");
            }
        }
    }
    else
    {
        Serial.println("failed to mount FS");
    }
    return newConfig;
}

///////////////////////////////////////////////////
///////////////////MAIN FUNCTIONS//////////////////
///////////////////////////////////////////////////

envData mimirOpen::readSensors()
{
    envData data;
    bsec_virtual_sensor_t BME680Readings[10] ={
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    BME680.updateSubscription(BME680Readings, 10, BSEC_SAMPLE_RATE_LP);
    BME680.run()?
        STATUS_BME680 = OKAY
        :STATUS_BME680 = ERROR_R;

    float avgTemp = (SHT31.readTemperature()+ BME680.temperature)/2;
    float avgHumi = (SHT31.readHumidity()+ BME680.humidity)/2;
    data.temperature = avgTemp;
    data.humidity = avgHumi;
    data.pressure = BME680.pressure;
    data.altitude = calcAltitude(BME680.pressure, avgTemp);
    data.luminance = (float)VEML6030.readLight();
    data.iaq = BME680.iaq;
    data.eCO2 = BME680.co2Equivalent;
    data.eVOC = BME680.breathVocEquivalent;

    if (COMPAS.measure())
    {
        data.bearing = ((atan2(COMPAS.y(), COMPAS.x())) * 180) / PI;
        STATUS_COMPAS = OKAY;
    }
    else {
        STATUS_COMPAS = ERROR_R;
    }
    return data;
}

void mimirOpen::saveToSPIFFS(config data)
{
    DynamicJsonDocument newConfigJson(1024);
    newConfigJson["userName"] = data.userName;
    newConfigJson["password"] = data.password;
    newConfigJson["deviceName"] = data.deviceName;
    newConfigJson["userId"] = data.userId;
    newConfigJson["deviceId"] = data.deviceId;

    fs::File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
        Serial.println("failed to open config file for writing");
    }

    //serializeJson(newConfigJson, Serial);
    serializeJson(newConfigJson, configFile);
    configFile.close();
}

void mimirOpen::printSensors(envData data)
{
    Serial.print("Temp: ");
    Serial.println(data.temperature);
    Serial.print("Humi: ");
    Serial.println(data.humidity);
    Serial.print("Pres: ");
    Serial.println(data.pressure);
    Serial.print("Alt: ");
    Serial.println(data.altitude);
    Serial.print("Lux: ");
    Serial.println(data.luminance);
    Serial.print("IAQ: ");
    Serial.println(data.iaq);
    Serial.print("eVOC: ");
    Serial.println(data.eVOC);
    Serial.print("eCO2: ");
    Serial.println(data.eCO2);
    Serial.print("Bearing: ");
    Serial.println(data.bearing);
}

void mimirOpen::sendData(String address, package data)
{
    if ((WiFi.status() == WL_CONNECTED))
    {
        String package = packageJSON(data);
        HTTPClient http;
        http.begin(address);
        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.POST(package);
        String response = http.getString();

        if (httpResponseCode > 0)
        {
            Serial.println("Successful Response from Server");
        }
        else
        {
            Serial.println("Failed Response from Server");
        }
        http.end();
    }
}

///////////////////////////////////////////////////
/////////////////HELPER FUNCTIONS/////////////////
///////////////////////////////////////////////////

int mimirOpen::getBatteryPercent() {
    int percent = 0;
    return percent;
};

systems mimirOpen::getStatus() {
    systems current;
    current.battery = STATUS_BATTERY;
    current.batteryPercent = getBatteryPercent();
    current.sd = STATUS_SD;
    current.server = STATUS_SERVER;
    current.wifi = STATUS_WIFI;
    current.BME680 = STATUS_BME680;
    current.COMPAS = STATUS_COMPAS;
    current.SHT31 = STATUS_SHT31;
    current.VEML6030 = STATUS_VEML6030;
    return current;
};

float mimirOpen::calcAltitude(float pressure, float temperature) {
    float SEALEVELPRESSURE_HPA = 1013.25;
    float altitude = 44330 * (1.0 - pow((pressure/100) / SEALEVELPRESSURE_HPA, 0.1903));
    return altitude;
};

float mimirOpen::averageValue(float values[])
{
    float sum;
    int total = 0;
    for (int i = 0; i < int(sizeof(values)); i++)
    {
        if (isnan(values[i]) || values[i] != 0)
        {
            sum += values[i];
            total += 1;
        }
    }
    if (total > 0)
    {
        return sum / total;
    }
    else
        return 0;
};

void mimirOpen::writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.print(message) ? Serial.println("File written") : Serial.println("Write failed");
    file.close();
}

void mimirOpen::appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    file.print(message) ? Serial.println("Message appended") : Serial.println("Append failed");
    file.close();
}

void mimirOpen::logData(envData data, String filename)
{
    systems status;
    status.battery = STATUS_BATTERY;
    status.wifi = STATUS_WIFI;
    status.sd = STATUS_SD;
    status.server = STATUS_SERVER;
    status.BME680 = STATUS_BME680;
    status.COMPAS = STATUS_COMPAS;
    status.SHT31 = STATUS_SHT31;
    status.VEML6030 = STATUS_VEML6030;

    Serial.print("Saving data");
    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("File doens't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename.c_str(), header().c_str());
    }
    file.close();
    appendFile(SD, filename.c_str(), stringData(data, status).c_str());
}

String mimirOpen::packageJSON(package _data)
{
    DynamicJsonDocument package(1024);
    String output;

    JsonObject auth = package.createNestedObject("auth");
    auth["userId"] = _data.auth.userId;
    auth["deviceId"] = _data.auth.deviceId;
    auth["macAddress"] = _data.auth.macAddress;

    JsonObject status = package.createNestedObject("status");
    status["Date"] = DateStr;
    status["Time"] = TimeStr;
    status["battery"] = _data.status.battery;
    status["batteryPercent"] = _data.status.batteryPercent;
    status["wifi"] = _data.status.wifi;
    status["sd"] = _data.status.sd;
    status["server"] = _data.status.server;
    status["BME680"] = _data.status.BME680;
    status["COMPAS"] = _data.status.COMPAS;
    status["SHT31"] = _data.status.SHT31;
    status["VEML6030"] = _data.status.VEML6030;

    JsonObject data = package.createNestedObject("data");

    data["temperature"] = _data.data.temperature;
    data["humidity"] = _data.data.humidity;
    data["pressure"] = _data.data.pressure;
    data["altitude"] = _data.data.altitude;
    data["luminance"] = _data.data.luminance;
    data["iaq"] = _data.data.iaq;
    data["eVOC"] = _data.data.eVOC;
    data["eCO2"] = _data.data.eCO2;
    data["bearing"] = _data.data.bearing;

    serializeJsonPretty(package, output);
    return output;
}

String mimirOpen::stringData(envData data, systems status)
{
    return DateStr + "," +
        TimeStr + "," +
        status.battery +
        "," +
        status.batteryPercent +
        "," +
        status.wifi +
        "," +
        status.sd +
        "," +
        status.server +
        "," +
        status.BME680 +
        "," +
        status.COMPAS +
        "," +
        status.SHT31 +
        "," +
        status.VEML6030 +
        "," +
        data.temperature +
        "," +
        data.humidity +
        "," +
        data.pressure +
        "," +
        data.altitude +
        "," +
        data.luminance +
        "," +
        data.iaq +
        "," +
        data.eVOC +
        "," +
        data.eCO2 +
        "," +
        data.bearing + "\r\n";
}

String mimirOpen::header()
{
    String output = "date,time,battery,batteryPercent,wifi,sd,server,BME680,COMPAS,SHT31,VEML6030,temperature,humidity,pressure,altitude,luminance,iaq,eVOC,eCO2,bearing\r\n";
    return output;
}

void mimirOpen::WiFiCallback(WiFiManager *myWiFiManager)
{
    Serial.println("-WiFiConfig-");
    Serial.println("----Mode----");
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
    Serial.print("Access IP: ");
    Serial.print(WiFi.softAPIP());
}

void mimirOpen::WiFi_ON()
{

    WiFiManager wifiManager;
    wifiManager.autoConnect("mimirAP");
    wifi_signal = WiFi.RSSI();
    SetupTime();
};
void mimirOpen::WiFi_OFF()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
};

void mimirOpen::SLEEP(long interval)
{
    //CONFIG Sleep Pin
    //Serial.println("Config Sleep Pin");
    // gpio_pullup_en(GPIO_NUM_32);    // use pullup on GPIO
    // gpio_pullup_en(GPIO_NUM_33);    // use pullup on GPIO
    // gpio_pulldown_dis(GPIO_NUM_32); // not use pulldown on GPIO
    // gpio_pulldown_dis(GPIO_NUM_33); // not use pulldown on GPIO

    // esp_sleep_enable_ext1_wakeup(0x200800000, ESP_EXT1_WAKEUP_ALL_LOW);

    //CONFIG Sleep Timer
    Serial.println("Config Sleep Timer");                                                           // Wake if GPIO is low
    long SleepTimer = (interval * 60 - ((CurrentMin % interval) * 60 + CurrentSec)) + 30; //Some ESP32 are too fast to maintain accurate time
    esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL);

    Serial.println("Entering " + String(SleepTimer) + "-secs of sleep time");
    Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
    Serial.println("Starting deep-sleep...");

    delay(100);
    pixel.ClearTo(black);
    pixel.Show();

    delay(100);
    esp_deep_sleep_start();
}

bool mimirOpen::SetupTime()
{
    configTime(0, 0, "ch.pool.ntp.org", "time.nist.gov");
    setenv("TZ", TZ_INFO, 1);
    delay(100);
    bool TimeStatus = UpdateLocalTime();
    return TimeStatus;
}

bool mimirOpen::UpdateLocalTime()
{
    struct tm timeinfo;
    char output[30], day_output[30];
    while (!getLocalTime(&timeinfo, 5000))
    { // Wait for up to 5-secs
        Serial.println(F("Failed to obtain time"));
        return false;
    }
    CurrentHour = timeinfo.tm_hour;
    CurrentMin = timeinfo.tm_min;
    CurrentSec = timeinfo.tm_sec;
    //See http://www.cplusplus.com/reference/ctime/strftime/
    //Serial.println(&timeinfo, "%H:%M:%S");                               // Displays: 14:05:49
    strftime(day_output, 30, "%F", &timeinfo);               // Displays: Sat 24-Jun-17
    strftime(output, sizeof(output), "%H:%M:%S", &timeinfo); // Creates: '14:05:49'
    DateStr = day_output;
    TimeStr = output;
    return true;
}
///////////////////////////////////////////////////
/////////////////TESTING FUNCTIONS/////////////////
///////////////////////////////////////////////////

void mimirOpen::i2cScanner()
{
    byte error, address;
    int nDevices;
    Wire.begin();
    Serial.println("Scanning...");
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("Device at 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("ERROR at 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

void mimirOpen::testPixels(int repeat, int _delay)
{
    Serial.println("Testing Pixels...");
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, red);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, green);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, blue);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, yellow);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, purple);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, lightBlue);
        pixel.Show();
        delay(500);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
}

void mimirOpen::testHTTPRequest(String address)
{

    if ((WiFi.status() == WL_CONNECTED))
    {
        HTTPClient http;
        http.begin(address);
        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.GET();
        if (httpResponseCode > 0)
        {
            String response = http.getString();
            Serial.println("Data Sent!");
            Serial.println(httpResponseCode);
            Serial.println(response);
        }
        else
        {
            Serial.println("ERROR");
            Serial.println("Error on sending GET request");
            Serial.println(httpResponseCode);
        }
        http.end();
    }
}