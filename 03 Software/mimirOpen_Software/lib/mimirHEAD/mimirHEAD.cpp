#include "mimirHEAD.h"
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
#include "SparkFun_VEML6075_Arduino_Library.h"
#include <Adafruit_BMP280.h>
#include "ccs811.h" //https://github.com/maarten-pennings/CCS811
#include <HSCDTD008A.h>

//Define Sensors
Adafruit_SHT31 sht31_L = Adafruit_SHT31();
Adafruit_SHT31 sht31_H = Adafruit_SHT31();
SparkFun_Ambient_Light veml6030(addrVEML6030);
VEML6075 veml6075;
CCS811 ccs811(36);
Adafruit_BMP280 bmp280;
HSCDTD008A compass(Wire, addrCompass);

//VEML6030 settings
float gain = .125;
int integTime = 100;

//bmp280 settings
#define SEALEVELPRESSURE_HPA (1013.25)

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

mimirHEAD::mimirHEAD(int baudRate)
{
    Serial.begin(baudRate);
}

void mimirHEAD::initPixels(int brightness)
{
    Serial.println("Starting Pixels...");
    pixel.Begin();
    pixel.Show();
    pixel.SetBrightness(brightness);
}

void mimirHEAD::initSensors()
{
    sht31_L.begin(addrSHT31D_L) ? Serial.println("SHT31_L Success!") : Serial.println("SHT31_L Failed!");
    sht31_H.begin(addrSHT31D_H) ? Serial.println("SHT31_H Success!") : Serial.println("SHT31_H Failed!");
    if (veml6030.begin())
    {
        veml6030.setGain(gain);
        veml6030.setIntegTime(integTime);
        veml6030.setPowSavMode(2);
        veml6030.enablePowSave();
        Serial.println("VEML6030 Success!");
    }
    else
        Serial.println("VEML6030 Failed!");
    veml6075.begin() ? Serial.println("VEML6075 Success!") : Serial.println("VEML6075 Failed!");
    if (ccs811.begin())
    {
        if (ccs811.start(CCS811_MODE_1SEC))
            Serial.println("CCS811 Success!");
    }
    else
        Serial.println("CCS811 Failed!");
    if (compass.begin())
    {
        compass.calibrate();
        Serial.println("Compass Success!");
    }
    else
        Serial.println("Compass Failed!");
    bmp280.begin(addrbmp280) ? Serial.println("BMP280 Success!") : Serial.println("BMP280 Failed!");
}

void mimirHEAD::initMicroSD()
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
    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("File doens't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename, "Date,Time,Battery(%),Temperature(SHT31_L),Temperature(SHT31_H),Temperature(BMP280),Altitude(BMP280),Humidity(SHT31_L),Humidity(SHT31_H),Pressure(BMP280),Luminance(VEML6030),UVA(VEML6075),UVB(VEML6075),UVIndex(VEML6075),eCO2(CCS811),tVOC(CCS811),bearing(Compass); \r\n");
    }
    else
    {
        Serial.println("File already exists");
    }
    file.close();
}
void mimirHEAD::initWIFI()
{
    WiFiManagerParameter custom_USER("User Name", "User Name", _USER, 40);
    WiFiManagerParameter custom_USER_ID("User ID", "User ID", _USER_ID, 40);
    WiFiManagerParameter custom_DEVICE_ID("Device ID", "Device ID", _DEVICE_ID, 40);

    WiFiManager wifiManager;

    wifiManager.addParameter(&custom_USER);
    wifiManager.addParameter(&custom_USER_ID);
    wifiManager.addParameter(&custom_DEVICE_ID);

    wifiManager.setAPCallback(WiFiCallback);

    wifiManager.autoConnect("mimirHEAD WIFI");

    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI();
        SetupTime();
    }

    //Update Device Info with Params
    strcpy(_USER, custom_USER.getValue());
    strcpy(_USER_ID, custom_USER_ID.getValue());
    strcpy(_DEVICE_ID, custom_DEVICE_ID.getValue());

    saveConfig();
    WiFi_OFF();
}

void mimirHEAD::initSPIFFS()
{
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
                    Serial.println("failed to load json config");
                    return;
                }
                Serial.println("\nparsed json");
                serializeJson(configJson, Serial);

                strcpy(_USER, configJson["User"] | "N/A");
                strcpy(_USER_ID, configJson["UserID"] | "N/A");
                strcpy(_DEVICE_ID, configJson["DeviceID"] | "N/A");
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
}

///////////////////////////////////////////////////
///////////////////MAIN FUNCTIONS//////////////////
///////////////////////////////////////////////////

void mimirHEAD::readSensors()
{
    uint16_t eco2, etvoc, errstat, raw;

    temp1 = (float)sht31_L.readTemperature();
    temp2 = (float)sht31_H.readTemperature();
    temp3 = (float)bmp280.readTemperature();
    hum1 = (float)sht31_L.readHumidity();
    hum2 = (float)sht31_H.readHumidity();

    avgTemp = (temp1 + temp2 + temp3) / 3;
    avgHum = (hum1 + hum2) / 2;

    pres = (float)bmp280.readPressure() / 100;
    alt = (float)bmp280.readAltitude(SEALEVELPRESSURE_HPA);

    lux = (float)veml6030.readLight();
    uvA = (float)veml6075.uva();
    uvB = (float)veml6075.uvb();
    uvIndex = (float)veml6075.index();

    float fract = modf(avgTemp, &avgTemp);
    uint16_t tempHIGH = (((uint16_t)avgTemp + 25) << 9);
    uint16_t tempLOW = ((uint16_t)(fract / 0.001953125) & 0x1FF);

    uint16_t tempCONVERT = (tempHIGH | tempLOW);
    uint16_t humCONVERT = avgHum << 1 | 0x00;

    ccs811.set_envdata(tempCONVERT, humCONVERT);
    ccs811.read(&eco2, &etvoc, &errstat, &raw);
    eCO2 = (float)eco2;
    tVOC = (float)etvoc;
    ccs811ERROR = errstat;

    if (compass.measure())
    {
        compassX = compass.x();
        compassY = compass.y();
        compassZ = compass.z();
        bearing = ((atan2(compassY, compassX)) * 180) / PI; //values will range from +180 to -180 degrees
    }
}

void mimirHEAD::printSensors()
{
    Serial.println("SHT31 Low:");
    Serial.print(temp1);
    Serial.println("C");
    Serial.print(hum1);
    Serial.println("%");
    Serial.println("SHT31 High:");
    Serial.print(temp2);
    Serial.println("C");
    Serial.print(hum2);
    Serial.println("%");
    Serial.println("BME280:");
    Serial.print(temp3);
    Serial.println("C");
    Serial.print(pres);
    Serial.println("hPa");
    Serial.print(alt);
    Serial.println("m");
    Serial.println("VEML6030:");
    Serial.print(lux);
    Serial.println("lux");
    Serial.println("VEML6075:");
    Serial.print("uvA");
    Serial.println(uvA);
    Serial.print("uvB");
    Serial.println(uvB);
    Serial.print("uvIndex");
    Serial.println(uvIndex);
    Serial.println("CCS811:");
    Serial.print(eCO2);
    Serial.println("ppm");
    Serial.print(tVOC);
    Serial.println("ppb");
    Serial.println(ccs811.errstat_str(ccs811ERROR));
    Serial.println("Compass:");
    Serial.println(bearing);
}

void mimirHEAD::sendData(String address)
{
    if ((WiFi.status() == WL_CONNECTED))
    {
        String package = packageJSON();
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

void mimirHEAD::writeFile(fs::FS &fs, const char *path, const char *message)
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

void mimirHEAD::appendFile(fs::FS &fs, const char *path, const char *message)
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

void mimirHEAD::logData(String data)
{

    Serial.print("Save data: ");
    Serial.println(data);

    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("File doens't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename, "Date,Time,Battery(%),Temperature(SHT31_L),Temperature(SHT31_H),Temperature(BMP280),Altitude(BMP280),Humidity(SHT31_L),Humidity(SHT31_H),Pressure(BMP280),Luminance(VEML6030),UVA(VEML6075),UVB(VEML6075),UVIndex(VEML6075),eCO2(CCS811),tVOC(CCS811),bearing(Compass); \r\n");
    }
    file.close();
    appendFile(SD, filename, data.c_str());
}

String mimirHEAD::packageJSON()
{
    DynamicJsonDocument package(1024);
    String output;

    JsonObject userInfo = package.createNestedObject("userInfo");
    userInfo["userName"] = _USER;
    userInfo["userId"] = _USER_ID;
    userInfo["deviceId"] = _DEVICE_ID;
    userInfo["ipAddress"] = _IP_ADDRESS;
    userInfo["macAddress"] = WiFi.macAddress();
    userInfo["version"] = MIMIR_VERSION;

    JsonObject status = package.createNestedObject("status");
    status["Battery_Percent"] = batteryPercent;
    status["WiFi_Signal"] = wifi_signal;
    status["Date"] = DateStr;
    status["Time"] = TimeStr;

    JsonObject data = package.createNestedObject("data");

    data["Temperature(SHT31_L)"] = temp1;
    data["Temperature(SHT31_H)"] = temp2;
    data["Temperature(bmp280)"] = temp3;
    data["Humidity(SHT31_L)"] = hum1;
    data["Humidity(SHT31_H)"] = hum2;
    data["Pressure(BMP280)"] = pres;
    data["Altitude(BMP280)"] = pres;
    data["Luminance(VEML6030)"] = lux;
    data["UVA(VEML6075)"] = uvA;
    data["UVB(VEML6075)"] = uvB;
    data["UVIndex(VEML6075)"] = uvIndex;
    data["eCO2(CCS811)"] = eCO2;
    data["VOC(CCS811)"] = tVOC;
    data["Bearing(Compass)"] = bearing;

    serializeJsonPretty(package, output);
    return output;
}

String mimirHEAD::stringData()
{
    return String(DateStr) + "," +
           String(TimeStr) + "," +
           String(batteryPercent) + "," +
           String(temp1) + "," +
           String(temp2) + "," +
           String(temp3) + "," +
           String(alt) + "," +
           String(hum1) + "," +
           String(hum2) + "," +
           String(pres) + "," +
           String(lux) + "," +
           String(uvA) + "," +
           String(uvB) + "," +
           String(uvIndex) + "," +
           String(eCO2) + "," +
           String(tVOC) + "," +
           String(bearing) + "\r\n";
}

void mimirHEAD::WiFiCallback(WiFiManager *myWiFiManager)
{
    Serial.println("-WiFiConfig-");
    Serial.println("----Mode----");
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
    Serial.print("Access IP: ");
    Serial.print(WiFi.softAPIP());
}

void mimirHEAD::WiFi_ON()
{

    WiFiManager wifiManager;
    wifiManager.autoConnect("mimirAP");
    wifi_signal = WiFi.RSSI();
    SetupTime();
};
void mimirHEAD::WiFi_OFF()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
};

void mimirHEAD::SLEEP()
{
    saveConfig();
    //CONFIG Sleep Pin
    Serial.println("Config Sleep Pin");
    gpio_pullup_en(GPIO_NUM_39);    // use pullup on GPIO
    gpio_pulldown_dis(GPIO_NUM_39); // not use pulldown on GPIO

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 0);

    //CONFIG Sleep Timer
    Serial.println("Config Sleep Timer");                                                           // Wake if GPIO is low
    long SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)) + 30; //Some ESP32 are too fast to maintain accurate time
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

void mimirHEAD::saveConfig()
{
    DynamicJsonDocument newConfigJson(1024);
    newConfigJson["User"] = _USER;
    newConfigJson["UserID"] = _USER_ID;
    newConfigJson["DeviceID"] = _DEVICE_ID;

    fs::File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
        Serial.println("failed to open config file for writing");
    }

    //serializeJson(newConfigJson, Serial);
    serializeJson(newConfigJson, configFile);
    configFile.close();
}

bool mimirHEAD::SetupTime()
{
    configTime(0, 0, "ch.pool.ntp.org", "time.nist.gov");
    setenv("TZ", TZ_INFO, 1);
    delay(100);
    bool TimeStatus = UpdateLocalTime();
    return TimeStatus;
}

bool mimirHEAD::UpdateLocalTime()
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
    createFileName(day_output);
    DateStr = day_output;
    TimeStr = output;
    return true;
}

void mimirHEAD::createFileName(char date[])
{
    filename[1] = date[0];
    filename[2] = date[1];
    filename[3] = date[2];
    filename[4] = date[3];
    filename[5] = date[4];
    filename[6] = date[5];
    filename[7] = date[6];
    filename[8] = date[7];
    filename[9] = date[8];
    filename[10] = date[9];
}
///////////////////////////////////////////////////
/////////////////TESTING FUNCTIONS/////////////////
///////////////////////////////////////////////////

void mimirHEAD::i2cScanner()
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

void mimirHEAD::testPixels(int repeat, int _delay)
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

void mimirHEAD::testHTTPRequest(String address)
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