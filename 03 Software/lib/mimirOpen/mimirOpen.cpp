#include "mimirOpen.h"
#include <FS.h> //this needs to be first, or it all crashes and burns...
#include "SPIFFS.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "time.h"
#include <EEPROM.h>
#include "driver/adc.h"
#include <esp_bt.h>

SPIClass spiSD(HSPI);

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <NeoPixelBus.h>

#include "Adafruit_SHT31.h"                         //https://github.com/adafruit/Adafruit_SHT31
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h" //https://github.com/sparkfun/SparkFun_Ambient_Light_Sensor_Arduino_Library
#include "bsec.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include "Adafruit_LC709203F.h"

//Define Sensors
Adafruit_SHT31 SHT31 = Adafruit_SHT31();
SparkFun_Ambient_Light VEML6030(addrVEML6030);
Bsec BME680;
Adafruit_LIS2MDL LSM303 = Adafruit_LIS2MDL();
Adafruit_LC709203F LC709;

//BME680 State
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_300s_4d/bsec_iaq.txt"
};
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;
bsec_virtual_sensor_t BME680Readings[10] = {
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

//VEML6030 settings
float gain = .125;
int integTime = 100;

//Define Pixels
#define colorSaturation 128

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixel(PIXEL_COUNT, PIXEL_PIN);

RgbColor red(colorSaturation, 0, 0);
RgbColor yellow(64, 64, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor teal(0, 64, 64);
RgbColor blue(0, 0, colorSaturation);
RgbColor violet(64, 0, 64);
RgbColor white(colorSaturation);
RgbColor black(0);

mimirOpen::mimirOpen(int baudRate)
{
    adc_power_on();
    Serial.begin(baudRate);
    StartTime = millis();
    pinMode(MONITOR_PIN, INPUT_PULLUP);
    pinMode(RADAR_PIN, INPUT_PULLUP);
    pinMode(ENVIRO_PIN, INPUT_PULLUP);
}

void mimirOpen::initPixels()
{
    Serial.println("Starting Pixels...");
    pinMode(PIXEL_PWR_PIN, OUTPUT);
    digitalWrite(PIXEL_PWR_PIN, LOW);
    pixel.Begin();
    pixel.Show();
}

bool mimirOpen::initSensors(uint8_t *BSECState, int64_t &BSECTime)
{
    bool connectSHT31 = false;
    bool connectVEML6030 = false;
    bool connectBME680 = false;
    bool connectLSM303 = false;
    bool connectLC709 = false;

    if (LC709.begin())
    {
        Serial.println("LC709 Success!");
        LC709.setPackSize(LC709203F_APA_2000MAH);
        LC709.setPowerMode(LC709203F_POWER_OPERATE);
        connectLC709 = true;
    }
    else
    {
        Serial.println("LC709 Failed!");
    }

    if (SHT31.begin(addrSHT31D))
    {
        Serial.println("SHT31 Success!");
        STATUS_SHT31 = OKAY;
        connectSHT31 = true;
    }
    else
    {
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
        connectVEML6030 = true;
    }
    else
    {
        Serial.println("VEML6030 Failed!");
        STATUS_VEML6030 = ERROR_L;
    }
    BME680.begin(addrBME680, Wire);
    if (BME680.status == BSEC_OK)
    {

        BME680.setConfig(bsec_config_iaq);
        STATUS_BME680 = OKAY;
        connectBME680 = true;
        Serial.println("BME680 Success!");
        if (BSECTime)
        {
            BME680.setState(BSECState);
        }
        else
        {
            Serial.println("Saved state missing");
        }
        BME680.updateSubscription(BME680Readings, 10, BSEC_SAMPLE_RATE_ULP);
    }
    else
    {
        STATUS_BME680 = ERROR_L;
        Serial.println("BME680 Failed!");
    }
    if (LSM303.begin())
    {
        Serial.println("LSM303 Success!");
        STATUS_LSM303 = OKAY;
        connectLSM303 = true;
    }
    else
    {
        Serial.println("LSM303 Failed!");
        STATUS_LSM303 = ERROR_L;
    }

    return connectSHT31 && connectVEML6030 && connectBME680 && connectLSM303 && connectLC709;
}

bool mimirOpen::initMicroSD(String filename)
{
    bool connected = false;
    pinMode(13, OUTPUT);
    spiSD.begin(14, 2, 15, 13);
    Serial.println("Initializing SD card...");
    if (!SD.begin(13, spiSD))
    {
        Serial.println("ERROR - SD card initialization failed!");
        return connected; // init failed
    }
    // If the data.txt file doesn't exist
    // Create a file on the SD card and write the data labels
    File file = SD.open(filename.c_str());
    if (!file)
    {
        Serial.println("File doesn't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename.c_str(), "Date,Time,Battery(%),Temperature(SHT31_L),Temperature(SHT31_H),Temperature(BMP280),Altitude(BMP280),Humidity(SHT31_L),Humidity(SHT31_H),Pressure(BMP280),Luminance(VEML6030),UVA(VEML6075),UVB(VEML6075),UVIndex(VEML6075),eCO2(BME680),tVOC(CCS811),bearing(Compass); \r\n");
        connected = true;
    }
    else
    {
        Serial.println("File already exists");
        connected = true;
    }
    file.close();
    return connected;
}
bool mimirOpen::initWIFI(config *config)
{
    bool connected = false;
    WiFiManagerParameter custom_EMAIL("Email", "Email", config->email.c_str(), 40, " type='email'");
    WiFiManagerParameter custom_DEVICE_NAME("Device Name", "Device Name", config->deviceName.c_str(), 40);
    WiFiManagerParameter introduction("<div><h3>Setting Up</h3><p>Lets get you setup with your new device!</p><ol><li>Go to the mimirHome app and add the device to your portal (SerialNumber is on the back of device)</li><li>Select your wifi network and enter you SSID password.</li><li>Enter you user infomation below and hit 'Save'</li><li>You will see some lights flash and when all are green then you are good to go!</li></ol></div>");
    WiFiManagerParameter contact("<p>This is just a text paragraph</p>");

    WiFiManager wifiManager;

    wifiManager.setDebugOutput(true);

    wifiManager.setCustomHeadElement("<script>Array.prototype.forEach.call(document.body.querySelectorAll('*[data-mask]'), applyDataMask); function applyDataMask(field) {var mask = field.dataset.mask.split('');function stripMask(maskedData){function isDigit(char){    return /\d /.test(char);}return maskedData.split('').filter(isDigit);} function applyMask(data){    return mask.map(function(char) {if (char != '_')    return char;if (data.length == 0)    return char;return data.shift();               })        .join('')} function reapplyMask(data){return applyMask(stripMask(data));}function changed(){var oldStart = field.selectionStart;var oldEnd = field.selectionEnd;field.value = reapplyMask(field.value);field.selectionStart = oldStart;field.selectionEnd = oldEnd;}field.addEventListener('click', changed)field.addEventListener('keyup', changed)}</script>");
    wifiManager.setCustomHeadElement("<style></style>");
    wifiManager.addParameter(&introduction);
    wifiManager.addParameter(&custom_EMAIL);
    wifiManager.addParameter(&custom_DEVICE_NAME);
    wifiManager.addParameter(&contact);

    wifiManager.setAPCallback(WiFiCallback);
    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setConnectTimeout(30);
    if (!wifiManager.autoConnect("mimirOpen WIFI"))
    {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        Serial.println("Going to sleep now");
        SLEEP(5);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI();
        SetupTime();
        connected = true;
    }
    //Update Device Info with Params
    config->email = custom_EMAIL.getValue();
    config->deviceName = custom_DEVICE_NAME.getValue();

    WiFi_OFF();
    return connected;
}

void mimirOpen::forceWIFI(config *config)
{
    WiFiManagerParameter custom_EMAIL("Email", "Email", config->email.c_str(), 40, " type='email'");
    WiFiManagerParameter custom_DEVICE_NAME("Device Name", "Device Name", config->deviceName.c_str(), 40);
    WiFiManagerParameter introduction("<div><h3>Setting Up</h3><p>Lets get you setup with your new device!</p><ol><li>Go to the mimirHome app and add the device to your portal (SerialNumber is on the back of device)</li><li>Select your wifi network and enter you SSID password.</li><li>Enter you user infomation below and hit 'Save'</li><li>You will see some lights flash and when all are green then you are good to go!</li></ol></div>");
    WiFiManagerParameter contact("<p>This is just a text paragraph</p>");

    WiFiManager wifiManager;

    wifiManager.setCustomHeadElement("<script>Array.prototype.forEach.call(document.body.querySelectorAll('*[data-mask]'), applyDataMask); function applyDataMask(field) {var mask = field.dataset.mask.split('');function stripMask(maskedData){function isDigit(char){    return /\d /.test(char);}return maskedData.split('').filter(isDigit);} function applyMask(data){    return mask.map(function(char) {if (char != '_')    return char;if (data.length == 0)    return char;return data.shift();               })        .join('')} function reapplyMask(data){return applyMask(stripMask(data));}function changed(){var oldStart = field.selectionStart;var oldEnd = field.selectionEnd;field.value = reapplyMask(field.value);field.selectionStart = oldStart;field.selectionEnd = oldEnd;}field.addEventListener('click', changed)field.addEventListener('keyup', changed)}</script>");
    wifiManager.setCustomHeadElement("<style></style>");
    wifiManager.addParameter(&introduction);
    wifiManager.addParameter(&custom_EMAIL);
    wifiManager.addParameter(&custom_DEVICE_NAME);
    wifiManager.addParameter(&contact);

    wifiManager.setAPCallback(WiFiCallback);
    if (!wifiManager.startConfigPortal("mimirOpen Forced"))
    {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI();
        SetupTime();
    }
    //Update Device Info with Params
    config->email = custom_EMAIL.getValue();
    config->deviceName = custom_DEVICE_NAME.getValue();

    WiFi_OFF();
}

config mimirOpen::initSPIFFS()
{
    config newConfig;
    if (SPIFFS.begin(true))
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
                serializeJsonPretty(configJson, Serial);

                newConfig.email = configJson["email"].as<String>();
                newConfig.deviceName = configJson["deviceName"].as<String>();
                newConfig.userId = configJson["userId"].as<String>();
                newConfig.mode = configJson["mode"].as<int>();
            }
            else
            {
                Serial.println("failed to load json config");
            }
            configFile.close();
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

config mimirOpen::updateConfig()
{
    config newConfig;
    Serial.println("Updating Config...");
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
            serializeJsonPretty(configJson, Serial);

            newConfig.email = configJson["email"].as<String>();
            newConfig.deviceName = configJson["deviceName"].as<String>();
            newConfig.userId = configJson["userId"].as<String>();
            newConfig.mode = configJson["mode"].as<int>();
        }
        else
        {
            Serial.println("failed to load json config");
        }
        configFile.close();
    }
    return newConfig;
}

envData mimirOpen::readSensors(uint8_t *BSECstate, int64_t &BSECTime)
{
    envData data;
    sensors_event_t event;

    if (BME680.run(getTimestamp()))
    {
        STATUS_BME680 = OKAY;
        BSECTime = getTimestamp();
        BME680.getState(BSECstate);
    }
    else
    {
        STATUS_BME680 = ERROR_R;
    }

    float avgTemp = (SHT31.readTemperature() + BME680.temperature) / 2;
    float avgHumi = (SHT31.readHumidity() + BME680.humidity) / 2;
    data.temperature = avgTemp;
    data.humidity = avgHumi;
    data.pressure = BME680.pressure;
    data.altitude = calcAltitude(BME680.pressure, avgTemp);
    data.luminance = (float)VEML6030.readLight();
    data.iaq = BME680.iaq;
    data.iaqAccuracy = BME680.iaqAccuracy;
    data.eCO2 = BME680.co2Equivalent;
    data.eVOC = BME680.breathVocEquivalent;
    data.batteryVoltage = LC709.cellVoltage();
    data.batteryPercentage = LC709.cellPercent();

    LSM303.getEvent(&event);
    data.bearing = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PI;

    return data;
}

void mimirOpen::saveToSPIFFS(config data)
{
    Serial.println("Saving to SPIFFS");
    DynamicJsonDocument newConfigJson(1024);
    newConfigJson["email"] = data.email;
    newConfigJson["deviceName"] = data.deviceName;
    newConfigJson["userId"] = data.userId;
    newConfigJson["mode"] = data.mode;

    fs::File configFile = SPIFFS.open("/config.json", FILE_WRITE);
    if (!configFile)
    {
        Serial.println("failed to open config file for writing");
    }
    //serializeJson(newConfigJson, Serial);
    serializeJson(newConfigJson, configFile);
    Serial.println("Successfully saved to SPIFFS");
    serializeJsonPretty(newConfigJson, Serial);
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
    Serial.print("IAQ Accuracy: ");
    Serial.println(data.iaqAccuracy);
    Serial.print("eVOC: ");
    Serial.println(data.eVOC);
    Serial.print("eCO2: ");
    Serial.println(data.eCO2);
    Serial.print("Bearing: ");
    Serial.println(data.bearing);
    Serial.print("Battery (V): ");
    Serial.println(data.batteryVoltage);
    Serial.print("Battery (%): ");
    Serial.println(data.batteryPercentage);
}

bool mimirOpen::sendData(String address, DataPackage data, config *config)
{
    bool connected = false;
    if ((WiFi.status() == WL_CONNECTED))
    {
        Serial.println("Sending Data...");
        String package = packageJSON(data);
        HTTPClient http;
        http.begin(address);
        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.POST(package);
        String response = http.getString();

        if (httpResponseCode > 0)
        {
            StaticJsonDocument<200> doc;
            Serial.println("Successful Sent Data!");
            Serial.println(response);
            deserializeJson(doc, response);
            const char *userId = doc["user_id"];
            Serial.print("User ID: ");
            Serial.println(userId);

            config->userId = userId;
            connected = true;
        }
        else
        {
            Serial.println("Failed Send Data");
            Serial.println(response);
        }
        http.end();
    }
    return connected;
}

bool mimirOpen::changeMode(config *config, int wait)
{
    Serial.println("Starting Change Mode Wait...");
    unsigned long currentMills = 0;
    currentMills = millis();
    bool nextTask = false;
    while (!nextTask)
    {
        if (millis() > currentMills + wait)
        {
            Serial.println("Mode Change - Timed Out");
            nextTask = true;
        }
        for (int i = 0; i < PIXEL_COUNT; i++)
        {
            pixel.SetPixelColor(i, teal);
        }
        pixel.Show();

        if (digitalRead(ENVIRO_PIN) == LOW)
        {
            Serial.println("ENVIRO MODE Set!");
            pixel.SetPixelColor(0, red);
            pixel.SetPixelColor(1, red);
            pixel.Show();
            config->mode = 0;
            nextTask = true;
        }
        if (digitalRead(MONITOR_PIN) == LOW)
        {
            Serial.println("MONITOR MODE Set!");
            pixel.SetPixelColor(1, red);
            pixel.SetPixelColor(2, red);
            pixel.SetPixelColor(3, red);
            pixel.Show();
            config->mode = 1;
            nextTask = true;
        }
        if (digitalRead(RADAR_PIN) == LOW)
        {
            Serial.println("RADAR MODE Set!");
            pixel.SetPixelColor(3, red);
            pixel.SetPixelColor(4, red);
            pixel.Show();
            config->mode = 2;
            nextTask = true;
        }
    }
    return true;
}

///////////////////////////////////////////////////
/////////////////HELPER FUNCTIONS/////////////////
///////////////////////////////////////////////////

String mimirOpen::getDeviceId()
{
    byte mac[6];
    esp_efuse_read_mac(mac);

    String deviceId = VERSION + String(mac[5], HEX) + String(mac[3], HEX) + String(mac[2], HEX) + String(mac[1], HEX) + String(mac[4], HEX) + String(mac[0], HEX) + String(mac[1], HEX) + String(mac[4], HEX);
    return deviceId;
};

void mimirOpen::printTimeDate()
{
    struct tm timeinfo;

    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
String mimirOpen::dateToString()
{
    struct tm timeinfo;
    char output[11];
    while (!getLocalTime(&timeinfo, 1000))
    { // Wait for up to 1-secs
        Serial.println(F("Failed to obtain time"));
        return "00-00-0000";
    }
    //See http://www.cplusplus.com/reference/ctime/strftime/
    //Serial.println(&timeinfo, "%H:%M:%S");                               // Displays: 14:05:49
    strftime(output, sizeof(output), "%d-%m-%Y", &timeinfo); // Creates: '14:05:49'

    return output;
}

int64_t mimirOpen::getTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}
bool mimirOpen::checkBSECSensor()
{
    if (BME680.status < BSEC_OK)
    {
        Serial.println("BSEC error, status %d!" + String(BME680.status));
        return false;
    }
    else if (BME680.status > BSEC_OK)
    {
        Serial.println("BSEC warning, status %d!" + String(BME680.status));
    }

    if (BME680.bme680Status < BME680_OK)
    {
        Serial.println("Sensor error, bme680_status %d!" + String(BME680.bme680Status));
        return false;
    }
    else if (BME680.bme680Status > BME680_OK)
    {
        Serial.println("Sensor warning, status %d!" + String(BME680.bme680Status));
    }

    return true;
}

void mimirOpen::printBSECState(const char *name, const uint8_t *state)
{
    Serial.println(name);
    for (int i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        Serial.printf("%02x ", state[i]);
        if (i % 16 == 15)
        {
            Serial.print("\n");
        }
    }
    Serial.print("\n");
}

void mimirOpen::checkBSECStatus()
{
    String output;
    if (BME680.status != BSEC_OK)
    {
        if (BME680.status < BSEC_OK)
        {
            output = "BSEC error code : " + String(BME680.status);
            Serial.println(output);
        }
        else
        {
            output = "BSEC warning code : " + String(BME680.status);
            Serial.println(output);
        }
    }
    if (BME680.bme680Status != BME680_OK)
    {
        if (BME680.bme680Status < BME680_OK)
        {
            output = "BME680 error code : " + String(BME680.bme680Status);
            Serial.println(output);
        }
        else
        {
            output = "BME680 warning code : " + String(BME680.bme680Status);
            Serial.println(output);
        }
    }
    BME680.status = BSEC_OK;
}

systems mimirOpen::getStatus()
{
    systems current;
    current.battery = STATUS_BATTERY;
    current.sd = STATUS_SD;
    current.server = STATUS_SERVER;
    current.wifi = STATUS_WIFI;
    current.BME680 = STATUS_BME680;
    current.LSM303 = STATUS_LSM303;
    current.SHT31 = STATUS_SHT31;
    current.VEML6030 = STATUS_VEML6030;
    current.MODE = STATUS_MODE;
    return current;
};

void mimirOpen::pixelStatus(systems sys, int duration)
{
    pixel.SetPixelColor(0, pixelBatteryColour(sys.battery));
    pixel.SetPixelColor(1, pixelSystemColour(sys.sd));
    pixel.SetPixelColor(2, pixelSystemColour(sys.BME680));
    pixel.SetPixelColor(3, pixelSystemColour(sys.server));
    pixel.SetPixelColor(4, pixelSystemColour(sys.wifi));
    pixel.Show();
    delay(duration);
    turnOFFPixels();
}

void mimirOpen::pixelSystemBusy(SYS_PIXEL system, RgbColor colour)
{
    pixel.SetPixelColor(system, colour);
    pixel.Show();
}

void mimirOpen::pixelSystemStatus(SYS_PIXEL system, RgbColor colour)
{
    pixel.SetPixelColor(system, colour);
    pixel.Show();
}

RgbColor mimirOpen::pixelSystemColour(SYS_STATUS stat)
{
    switch (stat)
    {
    case ERROR_L:
        return red;
    case ERROR_R:
        return red;
    case ERROR_W:
        return red;
    case UNMOUNTED:
        return black;
    case OKAY:
        return green;
    default:
        return black;
    };
}
RgbColor mimirOpen::pixelBatteryColour(BAT_STATUS battery)
{
    switch (battery)
    {
    case CRITICAL_BATTERY:
        return red;
    case LOW_BATTERY:
        return yellow;
    case GOOD_BATTERY:
        return green;
    case FULL_BATTERY:
        return green;
    case CHARGING:
        return blue;
    default:
        return black;
    };
}

float mimirOpen::calcAltitude(float pressure, float temperature)
{
    float SEALEVELPRESSURE_HPA = 1013.25;
    float altitude = 44330 * (1.0 - pow((pressure / 100) / SEALEVELPRESSURE_HPA, 0.1903));
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
    status.LSM303 = STATUS_LSM303;
    status.SHT31 = STATUS_SHT31;
    status.VEML6030 = STATUS_VEML6030;
    status.MODE = STATUS_MODE;

    Serial.print("Saving data");
    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("File doesn't exist");
        Serial.println("Creating file...");
        writeFile(SD, filename.c_str(), header().c_str());
    }
    file.close();
    appendFile(SD, filename.c_str(), stringData(data, status).c_str());
}

String mimirOpen::packageJSON(DataPackage _data)
{
    DynamicJsonDocument package(1024);
    String output;

    JsonObject auth = package.createNestedObject("auth");
    auth["user_id"] = _data.auth.userId;
    auth["device_id"] = _data.auth.deviceId;
    auth["email"] = _data.auth.email;
    auth["macAddress"] = _data.auth.macAddress;

    JsonObject status = package.createNestedObject("status");
    status["date"] = DateStr;
    status["time"] = TimeStr;
    status["bootCount"] = _data.status.bootCount;
    status["battery"] = _data.status.battery;
    status["wifi"] = _data.status.wifi;
    status["sd"] = _data.status.sd;
    status["server"] = _data.status.server;
    status["BME680"] = _data.status.BME680;
    status["LSM303"] = _data.status.LSM303;
    status["SHT31"] = _data.status.SHT31;
    status["VEML6030"] = _data.status.VEML6030;
    status["LC709"] = _data.status.LC709;
    status["MODE"] = _data.status.MODE;

    JsonObject data = package.createNestedObject("data");

    data["temperature"] = _data.data.temperature;
    data["humidity"] = _data.data.humidity;
    data["pressure"] = _data.data.pressure;
    data["altitude"] = _data.data.altitude;
    data["luminance"] = _data.data.luminance;
    data["iaq"] = _data.data.iaq;
    data["iaqAccuracy"] = _data.data.iaqAccuracy;
    data["eVOC"] = _data.data.eVOC;
    data["eCO2"] = _data.data.eCO2;
    data["bearing"] = _data.data.bearing;
    data["batteryVoltage"] = _data.data.batteryVoltage;
    data["batteryPercent"] = _data.data.batteryPercentage;

    serializeJsonPretty(package, output);
    return output;
}

String mimirOpen::stringData(envData data, systems status)
{
    return DateStr + "," +
           TimeStr + "," +
           status.battery +
           "," +
           status.wifi +
           "," +
           status.sd +
           "," +
           status.server +
           "," +
           status.BME680 +
           "," +
           status.LSM303 +
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
           data.iaqAccuracy +
           "," +
           data.eVOC +
           "," +
           data.eCO2 +
           "," +
           data.bearing +
           "," +
           data.batteryVoltage +
           "," +
           data.batteryPercentage + "\r\n";
}

String mimirOpen::header()
{
    String output = "date,time,battery,batteryPercent,wifi,sd,server,BME680,COMPAS,SHT31,VEML6030,temperature,humidity,pressure,altitude,luminance,iaq,iaqAccuracy,eVOC,eCO2,bearing,batteryVoltage,batteryPercentage\r\n";
    return output;
}

void mimirOpen::WiFiCallback(WiFiManager *myWiFiManager)
{
    pixel.SetPixelColor(1, black);
    pixel.SetPixelColor(2, black);
    pixel.SetPixelColor(3, black);
    pixel.SetPixelColor(4, black);
    pixel.SetPixelColor(5, yellow);
    pixel.Show();

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
    wifiManager.setConfigPortalTimeout(30);
    wifiManager.setConnectTimeout(30);
    if (!wifiManager.autoConnect("mimirAP"))
    {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        Serial.println("Going to sleep now");
        SLEEP(5);
    }
    wifi_signal = WiFi.RSSI();
    SetupTime();
};
void mimirOpen::WiFi_OFF()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
};

void mimirOpen::SLEEP(long interval)
{
    //CONFIG Sleep Pin
    Serial.println("Config Sleep Pin");
    //---SINGLE GPIO PIN---
    gpio_pullup_en(GPIO_NUM_26);
    gpio_pulldown_dis(GPIO_NUM_26);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, LOW);
    //---MULTI GPIO PIN---
    //****This will need pull up resistors on the other end of the buttons so they pull high, not low
    //GPIO Mask = 2^25+2^26+2^27 = 234881024 => 0xE000000
    //esp_sleep_enable_ext1_wakeup(GPIOBitMask, ESP_EXT1_WAKEUP_ANY_HIGH);

    if ((WiFi.status() == WL_CONNECTED))
    {
        Serial.println("Turning off Wifi Stuff...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    Serial.println("Turning off Bluetooth Stuff...");
    btStop();
    esp_bt_controller_disable();
    Serial.println("Turning off ADC Stuff...");
    adc_power_off();

    //CONFIG Sleep Timer
    Serial.println("Config Sleep Timer");                                            // Wake if GPIO is low
    long SleepTimer = (interval * 60 - ((CurrentMin % interval) * 60 + CurrentSec)); //Some ESP32 are too fast to maintain accurate time
    esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL);
    turnOFFPixels();

    Serial.println("Entering " + String(SleepTimer) + "-secs of sleep time");
    Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
    Serial.println("Starting deep-sleep...");

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
void mimirOpen::printBootReason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:
        Serial.println("Reset reason can not be determined");
        break;
    case ESP_RST_POWERON:
        Serial.println("Reset due to power-on event");
        break;
    case ESP_RST_EXT:
        Serial.println("Reset by external pin (not applicable for ESP32)");
        break;
    case ESP_RST_SW:
        Serial.println("Software reset via esp_restart");
        break;
    case ESP_RST_PANIC:
        Serial.println("Software reset due to exception/panic");
        break;
    case ESP_RST_INT_WDT:
        Serial.println("Reset (software or hardware) due to interrupt watchdog");
        break;
    case ESP_RST_TASK_WDT:
        Serial.println("Reset due to task watchdog");
        break;
    case ESP_RST_WDT:
        Serial.println("Reset due to other watchdogs");
        break;
    case ESP_RST_DEEPSLEEP:
        Serial.println("Reset after exiting deep sleep mode");
        break;
    case ESP_RST_BROWNOUT:
        Serial.println("Brownout reset (software or hardware)");
        break;
    case ESP_RST_SDIO:
        Serial.println("Reset over SDIO");
        break;
    }

    if (reset_reason == ESP_RST_DEEPSLEEP)
    {
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

        switch (wakeup_reason)
        {
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            Serial.println("In case of deep sleep: reset was not caused by exit from deep sleep");
            break;
        case ESP_SLEEP_WAKEUP_ALL:
            Serial.println("Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            Serial.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            Serial.println("Wakeup caused by ULP program");
            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            Serial.println("Wakeup caused by GPIO (light sleep only)");
            break;
        case ESP_SLEEP_WAKEUP_UART:
            Serial.println("Wakeup caused by UART (light sleep only)");
            break;
        }
    }
}

void mimirOpen::turnOFFPixels()
{
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, black);
    }
    pixel.Show();
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
        delay(_delay);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, green);
        pixel.Show();
        delay(_delay);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, blue);
        pixel.Show();
        delay(_delay);
        pixel.SetPixelColor(i, black);
        pixel.Show();
    }
}

void mimirOpen::pixelBootUp()
{
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        pixel.SetPixelColor(i, blue);
        pixel.Show();
        delay(100);
    }
    turnOFFPixels();
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