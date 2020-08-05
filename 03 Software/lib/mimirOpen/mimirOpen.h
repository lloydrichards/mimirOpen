#ifndef mimirOpen_h
#define mimirOpen_h

#include "Arduino.h"
#include <Wire.h>
#include <SD.h>
#include "time.h"
#include <WiFiManager.h>

#define addrSHT31D_L 0x44
#define addrSHT31D_H 0x45
#define addrVEML6030 0x48
#define addrVEML6075 0x10
#define addrCCS811 0x5A
#define addrbmp280 0x76
#define addrCompass 0X0C

enum SYS_STATUS
{
    ERROR_L = -3,
    ERROR_R =-2,
    ERROR_W =-1,
    UNMOUNTED =0,
    OKAY =1
};

struct config
{
    String userName;
    String password;
    String deviceName;
    String userId;
    String deviceId;
};

struct auth
{
    String userId;
    String deviceId;
    String macAddress;
};

struct systems
{
    SYS_STATUS battery;
    int batteryPercent;
    SYS_STATUS wifi;
    SYS_STATUS sd;
    SYS_STATUS server;
    SYS_STATUS BME680;
    SYS_STATUS COMPAS;
    SYS_STATUS SHT31;
    SYS_STATUS VEML6030;
};

struct envData
{
    float temperature;
    float humidity;
    float pressure;
    float altitude;
    float luminance;
    float iaq;
    float eVOC;
    float eCO2;
    int bearing;
};

class mimirOpen
{

public:
    mimirOpen(int baudRate = 115200);

    //Init
    void initPixels(int brightness = 64);
    void initSensors();
    void initMicroSD();
    void initWIFI(config config);
    config initSPIFFS();

    //Main
    void saveToSPIFFS(config data);
    void sendData(String address, envData data, systems SYS_STATUS, auth user);
    envData readSensors();
    void printSensors(envData data);
    void logData(envData data, systems SYS_STATUS);

    //Helping
    String stringData(envData data, systems SYS_STATUS);
    void WiFi_ON();
    void WiFi_OFF();
    void SLEEP();

    //Testing
    void testHTTPRequest(String address);
    void i2cScanner();
    void testPixels(int repeat = 5, int _delay = 100);

private:
    String _IP_ADDRESS;
    char _USER[40];
    char _USER_ID[40];
    char _DEVICE_ID[40];
    char MIMIR_VERSION[10] = "P2005.2";
    char filename[16] = "/0000-00-00.txt";

    String TimeStr, DateStr, ErrorMessage; // strings to hold time and date
    const char *TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

    int StartTime = 0, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0;
    long SleepDuration = 15; //minutes of the hour. eg 15 would wake up at XX:00, XX:15, XX:30 and XX:45

    int wifi_signal;
    int batteryPercent;

    float temp1;
    float temp2;
    float temp3;
    float alt;
    float hum1;
    float hum2;
    float pres;
    float lux;
    float uvA;
    float uvB;
    float uvIndex;
    float eCO2;
    float tVOC;
    uint16_t ccs811ERROR;
    int16_t compassX;
    int16_t compassY;
    int16_t compassZ;
    int bearing;

    double avgTemp;
    int avgHum;

    float averageValue(float values[]);
    String packageJSON(envData data, systems SYS_STATUS, auth user);
    String header();
    void createFileName(char date[]);
    bool SetupTime();
    bool UpdateLocalTime();
    static void WiFiCallback(WiFiManager *myWiFiManager);
    void writeFile(fs::FS &fs, const char *path, const char *message);
    void appendFile(fs::FS &fs, const char *path, const char *message);
};
#endif