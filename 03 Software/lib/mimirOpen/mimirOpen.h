#ifndef mimirOpen_h
#define mimirOpen_h

#include "Arduino.h"
#include <Wire.h>
#include <SD.h>
#include "time.h"
#include <WiFiManager.h>

#define addrSHT31D 0x45
#define addrVEML6030 0x48
#define addrCCS811 0x5A
#define addrBME680 0x76
#define addrCompas 0X0C

enum SYS_STATUS
{
    ERROR_L = -3,
    ERROR_R = -2,
    ERROR_W = -1,
    UNMOUNTED = 0,
    OKAY = 1
};

struct config
{
    String email;
    String serialNumber;
    String deviceName;
    String userId;
    String deviceId;
};

struct authType
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

struct DataPackage
{
    authType auth;
    systems status;
    envData data;
};

struct AuthPackage
{
    String email;
    String serialNumber;
    String macAddress;
};

class mimirOpen
{

public:
    mimirOpen(int baudRate = 115200);

    //Init
    void initPixels(int brightness = 64);
    void initSensors();
    void initMicroSD(String filename = "/0000-00-00.txt");
    void initWIFI(config config);
    void forceWIFI(config config);
    config initSPIFFS();

    //Main
    void saveToSPIFFS(config data);
    void sendAuth(String address, AuthPackage auth, config config);
    void sendData(String address, DataPackage data);
    envData readSensors();
    void printSensors(envData data);
    void logData(envData data, String filename = "/0000-00-00.txt");

    //Helping
    void printBootReason();
    String stringData(envData data, systems sys);
    systems getStatus();
    void WiFi_ON();
    void WiFi_OFF();
    void SLEEP(long interval = 15);

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

    String TimeStr, DateStr, ErrorMessage; // strings to hold time and date
    const char *TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

    int StartTime = 0, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0;

    int wifi_signal;
    int batteryPercent;

    SYS_STATUS STATUS_BATTERY = UNMOUNTED;
    SYS_STATUS STATUS_WIFI = UNMOUNTED;
    SYS_STATUS STATUS_SD = UNMOUNTED;
    SYS_STATUS STATUS_SERVER = UNMOUNTED;
    SYS_STATUS STATUS_BME680 = UNMOUNTED;
    SYS_STATUS STATUS_COMPAS = UNMOUNTED;
    SYS_STATUS STATUS_SHT31 = UNMOUNTED;
    SYS_STATUS STATUS_VEML6030 = UNMOUNTED;

    //Helper
    int getBatteryPercent();
    float calcAltitude(float pressure, float temperature);
    float averageValue(float values[]);
    String packageAuthJSON(AuthPackage auth);
    String packageJSON(DataPackage data);
    String header();
    bool SetupTime();
    bool UpdateLocalTime();
    static void WiFiCallback(WiFiManager *myWiFiManager);
    void writeFile(fs::FS &fs, const char *path, const char *message);
    void appendFile(fs::FS &fs, const char *path, const char *message);
};
#endif