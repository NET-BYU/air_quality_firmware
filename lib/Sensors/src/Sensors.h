#ifndef SENSORS_H_
#define SENSORS_H_

#include "PersistentConfig.h"
#include "RTClibrary.h"
#include "SPS30.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "TraceHeater.h"
#include "adafruit-sht31.h"
#include "sensor_packet.pb.h"

#define TEMP_HUM_I2C_ADDR 0x44 // Set to 0x45 for alternate i2c addr

class Sensors {
  public:
    Sensors(PersistentConfig *config);
    void setup();

    void read(SensorPacket *packet, PersistentConfig *config);

  private:
    // Auxiliary functs for setup/read process
    bool isRTCPresent();

    // Setup for individual sensors
    void setupRTC();
    void setupPM();
    void setupAir();
    void setupTempHum();
    void setupResetReason();

    // Read for individual sensors
    void readRTC(SensorPacket *packet, PersistentConfig *config);
    void readPMSensor(SensorPacket *packet, PersistentConfig *config);
    void readAirSensor(SensorPacket *packet, PersistentConfig *config);
    void readTemHumSensor(SensorPacket *packet, PersistentConfig *config);
    void readCOSensor(SensorPacket *packet, PersistentConfig *config);
    void readTraceHeater(SensorPacket *packet, PersistentConfig *config);
    void readResetReason(SensorPacket *packet, PersistentConfig *config);

    // Setters
    void setNewSerialData(bool newSerialData) { this->newSerialData = newSerialData; };

    // PM Sensor
    SPS30 pmSensor;
    float pmMeasurement[4]; // PM 1, 2.5, 4, 10
    bool pmSensorSetup = true;

    // CO2 + Temp + Humidity Sensor
    SCD30 airSensor;
    bool airSensorSetup = true;

    static Adafruit_SHT31 sht31; // = Adafruit_SHT31();
    float tempMeasurement;
    float humidityMeasurement;
    static bool tempHumPresent; // = true;

    // RTC
    RTC_DS3231 rtc;
    bool rtcPresent = true;
    bool rtcSet = true;

    uint32_t timeConstant;
    TraceHeater traceHeater;

    // Serial device
#define SERIAL_DATA_SIZE 200
    char serialData[SERIAL_DATA_SIZE];
    bool newSerialData = false;

    // Global variables to keep track of state
    int resetReason = RESET_REASON_NONE;

#ifdef PLATFORM_ID
    Logger sensorLog;
#endif
};

#endif