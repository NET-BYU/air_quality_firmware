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
    void setup();

    void read(SensorPacket *packet, PersistentConfig *config);

  private:
    void readPMSensor(SensorPacket *packet, PersistentConfig *config);
    void readAirSensor(SensorPacket *packet, PersistentConfig *config);
    void readTemHumSensor(SensorPacket *packet, PersistentConfig *config);

    void readRTC(SensorPacket *packet, PersistentConfig *config);

    // PM Sensor
    SPS30 pmSensor;
    float pmMeasurement[4]; // PM 1, 2.5, 4, 10
    bool pmSensorSetup = true;

    // CO2 + Temp + Humidity Sensor
    SCD30 airSensor;
    bool airSensorSetup = true;

    Adafruit_SHT31 sht31; // = Adafruit_SHT31();
    float tempMeasurement;
    float humidityMeasurement;
    bool tempHumPresent = true;

    // RTC
    RTC_DS3231 rtc;
    bool rtcPresent = true;
    bool rtcSet = true;

    // TraceHeater traceHeater(config.data.boardTimeConstant, []() {
    //     if (tempHumPresent) {
    //         return sht31.readTemperature();
    //     }
    //     return INFINITY;
    // });
};

#endif