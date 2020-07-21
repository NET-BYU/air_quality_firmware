#ifndef SENSORS_H_
#define SENSORS_H_

#include "DHT.h"
#include "DiagnosticsHelperRK.h"
#include "PersistentConfig.h"
#include "RTClibrary.h"
#include "SPS30.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "TraceHeater.h"
#include "adafruit-sht31.h"
#include "sensor_packet.pb.h"

#define TEMP_HUM_I2C_ADDR 0x44 // Set to 0x45 for alternate i2c addr

class Sensors {

    static Sensors *instance;

  public:
    static Sensors *getInstance() {
        if (!instance) {
            instance = new Sensors;
        }
        return instance;
    }

    void setup(PersistentConfig *config);
    void read(SensorPacket *packet, PersistentConfig *config);

    // Getters
    bool getRTCPresent() { return rtcPresent; };
    bool getRTCSet() { return rtcSet; };
    bool getPmSensorSetup() { return pmSensorSetup; };
    bool getAirSensorSetup() { return airSensorSetup; };

    bool isRTCPresent();

    bool getTempHumPresent() { return tempHumPresent; };
    Adafruit_SHT31 *getSHT31() { return &sht31; };

    // Setters
    void setRTCSet(bool isSetup) { rtcSet = isSetup; };
    void setRTCPresent(bool isSetup) { rtcPresent = isSetup; };

    TraceHeater traceHeater;

    // CO2 + Temp + Humidity Sensor
    SCD30 airSensor;

    // RTC
    RTC_DS3231 rtc;

  private:
    Sensors();

    // Auxiliary functs for setup/read process
    float readACCurrentValue();
    void serialEvent1();

    // Setup for individual sensors
    void setupRTC();
    void setupPM();
    void setupAir();
    void setupTempHum();
    void setupResetReason();
    void setupCOSensor();
    void setupEnergySensor();
    void setupTraceHeater(PersistentConfig *config);
    // void setupDHT22();

    // Read for individual sensors
    void readRTC(SensorPacket *packet);
    void readPMSensor(SensorPacket *packet);
    void readAirSensor(SensorPacket *packet, PersistentConfig *config);
    void readTemHumSensor(SensorPacket *packet, PersistentConfig *config);
    void readCOSensor(SensorPacket *packet);
    void readTraceHeater(SensorPacket *packet, PersistentConfig *config);
    void readResetReason(SensorPacket *packet);
    void readBatteryCharge(SensorPacket *packet);
    void readFreeMem(SensorPacket *packet);
    void readEnergySensor(SensorPacket *packet, PersistentConfig *config);
    void readDHT22(SensorPacket *packet);

    // Setters
    void setNewSerialData(bool newSerialData) { this->newSerialData = newSerialData; };

    // PM Sensor
    SPS30 pmSensor;
    float pmMeasurement[4]; // PM 1, 2.5, 4, 10
    bool pmSensorSetup = true;

    bool airSensorSetup = true;

    bool rtcPresent = true;
    bool rtcSet = true;

    Adafruit_SHT31 sht31;
    float tempMeasurement;
    float humidityMeasurement;
    bool tempHumPresent;

    // Serial device
#define SERIAL_DATA_SIZE 200
    char serialData[SERIAL_DATA_SIZE];
    bool newSerialData = false;

    // Global variables to keep track of state
    int resetReason = RESET_REASON_NONE;

    // Energy Sensor Stuff
#define ENERGY_SENSOR_PRESENT_PIN D6
#define AC_PIN A0         // set arduino signal read pin
#define ACTectionRange 20 // set Non-invasive AC Current Sensor tection range (5A,10A,20A)
#define VREF 3.3          // VREF: Analog reference
// #define UPPER_VOLTAGE_THRESHOLD 3000    // Some value less than ADC_MAX used for detecting a pull
// down resistor
#define ENERGY_SENSOR_DETECTED                                                                     \
    LOW // What the ENERGY_SENSOR_PRESENT_PIN should read if there is an energy sensor

// DHT22
#define DHTPIN D2
#define DHTTYPE DHT22
    bool dht22Setup = true;
    DHT dht22;

#ifdef PLATFORM_ID
    Logger sensorLog;
#endif
};

#endif