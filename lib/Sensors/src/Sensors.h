#ifndef SENSORS_H_
#define SENSORS_H_

#include "PersistentConfig.h"
#include "SPS30.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "sensor_packet.pb.h"

#define TEMP_HUM_I2C_ADDR 0x44 // Set to 0x45 for alternate i2c addr

class Sensors {
  public:
    void setup();

    void read(SensorPacket *packet, PersistentConfig *config);

  private:
    void readPMSensor(SensorPacket *packet, PersistentConfig *config);
    void readAirSensor(SensorPacket *packet, PersistentConfig *config);

    // PM Sensor
    SPS30 pmSensor;
    float pmMeasurement[4]; // PM 1, 2.5, 4, 10
    bool pmSensorSetup = true;

    // CO2 + Temp + Humidity Sensor
    SCD30 airSensor;
    bool airSensorSetup = true;
};

#endif