#ifndef SENSORS_H_
#define SENSORS_H_

#include "SPS30.h"

class Sensors {
  public:
    void setup();

    void readSensors();

  private:
    // PM Sensor
    SPS30 pmSensor;
    float pmMeasurement[4]; // PM 1, 2.5, 4, 10
    bool pmSensorSetup = true;
};

#endif