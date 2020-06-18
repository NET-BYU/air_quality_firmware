#include "Sensors.h"

void Sensors::setup() {
    pmSensor.begin();

    if (!airSensor.begin()) {
        Log.error("Could not start CO2 sensor!");
        airSensorSetup = false;
    }
}

void Sensors::read(SensorPacket *packet, PersistentConfig *config) {
    readPMSensor(packet, config);
    readAirSensor(packet, config);
}

void Sensors::readPMSensor(SensorPacket *packet, PersistentConfig *config) {
    if (pmSensor.dataAvailable()) {
        pmSensor.getMass(pmMeasurement);

        for (size_t i = 0; i < sizeof(pmMeasurement) / sizeof(pmMeasurement[0]); i++) {
            if (pmMeasurement[i] > INT32_MAX) {
                pmMeasurement[i] = INT32_MAX;
            }
        }

        packet->pm1 = pmMeasurement[0];
        packet->has_pm1 = true;
        packet->pm2_5 = pmMeasurement[1];
        packet->has_pm2_5 = true;
        packet->pm4 = pmMeasurement[2];
        packet->has_pm4 = true;
        packet->pm10 = pmMeasurement[3];
        packet->has_pm10 = true;
    } else {
        // There should always be data available so begin measuring again
        pmSensor.begin();
    }
}

void Sensors::readAirSensor(SensorPacket *packet, PersistentConfig *config) {
    if (airSensor.dataAvailable()) {
        uint32_t co2 = airSensor.getCO2();
        packet->co2 = co2;
        packet->has_co2 = true;

        if (!config->data.traceHeaterEnabled) {
            float temp = airSensor.getTemperature();
            packet->temperature = (int32_t)round(temp * 10);
            packet->has_temperature = true;
        }

        float humidity = airSensor.getHumidity();
        packet->humidity = (uint32_t)round(humidity * 10);
        packet->has_humidity = true;
        Log.info("readAirSensor(): CO2 - CO2=%ld, temp=%ld, hum=%ld", packet->co2,
                 packet->temperature, packet->humidity);
    } else {
        Log.error("can't read CO2");
        // There should always be data available so begin measuring again
        airSensor.begin();
    }
}

void Sensors::readTemHumSensor(SensorPacket *packet, PersistentConfig *config) {
    if (tempHumPresent) {
        if (!config->data.traceHeaterEnabled) {
            float temp = sht31.readTemperature();
            packet->temperature = (int32_t)round(temp * 10);
            packet->has_temperature = true;
        }
        float humidity = sht31.readHumidity();
        packet->humidity = (uint32_t)round(humidity * 10);
        packet->has_humidity = true;

        Log.info("readTemHumSensor(): tempHum - temp=%ld, hum=%ld", packet->temperature,
                 packet->humidity);
    } else {
        sht31.begin(TEMP_HUM_I2C_ADDR);
    }
}

// void Sensors::readTraceHeater(SensorPacket *packet, PersistentConfig *config) {
//     if (config->data.traceHeaterEnabled && traceHeater.hasNewTemperatureData()) {
//         packet->temperature = (int32_t)round(traceHeater.getTemperatureData() * 10);
//         packet->has_temperature = true;
//     }
// }

void Sensors::readRTC(SensorPacket *packet, PersistentConfig *config) {
    if (rtcPresent) {
        Log.info("readRTC(): RTC is present");
        DateTime now = rtc.now();
        packet->timestamp = now.unixtime();
    } else {
        Log.error("readRTC(): RTC is NOT present!");
        packet->timestamp = Time.now();
    }
    if (rtcSet) {
        Log.info("readRTC(): RTC is set");
    } else {
        Log.error("readRTC(): RTC is NOT set!");
    }
    if (rtcPresent) {
        packet->rtc_temperature = rtc.getTemperature();
        packet->has_rtc_temperature = true;
    }
}