#include "Sensors.h"

Sensors::Sensors(PersistentConfig *config)
    : traceHeater(config->data.boardTimeConstant, []() {
          if (tempHumPresent) {
              return sht31.readTemperature();
          }
          return INFINITY;
      }) {
#ifdef PLATFORM_ID
    sensorLog("app.Sensors");
#endif
}

void Sensors::setup() {
    setupResetReason();
    setupPM();
    setupAir();
    setupTempHum();
    traceHeater.begin();
}

bool Sensors::isRTCPresent() {
    uint32_t first = rtc.now().unixtime();
    delay(1000);
    uint32_t second = rtc.now().unixtime();

    return first != second;
}

void Sensors::setupRTC() {

    // Make sure RTC is really working
    if (!rtc.begin() || !isRTCPresent()) {
        sensorLog.error("Could not start RTC!");
        rtcPresent = false;
    } else {
        uint32_t now = rtc.now().unixtime();

        // Now make sure the time is somewhat right
        if (now < 1560000000) {
            sensorLog.warn("RTC is present, but has not been set.");
            rtcSet = false;
        }
    }
}

void Sensors::setupPM() {
    if (!pmSensor.begin()) {
        sensorLog.error("Could not start PM sensor!");
        pmSensorSetup = false;
    }
}

void Sensors::setupAir() {
    if (!airSensor.begin()) {
        sensorLog.error("Could not start CO2 sensor!");
        airSensorSetup = false;
    }
}

void Sensors::setupTempHum() {
    tempHumPresent = true;
    if (!sht31.begin(TEMP_HUM_I2C_ADDR)) {
        Serial.println("Couldn't find SHT31 (temp humidity)!");
        tempHumPresent = false;
    }
    Serial.printf("sht31 status = %d\n", sht31.readStatus());
}

void Sensors::setupResetReason() {
#ifdef PLATFORM_ID
    resetReason = System.resetReason();
#endif
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
        sensorLog.info("readAirSensor(): CO2 - CO2=%ld, temp=%ld, hum=%ld", packet->co2,
                       packet->temperature, packet->humidity);
    } else {
        sensorLog.error("can't read CO2");
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

        sensorLog.info("readTemHumSensor(): tempHum - temp=%ld, hum=%ld", packet->temperature,
                       packet->humidity);
    } else {
        sht31.begin(TEMP_HUM_I2C_ADDR);
    }
}

void Sensors::readTraceHeater(SensorPacket *packet, PersistentConfig *config) {
    if (config->data.traceHeaterEnabled && traceHeater.hasNewTemperatureData()) {
        packet->temperature = (int32_t)round(traceHeater.getTemperatureData() * 10);
        packet->has_temperature = true;
    }
}

void Sensors::readRTC(SensorPacket *packet, PersistentConfig *config) {
    if (rtcPresent) {
        sensorLog.info("readRTC(): RTC is present");
        DateTime now = rtc.now();
        packet->timestamp = now.unixtime();
    } else {
        sensorLog.error("readRTC(): RTC is NOT present!");
        packet->timestamp = Time.now();
    }
    if (rtcSet) {
        sensorLog.info("readRTC(): RTC is set");
    } else {
        sensorLog.error("readRTC(): RTC is NOT set!");
    }
    if (rtcPresent) {
        packet->rtc_temperature = rtc.getTemperature();
        packet->has_rtc_temperature = true;
    }
}

void Sensors::readCOSensor(SensorPacket *packet, PersistentConfig *config) {
    if (newSerialData) {
        uint32_t sensorNum;
        float conc;
        float temp;
        float rh;
        uint32_t conc_c;
        uint32_t temp_c;
        uint32_t rh_c;
        uint32_t days;
        uint32_t hours;
        uint32_t minutes;
        uint32_t seconds;
        int result =
            sscanf(serialData, "%lu, %f, %f, %f, %lu, %lu, %lu, %lu, %lu, %lu, %lu", &sensorNum,
                   &conc, &temp, &rh, &conc_c, &temp_c, &rh_c, &days, &hours, &minutes, &seconds);
        if (result == 11) {
            packet->has_co = true;
            packet->co = (uint32_t)(conc * 100);
            sensorLog.info("readCOSensor(): Result of %d, Reading co value of %f, transmitting %ld",
                           result, conc, packet->co);
        } else {
            sensorLog.error("readCOSensor(): Could not interpret co value");
        }
        newSerialData = false;
        Serial1.write('\r'); // Ask for another measurement from the CO sensor
    }
}

void Sensors::readResetReason(SensorPacket *packet, PersistentConfig *config) {
#ifdef PLATFORM_ID
    if (resetReason != RESET_REASON_NONE) {
        packet->reset_reason = resetReason;
        packet->has_reset_reason = true;

        if (resetReason == RESET_REASON_PANIC) {
            uint32_t resetReasondata = System.resetReasonData();
            packet->reset_reason_data = resetReasondata;
            packet->has_reset_reason_data = true;
        }

        // Make sure to read reset reason only once
        resetReason = RESET_REASON_NONE;
    }
#endif
}