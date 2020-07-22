#include "Sensors.h"

Sensors::Sensors() : dht22(DHTPIN, DHTTYPE) {
#ifdef PLATFORM_ID
    sensorLog("app.Sensors");
#endif
}

void Sensors::setup(PersistentConfig *config) {
    setupResetReason();
    setupRTC();
    setupPM();
    setupAir();
    setupTempHum();
    setupCOSensor();
    setupEnergySensor();
    setupTraceHeater(config);
    setupDHT22();
}

bool Sensors::isRTCPresent() {
    uint32_t first = rtc.now().unixtime();
    delay(1000);
    uint32_t second = rtc.now().unixtime();

    return first != second;
}

float Sensors::readACCurrentValue() {
    float ACCurrtntValue = 0;
    int32_t peakVoltage = 0;
    float voltageVirtualValue = 0; // Vrms
    for (int i = 0; i < 1000; i++) {
        peakVoltage += analogRead(AC_PIN); // read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 1000;
    Serial.printf("ADC:%x\t\t", peakVoltage);
    voltageVirtualValue =
        peakVoltage * 0.707; // change the peak voltage to the Virtual Value of voltage

    /*The circuit is amplified by 2 times, so it is divided by 2.*/
    voltageVirtualValue = (voltageVirtualValue / 4096 * VREF) / 2;

    Log.info("voltageVirtualValue=%f", voltageVirtualValue);

    ACCurrtntValue = voltageVirtualValue * ACTectionRange;

    return ACCurrtntValue;
}

void Sensors::serialEvent1() {
    sensorLog.info("SerialEvent1!");
    Serial1.readBytesUntil('\n', serialData, SERIAL_DATA_SIZE);
    sensorLog.info("Serial Data received: %s\n", serialData);
    newSerialData = true;
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

void Sensors::setupCOSensor() {
    // Setup co sensor
    Serial1.begin(9600);
    Serial1.flush();
    delay(1000);
    Serial1.write('A'); // Set the running average so it uses 300 measurements
    delay(1000);
    Serial1.print(300);
    delay(500);
    Serial1.write('\r'); // Request a measurement
    delay(2500);
}

void Sensors::setupEnergySensor() {
    pinMode(ENERGY_SENSOR_PRESENT_PIN, INPUT_PULLUP);
    if (digitalRead(ENERGY_SENSOR_PRESENT_PIN) == ENERGY_SENSOR_DETECTED) {
        Log.info("Energy sensor present!");
    }
}

void Sensors::setupTraceHeater(PersistentConfig *config) {
    traceHeater.setTimeConst(config->data.boardTimeConstant);
    traceHeater.setIntTempFunct([]() {
        if (getInstance()->getTempHumPresent()) {
            return getInstance()->getSHT31()->readTemperature();
        }
        return INFINITY;
    });
    traceHeater.setExtTempFunct([]() {
        if (getInstance()->getDHT22() != NULL) {
            return getInstance()->getDHT22()->readTemperature();
        }
        return INFINITY;
    });
    traceHeater.begin();
}

void Sensors::setupDHT22() {
    dht22.begin();
    delay(4000); // Apparently the DHT22 takes a while to gather all readings
    if (isnan(dht22.readTemperature()) /* || isnan(dht22.readHumidity())*/) {
        dht22Setup = false;
    } else {
        dht22Setup = true;
    }
}

void Sensors::read(SensorPacket *packet, PersistentConfig *config) {
    readRTC(packet);
    readPMSensor(packet);
    readAirSensor(packet, config);
    readTemHumSensor(packet, config);
    readTraceHeater(packet, config);
    readCOSensor(packet);
    readResetReason(packet);
    readBatteryCharge(packet);
    readFreeMem(packet);
    readEnergySensor(packet, config);
    readDHT22(packet);
}

void Sensors::readPMSensor(SensorPacket *packet) {
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
            packet->internal_temperature = (int32_t)round(temp * 10);
            packet->has_internal_temperature = true;
        }
        float humidity = sht31.readHumidity();
        packet->internal_humidity = (uint32_t)round(humidity * 10);
        packet->has_internal_humidity = true;

        sensorLog.info("readTemHumSensor(): tempHum - temp=%ld, hum=%ld",
                       packet->internal_temperature, packet->internal_humidity);
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

void Sensors::readRTC(SensorPacket *packet) {
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

void Sensors::readCOSensor(SensorPacket *packet) {
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

void Sensors::readResetReason(SensorPacket *packet) {
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

void Sensors::readBatteryCharge(SensorPacket *packet) {
#ifdef PLATFORM_ID
#if PLATFORM_ID == PLATFORM_BORON
    if (DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_BATTERY_STATE) == BATTERY_STATE_DISCONNECTED ||
        DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_BATTERY_STATE) == BATTERY_STATE_UNKNOWN) {
        sensorLog.warn("The battery is either disconnected or in an unknown state.");
        packet->has_battery_charge = false;
    } else if (DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE) == POWER_SOURCE_BATTERY) {
        int32_t batteryCharge = (int32_t)roundf(System.batteryCharge());
        sensorLog.info("System.batteryCharge(): %ld%%", batteryCharge);
        packet->has_battery_charge = true;
        packet->battery_charge = batteryCharge;
    } else if (DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE) != POWER_SOURCE_BATTERY) {
        sensorLog.info("The sensor is plugged-in or connected via USB");
        packet->has_battery_charge = false;
    }
#endif
#endif
}

void Sensors::readFreeMem(SensorPacket *packet) {
#ifdef PLATFORM_ID
#if Wiring_WiFi
    uint32_t freeMem = System.freeMemory();
    packet->free_memory = freeMem;
    packet->has_free_memory = true;
#endif
#endif
}

void Sensors::readEnergySensor(SensorPacket *packet, PersistentConfig *config) {
    if (digitalRead(ENERGY_SENSOR_PRESENT_PIN) == ENERGY_SENSOR_DETECTED) {
        sensorLog.info("readEnergySensor(): Energy sensor detected.");
        float acCurrentValue = readACCurrentValue(); // read AC Current Value
        float heaterPF =
            ((float)config->data.heaterPowerFactor / 1000.0f); // Puts power factor into float form
        float measuredPower =
            acCurrentValue * config->data.countryVoltage * heaterPF; // Calculates real power
        sensorLog.info("heaterPowerFactor: pf=%f", heaterPF);
        sensorLog.info("countryVoltage: int voltage=%ld", config->data.countryVoltage);
        sensorLog.info("readEnergySensor(): float current=%f", acCurrentValue);
        packet->has_current = true;
        packet->current = (int32_t)(acCurrentValue * 1000);
        sensorLog.info("readEnergySensor(): float power=%f", measuredPower);
        packet->has_power = true;
        packet->power = (int32_t)measuredPower;
    }
}

void Sensors::readDHT22(SensorPacket *packet) {
    if (dht22Setup) {
        packet->has_temperature = true;
        packet->has_humidity = true;
        packet->temperature = dht22.readTemperature();
        packet->humidity = dht22.readHumidity();
    } else {
        sensorLog.warn("DHT22 is not set up");
        packet->has_temperature = false;
        packet->has_humidity = false;
    }
};