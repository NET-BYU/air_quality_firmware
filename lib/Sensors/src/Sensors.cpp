#include "Sensors.h"

#define DHT22_MAX_READ_ATTEMPTS 20

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
    setupDHT22();
    setupTraceHeater(config);
}

bool Sensors::isRTCPresent() {
    uint32_t first = rtc.now().unixtime();
    delay(1000);
    uint32_t second = rtc.now().unixtime();

    return first != second;
}

void Sensors::isCOSetup() {
    if (!coSetup) {
        Serial1.write('c'); // Request a measurement
        Serial1.flush();
        sensorLog.info("Writing: c");
        delay(2500);
        coSetup = true;
    }
}

void Sensors::addNullTermSerialData(size_t serialDataLength) {
    serialData[serialDataLength] = '\0';
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
    sensorLog.info("ADC:%x\t\t", peakVoltage);
    voltageVirtualValue =
        peakVoltage * 0.707; // change the peak voltage to the Virtual Value of voltage

    /*The circuit is amplified by 2 times, so it is divided by 2.*/
    voltageVirtualValue = (voltageVirtualValue / 4096 * VREF) / 2;

    Log.info("voltageVirtualValue=%f", voltageVirtualValue);

    ACCurrtntValue = voltageVirtualValue * ACTectionRange;

    return ACCurrtntValue;
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
        sensorLog.error("Couldn't find SHT31 (temp humidity)!");
        tempHumPresent = false;
    }
    sensorLog.info("sht31 status = %d\n", sht31.readStatus());
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
    delay(5000);
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
        float temp;
        for (uint8_t i = 0; i < DHT22_MAX_READ_ATTEMPTS; i++) {
            if (getInstance()->getDHT22() != NULL) {
                temp = getInstance()->getDHT22()->readTemperature(false, true);
                if (!isnan(temp)) {
                    return temp;
                }
            }
        }
        return INFINITY;
    });
    traceHeater.setUnixTimeFunct([]() {
        uint32_t time = 0;
        if (getInstance()->getRTCPresent()) {
            time = getInstance()->rtc.now().unixtime();
        }
        return time;
    });
    traceHeater.begin();
}

void Sensors::setupDHT22() {
    dht22.begin();
    delay(4000); // Apparently the DHT22 takes a while to gather all readings
    // if (isnan(dht22.readTemperature()) || isnan(dht22.readHumidity())) {
    //     dht22Setup = false;
    // } else {
    //     dht22Setup = true;
    // }
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

        float temp = airSensor.getTemperature();
        packet->temperature = (int32_t)round(temp * 10);
        packet->has_temperature = true;

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
        float temp = sht31.readTemperature();
        packet->internal_temperature = (int32_t)round(temp * 10);
        packet->has_internal_temperature = true;
        float humidity = sht31.readHumidity();
        packet->internal_humidity = (uint32_t)round(humidity * 10);
        packet->has_internal_humidity = true;

        sensorLog.info("readTemHumSensor(): tempHum - temp=%ld, hum=%ld",
                       packet->internal_temperature, packet->internal_humidity);
    } else {
        sensorLog.error("can't read from SHT31");
        sht31.begin(TEMP_HUM_I2C_ADDR);
    }
}

void Sensors::readTraceHeater(SensorPacket *packet, PersistentConfig *config) {
    if (config->data.traceHeaterEnabled) {
        // TODO: Add some logic ignoring cases where the estimated temperature is not accurate
        packet->estimated_temperature = (int32_t)round(traceHeater.getEstimatedTemperature() * 10);
        packet->has_estimated_temperature = true;
        packet->trace_heater_state = traceHeater.state;
        packet->has_trace_heater_state = true;
        packet->trace_heater_th = (int32_t)round(traceHeater.T_H * 10);
        packet->has_trace_heater_th = true;
        packet->trace_heater_tc = (int32_t)round(traceHeater.T_C * 10);
        packet->has_trace_heater_tc = true;
        packet->trace_heater_current_temp = (int32_t)round(traceHeater.internal_temperature * 10);
        packet->has_trace_heater_current_temp = true;
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
        unsigned int offset = 0;
        int32_t conc = INT32_MAX;
        int32_t temp = INT32_MAX;
        int32_t rh = INT32_MAX;
        int32_t conc_c = INT32_MAX;
        int32_t temp_c = INT32_MAX;
        int32_t rh_c = INT32_MAX;
        int32_t days = INT32_MAX;
        int32_t hours = INT32_MAX;
        int32_t minutes = INT32_MAX;
        int32_t seconds = INT32_MAX;
        for (offset = 0;
             offset <= SERIAL_DATA_SIZE && serialData[offset] != '\0' && serialData[offset] != ',';
             offset++) {
        }
        if (serialData[offset] != ',') {
            return;
        }
        sensorLog.info("Serial Data received: %s\n", serialData + offset);
        int result =
            sscanf(serialData + offset, ", %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld", &conc,
                   &temp, &rh, &conc_c, &temp_c, &rh_c, &days, &hours, &minutes, &seconds);
        if (result == 10) {
            Log.info("readCOSensor(): Result of %d, Reading co value of %ld", result, conc);
            sensorLog.info("Data: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", conc, temp, rh, conc_c,
                           temp_c, rh_c, days, hours, minutes, seconds);
            packet->co = conc;
            packet->has_co = true;
        } else {
            sensorLog.info("Data: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", conc, temp, rh, conc_c,
                           temp_c, rh_c, days, hours, minutes, seconds);
            Log.info("readCOSensor(): Could not interpret co value. Number of elements scanned: %d",
                     result);
        }
        newSerialData = false;
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
    float temp;
    float hum;
    bool temp_valid = false;
    bool hum_valid = false;
    for (uint8_t i = 0; i < DHT22_MAX_READ_ATTEMPTS && !(temp_valid && hum_valid); i++) {
        if (!temp_valid) {
            temp = dht22.readTemperature(false, true);
            if (!isnan(temp)) {
                packet->has_temperature = true;
                packet->temperature = (int32_t)round(temp * 10);
                temp_valid = true;
            }
        }
        if (!hum_valid) {
            hum = dht22.readHumidity(true);
            if (!isnan(hum)) {
                packet->has_humidity = true;
                packet->humidity = (uint32_t)round(hum * 10);
                hum_valid = true;
            }
        }
    }
}