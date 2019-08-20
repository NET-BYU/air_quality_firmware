#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "SimpleAckTracker.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"
#include "ArduinoJson.h"

#define READ_PERIOD_MS 60000
#define UPLOAD_PERIOD_MS 1000
#define UPLOAD_BATCH_SIZE 1
#define PRINT_SYS_INFO_MS 10000

#define MAX_PUB_SIZE 600 // It is really 622



#define LED1 D6
#define LED2 D7
#define LED3 D8

// Write data points to SD card and keep track of what has been ackowledged
SimpleAckTracker tracker;
uint8_t pendingPublishes = 0;

// PM Sensor
SPS30 pmSensor;
float pmMeasurement[4];

// CO2 + Temp + Humidity Sensor
SCD30 airSensor;

// RTC
RTC_DS3231 rtc;
bool rtcPresent = true;

// Energy Meter Data
#define ENERGY_METER_DATA_SIZE 200
char energyMeterData[ENERGY_METER_DATA_SIZE];
bool newEnergyMeterData = false;

// Counters
int sequence = 0;

// Global variables to keep track of state
bool saveDataSucess = true;
int resetReason = RESET_REASON_NONE;

// Publishing information
particle::Future<bool> currentPublish;
bool currentlyPublishing = false;

// LEDs
auto bootLED = JLed(LED1);
auto readLED = JLed(LED2);
auto publishLED = JLed(LED3);

// Timers
bool readDataFlag = true;
Timer readTimer(READ_PERIOD_MS, []() { readDataFlag = true; });
bool uploadFlag = true;
Timer uploadTimer(UPLOAD_PERIOD_MS, []() { uploadFlag = true; });

// Remote reset variables
#define DELAY_BEFORE_REBOOT 2000
unsigned long rebootSync = millis();
bool resetFlag = false;

// Print system information variables
bool enablePrintSystemInfo = false;
bool printSystemInfoFlag = true;
Timer printSystemInfoTimer(PRINT_SYS_INFO_MS, []() { printSystemInfoFlag = true; });

// Logging
Logger encodeLog("app.enocde");
Logger serialLog("app.serial");
SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO},
                                             {"app.enocde", LOG_LEVEL_INFO}});

// Particle system stuff
SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

void setup()
{
    bool success = true;
    resetReason = System.resetReason();

    // Set up cloud functions
    Particle.function("reset", cloudReset);

    // Debugging port
    Serial.begin(9600);

    // Energy meter port
    Serial1.begin(115200);

    delay(5000);

    if (!rtc.begin())
    {
        Log.error("Could not start RTC!");
        success = false;
        rtcPresent = false;
    }

    // Make sure RTC is really working
    uint32_t first = rtc.now().unixtime();
    delay(1000);
    uint32_t second = rtc.now().unixtime();
    if (first == second)
    {
        Log.error("Could not start RTC!");
        success = false;
        rtcPresent = false;
    }

    if (!pmSensor.begin())
    {
        Log.error("Could not start PM sensor!");
        success = false;
    }

    if (!airSensor.begin())
    {
        Log.error("Could not start CO2 sensor!");
        success = false;
    }

    // Set up LEDs
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);

    if (!success)
    {
        bootLED.Blink(500, 500).Forever();
    }
    else
    {
        bootLED.Blink(1000, 1000);
    }

    // Start timers
    readTimer.start();
    uploadTimer.start();

    if (enablePrintSystemInfo)
    {
        printSystemInfoTimer.start();
    }
}

void loop()
{
    // Read sensor task
    if (readDataFlag)
    {
        readLED.On().Update();
        Log.info("Reading sensors...");
        SensorPacket packet = SensorPacket_init_zero;
        readSensors(&packet);
        printPacket(&packet);

        Log.info("Putting data into a protobuf...");
        uint8_t data[SensorPacket_size];
        uint8_t length;

        if (packMeasurement(&packet, SensorPacket_size, data, &length))
        {
            Log.info("Adding data to tracker (%d)...", length);
            tracker.add(packet.timestamp, length, data);
            if (true)
            {
                saveDataSucess = true;
                readLED.Off().Update();
            }
            else
            {
                saveDataSucess = false;
                readLED.Blink(250, 250).Forever();
            }
        }
        else
        {
            saveDataSucess = false;
            readLED.Blink(250, 250).Forever();
        }

        sequence++;
        readDataFlag = false;
    }

    // Check on message that is currently being published
    if (currentlyPublishing && currentPublish.isDone())
    {
        if (currentPublish.isSucceeded())
        {
            Log.info("Publication was successful!");
            for (uint8_t i = 0; i < pendingPublishes; i++)
            {
                tracker.confirmNext();
            }
            Log.info("Unconfirmed count: %ld", tracker.unconfirmedCount());
            publishLED.Off().Update();
        }
        else
        {
            Log.warn("Publication was NOT successful!");
            publishLED.Blink(250, 250).Forever();
        }

        pendingPublishes = 0;
        currentlyPublishing = false;
    }

    // Upload data
    if (uploadFlag)
    {
        Log.trace("Trying to upload data... (%d, %d, %d)", !currentlyPublishing, Particle.connected(), tracker.unconfirmedCount() >= UPLOAD_BATCH_SIZE);
        if (!currentlyPublishing && Particle.connected() && tracker.unconfirmedCount() >= UPLOAD_BATCH_SIZE)
        {
            uint32_t maxLength = MAX_PUB_SIZE - (MAX_PUB_SIZE / 4); // TODO: Should do the ceiling of the division just to be safe
            uint8_t data[maxLength];
            uint32_t dataLength;
            getMeasurements(data, maxLength, dataLength, pendingPublishes);

            uint8_t encodedData[MAX_PUB_SIZE];
            uint32_t encodedDataLength;
            encodeMeasurements(data, dataLength, encodedData, &encodedDataLength);

            Log.info("Unconfirmed count: %ld", tracker.unconfirmedCount());
            Log.info("Publishing data: " + String((char *)encodedData));
            currentPublish = Particle.publish("mn/d", (char *)encodedData, 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
            publishLED.On().Update();
        }

        uploadFlag = false;
    }

    //  Remote Reset Function
    if (resetFlag && (millis() - rebootSync >=  DELAY_BEFORE_REBOOT)) {
        Serial.println("Rebooting...");
        Serial1.printf("reset");
        Serial1.flush();
        System.reset();
    }

    if (printSystemInfoFlag)
    {
        printSystemInfo();
        printSystemInfoFlag = false;
    }

    // Update LEDs
    bootLED.Update();
    readLED.Update();
    publishLED.Update();
}

void serialEvent1()
{
    serialLog.info("SerialEvent1!");
    Serial1.readBytes(energyMeterData, ENERGY_METER_DATA_SIZE);
    serialLog.info("Energy meter data: %s\n", energyMeterData);
    newEnergyMeterData = true;
}

void getMeasurements(uint8_t *data, uint32_t maxLength, uint32_t &length, uint8_t &count)
{
    uint32_t total = 0;
    count = 0;

    // Figure out how many measurements can fit in to buffer
    while (total < maxLength - count && count < tracker.unconfirmedCount())
    {
        total += tracker.getLengthOf(count);
        count++;
    }
    Log.info("Number of measurements: %d (%ld)\n", count, total);

    // Copy over data into data buffers
    uint32_t offset = 0;
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t id;
        uint8_t item_length;
        tracker.get(i, id, item_length, data + offset + 1);
        data[offset] = item_length;
        offset += item_length + 1;
    }

    length = offset;
}

void readSensors(SensorPacket *packet)
{
    uint32_t timestamp;
    if (rtcPresent)
    {
        DateTime now = rtc.now();
        timestamp = now.unixtime();
    }
    else
    {
        timestamp = Time.now();
    }
    packet->timestamp = timestamp;

    packet->sequence = sequence;

    if (rtcPresent)
    {
        packet->rtc_temperature = rtc.getTemperature();
        packet->has_rtc_temperature = true;
    }

    // packet->card_present = saveDataSucess;
    // packet->has_card_present = true;

    packet->queue_size = tracker.unconfirmedCount();
    packet->has_queue_size = true;

    if (pmSensor.dataAvailable())
    {
        pmSensor.getMass(pmMeasurement);
        packet->pm1 = pmMeasurement[0];
        packet->has_pm1 = true;
        packet->pm2_5 = pmMeasurement[1];
        packet->has_pm2_5 = true;
        packet->pm4 = pmMeasurement[2];
        packet->has_pm4 = true;
        packet->pm10 = pmMeasurement[3];
        packet->has_pm10 = true;
    }

    if (airSensor.dataAvailable())
    {
        uint32_t co2 = airSensor.getCO2();
        packet->co2 = co2;
        packet->has_co2 = true;

        float temp = airSensor.getTemperature();
        packet->temperature = (uint32_t)round(temp * 10);
        packet->has_temperature = true;

        float humidity = airSensor.getHumidity();
        packet->humidity = (uint32_t)round(humidity * 10);
        packet->has_humidity = true;
    }

    if (newEnergyMeterData)
    {
        StaticJsonDocument<ENERGY_METER_DATA_SIZE> doc;
        DeserializationError error = deserializeJson(doc, energyMeterData);

        if (error)
        {
            Log.warn("Unable to parse JSON...");
            Log.warn(error.c_str());
        }
        else
        {
            packet->current = (float)doc["c"] * 1000;
            packet->has_current = true;

            packet->voltage = doc["v"];
            packet->has_voltage = true;

            packet->total_energy = (float)doc["t"] * 1000;
            packet->has_total_energy = true;

            packet->power = doc["p"];
            packet->has_power = true;

            packet->apparent_power = doc["a"];
            packet->has_apparent_power = true;

            packet->reactive_power = doc["r"];
            packet->has_reactive_power = true;

            packet->power_factor = (float)doc["f"] * 100;
            packet->has_power_factor = true;
        }

        newEnergyMeterData = false;
    }

    if (resetReason != RESET_REASON_NONE)
    {
        packet->reset_reason = resetReason;
        packet->has_reset_reason = true;

        // Make sure to read reset reason only once
        resetReason = RESET_REASON_NONE;
    }

#if Wiring_WiFi
    uint32_t freeMem = System.freeMemory();
    packet->free_memory = freeMem;
    packet->has_free_memory = true;
#endif // Wiring_WiFi
}

bool packMeasurement(SensorPacket *inPacket, uint32_t inLength, uint8_t *out, uint8_t *outLength)
{
    pb_ostream_t stream = pb_ostream_from_buffer(out, inLength);

    if (!pb_encode(&stream, SensorPacket_fields, inPacket))
    {
        encodeLog.error("Unable to encode data into protobuffer");
        return false;
    }

    *outLength = stream.bytes_written;
    encodeLog.trace("Bytes written to protobuffer: %d", stream.bytes_written);
    return true;
}

bool encodeMeasurements(uint8_t *in, uint32_t inLength, uint8_t *out, uint32_t *outLength)
{
    char *end_ptr = bintob85((char *)out, in, inLength);
    *outLength = end_ptr - (char *)out;

    encodeLog.trace("Added bytes for encoding: %ld", *outLength - inLength);

    if (encodeLog.isTraceEnabled())
    {
        for (unsigned int i = 0; i < inLength; i++)
        {
            Serial.printf("%02x ", in[i]);
        }
        Serial.println();

        for (unsigned int i = 0; i < *outLength; i++)
        {
            Serial.printf("%02x ", out[i]);
        }
        Serial.println();
    }

    return true;
}

int cloudReset(String arg) {
    Serial.println("Cloud reset called...");
    resetFlag = true;
    rebootSync = millis();
    return 0;
}

void printSystemInfo()
{
    uint32_t freeMem = System.freeMemory();
    Log.info("Free memory: %ld", freeMem);

#if Wiring_WiFi
    WiFiSignal sig = WiFi.RSSI();
    Log.info("WiFi quality: %.02f (%.02f)", sig.getQualityValue(), sig.getStrengthValue());
#endif // Wiring_WiFi

#if Wiring_Cellular
    CellularData data;
    Cellular.getDataUsage(data);
    Log.info("Cell usage: TX – %d, RX – %d", data.tx_total, data.rx_total);

    CellularSignal sig = Cellular.RSSI();
    Log.info("Cell quality: %d (%d)", sig.qual, sig.rssi);
#endif // Wiring_Cellular
}

void printPacket(SensorPacket *packet)
{
    Log.info("Packet:");
    Log.info("\tTimestamp: %ld", packet->timestamp);
    Log.info("\tSequence: %ld", packet->sequence);

    if (packet->has_temperature)
    {
        Log.info("\tTemperature: %ld", packet->temperature);
    }

    if (packet->has_temperature)
    {
        Log.info("\tHumidity: %ld", packet->humidity);
    }

    if (packet->has_rtc_temperature)
    {
        Log.info("\tRTC temperature: %ld", packet->rtc_temperature);
    }

    if (packet->has_pm1)
    {
        Log.info("\tPM1: %ld", packet->pm1);
    }

    if (packet->has_pm2_5)
    {
        Log.info("\tPM2.5: %ld", packet->pm2_5);
    }

    if (packet->has_pm4)
    {
        Log.info("\tPM4: %ld", packet->pm4);
    }

    if (packet->has_pm10)
    {
        Log.info("\tPM10: %ld", packet->pm10);
    }

    if (packet->has_card_present)
    {
        Log.info("\tCard present: %d", packet->card_present);
    }

    if (packet->has_queue_size)
    {
        Log.info("\tQueue size: %ld", packet->queue_size);
    }

    if (packet->has_co2)
    {
        Log.info("\tC02: %ld", packet->co2);
    }

    if (packet->has_voltage)
    {
        Log.info("\tVoltage: %ld", packet->voltage);
    }

    if (packet->has_current)
    {
        Log.info("\tCurrent: %ld", packet->current);
    }

    if (packet->has_total_energy)
    {
        Log.info("\tTotal energy: %ld", packet->total_energy);
    }

    if (packet->has_power)
    {
        Log.info("\tPower: %ld", packet->power);
    }

    if (packet->has_apparent_power)
    {
        Log.info("\tApparent power: %ld", packet->apparent_power);
    }

    if (packet->has_reactive_power)
    {
        Log.info("\tReactive power: %ld", packet->reactive_power);
    }

    if (packet->has_power_factor)
    {
        Log.info("\tPower factor: %ld", packet->power_factor);
    }

    if (packet->has_free_memory)
    {
        Log.info("\tFree Memory: %ld", packet->free_memory);
    }

    if (packet->has_reset_reason)
    {
        Log.info("\tReset Reason: %ld", packet->reset_reason);
    }
}