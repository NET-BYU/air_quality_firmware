#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "SimpleAckTracker.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"

#define READ_PERIOD_MS 10000
#define UPLOAD_PERIOD_MS 1000
#define PRINT_SYS_INFO_MS 1000000

#define MAX_PUB_SIZE 600 // It is really 622

#define LED1 D6
#define LED2 D7
#define LED3 D8

// Write data points to SD card and keep track of what has been ackowledged
SimpleAckTracker tracker;

// PM Sensor
SPS30 pmSensor;
float pmMeasurement[4];

// CO2 + Temp + Humidity Sensor
SCD30 airSensor;

// RTC
RTC_DS3231 rtc;

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
bool printSystemInfoFlag = true;
Timer printSystemInfoTimer(PRINT_SYS_INFO_MS, []() { printSystemInfoFlag = true; });

// Logging
Logger encodeLog("app.enocde");
SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO},
                                             {"app.enocde", LOG_LEVEL_TRACE}});

// Particle system stuff
SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

void setup()
{
    bool success = true;
    resetReason = System.resetReason();

    Serial.begin(9600);
    delay(5000);

    if (!rtc.begin())
    {
        Log.error("Could not start RTC!");
        success = false;
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
    printSystemInfoTimer.start();
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

        Log.info("Putting data into a protobuf and base85 encoding...");
        uint8_t data[256];
        uint8_t length;

        if (encode(&packet, data, &length))
        {
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
            tracker.confirmNext();
            publishLED.Off().Update();
        }
        else
        {
            Log.warn("Publication was NOT successful!");
            publishLED.Blink(250, 250).Forever();
        }

        currentlyPublishing = false;
    }

    // Upload data
    if (uploadFlag)
    {
        Log.trace("Trying to upload data... (%d, %d, %d)", !currentlyPublishing, Particle.connected(), tracker.unconfirmedCount() > 0);
        if (!currentlyPublishing && Particle.connected() && tracker.unconfirmedCount() > 0)
        {
            uint8_t data[MAX_PUB_SIZE];
            getMeasurements(data);

            Log.info("Publishing data: " + String((char *)data));
            currentPublish = Particle.publish("netlab/test", (char *)data, 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
            publishLED.On().Update();
        }

        uploadFlag = false;
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

void getMeasurements(uint8_t *data)
{
    uint32_t count = 0;
    uint32_t total = 0;

    // Figure out how many measurements can fit in to buffer
    // TODO: I'm pretty sure that count the NULL terminator even though only the last one is preserved.
    while (total < MAX_PUB_SIZE - count && count < tracker.unconfirmedCount())
    {
        total += tracker.getLengthOf(count);
        count++;
    }
    Log.info("Number of measurements: %ld (%ld)\n", count, total);

    // Copy over data into data buffers
    uint32_t offset = 0;
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t id;
        uint8_t length;
        tracker.get(i, id, length, data + offset + 1);
        data[offset] = length - 1; // Subtract 1 because of the null terminator byte
        offset += length - 1;
    }
}

void readSensors(SensorPacket *packet)
{
    DateTime now = rtc.now();
    packet->timestamp = now.unixtime();

    packet->sequence = sequence;

    int32_t rtcTemperature = rtc.getTemperature();
    packet->rtc_temperature = rtcTemperature;
    packet->has_rtc_temperature = true;

    packet->card_present = saveDataSucess;
    packet->has_card_present = true;

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

    if (resetReason != RESET_REASON_NONE)
    {
        packet->reset_reason = resetReason;
        packet->has_reset_reason = true;

        // Make sure to read reset reason only once
        resetReason = RESET_REASON_NONE;
    }
}

bool encode(SensorPacket *in_packet, uint8_t *out, uint8_t *length)
{
    uint8_t buffer[SensorPacket_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer) / sizeof(buffer[0]));

    if (!pb_encode(&stream, SensorPacket_fields, in_packet))
    {
        encodeLog.error("Unable to encode data into protobuffer");
        return false;
    }
    encodeLog.trace("Bytes written to protobuffer: %d", stream.bytes_written);

    char *end_ptr = bintob85((char *)out, buffer, stream.bytes_written);
    *length = end_ptr - (char *)out;

    encodeLog.trace("Added bytes for encoding: %d", *length - stream.bytes_written);

    if (encodeLog.isTraceEnabled())
    {
        for (unsigned int i = 0; i < stream.bytes_written; i++)
        {
            Serial.printf("%02x ", buffer[i]);
        }
        Serial.println();

        for (unsigned int i = 0; i < *length; i++)
        {
            Serial.printf("%02x ", out[i]);
        }
        Serial.println();
    }

    return true;
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
    Log.info("\tTemperature: %ld", packet->temperature);
    Log.info("\tHumidity: %ld", packet->humidity);
    Log.info("\tRTC temperature: %ld", packet->rtc_temperature);
    Log.info("\tPM1: %ld", packet->pm1);
    Log.info("\tPM2.5: %ld", packet->pm2_5);
    Log.info("\tPM4: %ld", packet->pm4);
    Log.info("\tPM10: %ld", packet->pm10);
    Log.info("\tCard present: %d", packet->card_present);
    Log.info("\tQueue size: %ld", packet->queue_size);
    Log.info("\tC02: %ld", packet->co2);
    Log.info("\tVoltage: %ld", packet->voltage);
    Log.info("\tCurrent: %ld", packet->current);
    Log.info("\tWatt Hours: %ld", packet->watt_hours);
}