#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "PublishQueueAsyncRK.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "SimpleAckTracker.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"
#include "SQLAckTracker.h"

#define READ_PERIOD_MS 60000
#define UPLOAD_PERIOD_MS 1000

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
Timer readTimer(READ_PERIOD_MS, updateReadDataFlag);
bool readDataFlag = true;
Timer uploadTimer(UPLOAD_PERIOD_MS, updateUploadFlag);
bool uploadFlag = true;

// Logging
SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO}});

// Particle system stuff
SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

void setup()
{
    bool success = true;
    resetReason = System.resetReason();

    Serial.begin(9600);
    delay(3000);

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

    readTimer.start();
    uploadTimer.start();

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
        uint8_t *data = new uint8_t[256]; // TODO: Temporary, do not allocate on the heap
        uint32_t length;

        if (encode(&packet, data, &length))
        {
            tracker.add(tracker.pack(packet.timestamp, length, data));
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
            AckTracker::Packet packet = tracker.next(); // TODO: Temporary, remove completely
            tracker.confirmNext();
            delete packet.data; // TODO: Temporary, remove completely
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
        Log.trace("Trying to upload data... (%d, %d, %d)", currentlyPublishing, Particle.connected(), tracker.amount() > 0);
        if (!currentlyPublishing && Particle.connected() && tracker.amount() > 0)
        {
            AckTracker::Packet packet = tracker.next();

            Log.info("Publishing data: " + String((char *)packet.data));
            currentPublish = Particle.publish("mn/d", (char *)packet.data, 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
            publishLED.On().Update();
        }

        uploadFlag = false;
    }

    // Update LEDs
    bootLED.Update();
    readLED.Update();
    publishLED.Update();
}

void readSensors(SensorPacket *packet)
{
    DateTime now = rtc.now();
    packet->timestamp = now.unixtime();

    packet->sequence = sequence;

    int32_t rtcTemperature = rtc.getTemperature();
    packet->rtc_temperature = rtcTemperature;
    packet->has_rtc_temperature = true;

    uint32_t freeMem = System.freeMemory();
    Log.info("Free memory: %ld", freeMem);

    packet->card_present = saveDataSucess;
    packet->has_card_present = true;

    packet->queue_size = tracker.amount();
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

bool encode(SensorPacket *in_packet, uint8_t *out, uint32_t *length)
{
    uint8_t buffer[SensorPacket_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer) / sizeof(buffer[0]));

    if (!pb_encode(&stream, SensorPacket_fields, in_packet))
    {
        Log.error("Unable to encode data into protobuffer");
        return false;
    }
    Log.trace("Bytes written to protobuffer: %d", stream.bytes_written);

    char *end_ptr = bintob85((char *)out, buffer, stream.bytes_written);
    *length = end_ptr - (char *)out;

    return true;
}

void updateReadDataFlag()
{
    readDataFlag = true;
}

void updateUploadFlag()
{
    uploadFlag = true;
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