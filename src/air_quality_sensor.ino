#include "base85.h"
#include "DataLogger.h"
#include "jled.h"
#include "pb_encode.h"
#include "PublishQueueAsyncRK.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"

#define READ_PERIOD_MS 15000
#define UPLOAD_PERIOD_MS 1000

#define LED1 D6
#define LED2 D7
#define LED3 D8

// SD Card
DataLogger logger;

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
bool sdCardSuccess = true;
uint32_t queueSize = 0;

// Publishing information
particle::Future<bool> currentPublish;
bool currentlyPublishing = false;

// LED statuses
// LEDStatus errorLEDStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
// LEDStatus normaLEDStatus(RGB_COLOR_BLUE, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
auto successfulBootLED = JLed(LED1).On();
auto failedBootLED = JLed(LED1).Blink(500, 500).Forever();
auto bootLED = successfulBootLED;
auto readLED = JLed(LED2);
auto publishLED = JLed(LED3);

// Timers
Timer readTimer(READ_PERIOD_MS, updateReadDataFlag);
bool readDataFlag = true;
Timer uploadTimer(UPLOAD_PERIOD_MS, updateUploadFlag);
bool uploadFlag = true;

// Logging
SerialLogHandler logHandler;

// TEMPORARY
char publishData[256];
bool uploaded = true;

SYSTEM_THREAD(ENABLED);
// SYSTEM_MODE(MANUAL);

void setup()
{
    bool success = true;

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
        bootLED = failedBootLED;
    }
}

void loop()
{
    // Read sensor task
    if (readDataFlag)
    {
        Log.info("Reading sensors...");
        SensorPacket packet = SensorPacket_init_zero;
        readSensors(&packet);
        printPacket(&packet);

        Log.info("Putting data into a protobuf and base85 encoding...");
        encode(&packet, publishData);

        // TODO: Normally we would write to a file, but since we are testing, we are writing to an array
        uploaded = false;

        // Write data to SD card
        if (true /*logger.write(packet)*/)
        {
            sdCardSuccess = true;
            readLED.Blink(4000, 500);
        }
        else
        {
            sdCardSuccess = false;
            readLED.Blink(500, 500).Forever();
        }

        sequence++;
        readDataFlag = false;
    }

    // Check on message that is currently being published
    if (currentlyPublishing && currentPublish.isDone())
    {
        if (currentPublish.isSucceeded())
        {
            // Update queue
            // logger.ackData();
            Log.info("Publication was successful!");
            publishLED.Blink(4000, 500);
            uploaded = true;
        }
        else
        {
            Log.warn("Publication was NOT successful!");
            publishLED.Blink(500, 500).Forever();
        }

        currentlyPublishing = false;
    }

    // Upload data
    if (uploadFlag)
    {
        // Log.info("Trying to upload data...");
        // Log.info("\tcurrentlyPublishing: %d", currentlyPublishing);
        // Log.info("\tParticle.connected(): %d", Particle.connected());
        // Log.info("\tuploaded: %d", uploaded);
        if (!currentlyPublishing && Particle.connected() /*&& logger.hasNext()*/ && !uploaded)
        {
            // uint32_t data = logger.getNext();

            Log.info("Publishing data: " + String(publishData));
            currentPublish = Particle.publish("mn/d", publishData, 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
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

    // TODO: Add osVersion
    // uint32_t osVersion = System.versionNumber();

    int32_t rtcTemperature = rtc.getTemperature();
    packet->rtc_temperature = rtcTemperature;
    packet->has_rtc_temperature = true;

    // TODO: Add freeMemory
    uint32_t freeMem = System.freeMemory();
    Serial.printf("Free memory: %d\n", freeMem);

    packet->card_present = sdCardSuccess;
    packet->has_card_present = true;

    packet->queue_size = queueSize;
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
}

bool encode(SensorPacket *in_packet, char *out)
{
    uint8_t buffer[SensorPacket_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer) / sizeof(buffer[0]));

    if (!pb_encode(&stream, SensorPacket_fields, in_packet))
    {
        Log.error("Unable to encode data into protobuffer");
        return false;
    }
    Log.info("Bytes written to protobuffer: %d", stream.bytes_written);

    bintob85(publishData, buffer, stream.bytes_written);

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
    Serial.printf("Packet:\n");
    Serial.printf("\tTimestamp: %d\n", packet->timestamp);
    Serial.printf("\tSequence: %d\n", packet->sequence);
    Serial.printf("\tTemperature: %d\n", packet->temperature);
    Serial.printf("\tHumidity: %d\n", packet->humidity);
    Serial.printf("\tRTC temperature: %d\n", packet->rtc_temperature);
    Serial.printf("\tPM1: %d\n", packet->pm1);
    Serial.printf("\tPM2.5: %d\n", packet->pm2_5);
    Serial.printf("\tPM4: %d\n", packet->pm4);
    Serial.printf("\tPM10: %d\n", packet->pm10);
    Serial.printf("\tCard present: %d\n", packet->card_present);
    Serial.printf("\tQueue size: %d\n", packet->queue_size);
    Serial.printf("\tC02: %d\n", packet->co2);
    Serial.printf("\tVoltage: %d\n", packet->voltage);
    Serial.printf("\tCurrent: %d\n", packet->current);
    Serial.printf("\tWatt Hours: %d\n", packet->watt_hours);
}