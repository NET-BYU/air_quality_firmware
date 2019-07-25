#include "DataLogger.h"
#include "PublishQueueAsyncRK.h"
#include "RTClibrary.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"

#define READ_PERIOD_MS 1000
#define UPLOAD_PERIOD_MS 5000

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
int success = 0;
int failures = 0;

// Publishing information
particle::Future<bool> currentPublish;
bool currentlyPublishing = false;

// LED statuses
LEDStatus errorLEDStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
LEDStatus normaLEDStatus(RGB_COLOR_BLUE, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);

// Timers
Timer readTimer(READ_PERIOD_MS, updateReadDataFlag);
bool readDataFlag = true;
Timer uploadTimer(UPLOAD_PERIOD_MS, updateUploadFlag);
bool uploadFlag = true;

// SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

void setup()
{
    Serial.begin(9600);
    delay(3000);

    if (!rtc.begin())
    {
        Serial.println("Could not start RTC!");
    }

    if (!pmSensor.begin())
    {
        Serial.println("Could not start PM sensor!");
    }

    if (!airSensor.begin())
    {
        Serial.println("Could not start CO2 sensor!");
    }

    readTimer.start();
    uploadTimer.start();
}

void loop()
{
    // Read sensor task
    if (readDataFlag)
    {
        String data = readSensors();

        // Write data to SD card
        if (logger.write(data))
        {
            normaLEDStatus.setActive(true);
            success++;
        }
        else
        {
            errorLEDStatus.setActive(true);
            failures++;
        }

        sequence++;
        Serial.println(data + "\n");
        readDataFlag = false;
    }

    // Check on message that is currently being published
    if (currentlyPublishing && currentPublish.isDone())
    {
        if (currentPublish.isSucceeded())
        {
            // Update queue
            logger.ackData();
        }

        currentlyPublishing = false;
    }

    // Upload data
    if (uploadFlag)
    {
        if (!currentlyPublishing && Particle.connected() && logger.hasNext())
        {
            uint32_t data = logger.getNext();
            currentPublish = Particle.publish("mn/d", String(data), 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
        }

        uploadFlag = false;
    }
}

String readSensors()
{
    String data = "";

    DateTime now = rtc.now();
    data += String(now.unixtime()) + " ";

    data += String(sequence) + " ";

    uint32_t osVersion = System.versionNumber();
    data += String(osVersion) + " ";

    int32_t rtcTemperature = rtc.getTemperature();
    data += String(rtcTemperature) + " ";

    uint32_t freeMem = System.freeMemory();
    data += String(freeMem) + " ";

    data += String(failures) + " ";

    if (pmSensor.dataAvailable())
    {
        pmSensor.getMass(pmMeasurement);
        data += String(pmMeasurement[0]) + " " + String(pmMeasurement[1]) + " " + String(pmMeasurement[2]) + " " + String(pmMeasurement[3]) + " ";
    }

    if (airSensor.dataAvailable())
    {
        uint32_t co2 = airSensor.getCO2();
        data += String(co2) + " ";

        float temp = airSensor.getTemperature();
        data += String(temp) + " ";

        float humidity = airSensor.getHumidity();
        data += String(humidity) + " ";
    }

    return data;
}

void updateReadDataFlag()
{
    readDataFlag = true;
}

void updateUploadFlag()
{
    uploadFlag = true;
}