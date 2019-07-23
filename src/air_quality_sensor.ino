#include "DataLogger.h"
#include "PublishQueueAsyncRK.h"
#include "RTClibrary.h"
#include "SPS30.h"

// Queue
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

// SD Card
DataLogger logger;

// PM Sensor
SPS30 pmSensor;
float pmMeasurement[4];

// RTC
RTC_DS3231 rtc;

// Counters
int sequence = 0;
int success = 0;
int failures = 0;

// Timing related to reading the sensors
const unsigned long READ_PERIOD_MS = 1000;
unsigned long lastRead = 0;

// LED statuses
LEDStatus errorLEDStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
LEDStatus normaLEDStatus(RGB_COLOR_BLUE, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);

// SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        Particle.process();
    }
    delay(3000);

    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
    }

    if (!pmSensor.begin())
    {
        Serial.println("PM sensor not connected!");
    }

    publishQueue.setup();
}

void loop()
{
    if (millis() - lastRead < READ_PERIOD_MS)
    {
        return;
    }

    lastRead = millis();

    ////////////////////////////////////
    //       READ SENSORS HERE        //
    ////////////////////////////////////
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

    // Insert data into queue to be published
    // publishQueue.publish("mn/d", data.c_str(), 60, PRIVATE, WITH_ACK);

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
}
