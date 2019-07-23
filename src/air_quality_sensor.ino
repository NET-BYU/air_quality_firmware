#include "DataLogger.h"
#include "PublishQueueAsyncRK.h"
#include "RTClibrary.h"

// Queue
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

// SD Card
DataLogger logger;

// RTC
RTC_DS3231 rtc;

// Counters
int sequence = 0;
int success = 0;
int failures = 0;

// Timing related to reading the sensors
const unsigned long READ_PERIOD_MS = 30000;
unsigned long lastRead = 8000 - READ_PERIOD_MS;

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
    DateTime now = rtc.now();
    int32_t rtcTemperature = rtc.getTemperature();
    uint32_t freeMem = System.freeMemory();
    uint32_t osVersion = System.versionNumber();

    // TODO: Format data
    String data = String(now.unixtime()) + " " +
                  String(osVersion) + " " +
                  String(rtcTemperature) + " " +
                  String(freeMem) + " " +
                  String(success) + " " +
                  String(failures) + " " +
                  String(sequence);

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