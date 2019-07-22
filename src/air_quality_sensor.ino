#include "DataLogger.h"
#include "RTClibrary.h"

// Declare functions
void uploadData();
void readSensors();

// Global variables
DataLogger logger;
RTC_DS3231 rtc;

// Counters
int total = 0;
int success = 0;
int failures = 0;

// LED statuses
LEDStatus errorLEDStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
LEDStatus normaLEDStatus(RGB_COLOR_BLUE, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);

// Timers to run functionality
Timer sensorTimer(1000, readSensors);
Timer uploadTimer(1000, uploadData);

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

    sensorTimer.start();
    // uploadTimer.start();
}

void loop()
{
}

void readSensors()
{
    ////////////////////////////////////
    //       READ SENSORS HERE        //
    ////////////////////////////////////
    DateTime now = rtc.now();
    int32_t rtcTemperature = rtc.getTemperature();
    uint32_t freeMem = System.freeMemory();


    // TODO: Format data

    String data = String(now.unixtime()) + " " + String(rtcTemperature) + " " + String(freeMem) + " " + String(success) + " " + String(failures) + " " + String(total);

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
    total++;
    Serial.println(data + "\n");
}

void uploadData()
{
    Serial.println("Upload data...\n");
}