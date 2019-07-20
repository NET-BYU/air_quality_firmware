/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "/Users/philipbl/Desktop/air_quality_sensor/src/air_quality_sensor.ino"
#include "DataLogger.h"

void setup();
void loop();
#line 3 "/Users/philipbl/Desktop/air_quality_sensor/src/air_quality_sensor.ino"
int total = 0;
int success = 0;
int failures = 0;
DataLogger logger;
LEDStatus errorLEDStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);
LEDStatus normaLEDStatus(RGB_COLOR_BLUE, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_NORMAL);

// SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

void setup()
{
    normaLEDStatus.setActive(true);

    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial)
    {
        yield();
    }
    delay(3000);
}

void loop()
{
    String data = String(millis()) + " " + String(success) + " " + String(failures) + " " + String(total);
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
    Serial.println(data);
    delay(1000);
}