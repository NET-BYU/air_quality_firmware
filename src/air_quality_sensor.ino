#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "AckTracker.h"
#include "FileAckTracker.h"
#include "MemoryAckTracker.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"
#include "ArduinoJson.h"
#include "PersistentCounter.h"
#include "PersistentConfig.h"
#include "SdFat.h"
#include "SdCardLogHandlerRK.h"
#include "adafruit-sht31.h"
#include "DiagnosticsHelperRK.h"

#if PLATFORM_ID == PLATFORM_ARGON
PRODUCT_ID(9901);
PRODUCT_VERSION(3);
#endif

#if PLATFORM_ID == PLATFORM_BORON
PRODUCT_ID(9861);
PRODUCT_VERSION(3);
#endif

// Trace Heater
#define TRACE_HEATER_PIN D7
#define TRACE_HEATER_ON LOW     // LOW activates the PMOS, while HIGH disables the PMOS controlling the trace heater current
#define TRACE_HEATER_OFF HIGH
/*
                -----------------------------------------
                | offLengthSec = 0  | offLengthSec > 0  |
---------------------------------------------------------
onLengthSec = 0 | heater disabled   | heater disabled   |
onLengthSec > 0 | heater always on  | heater uses timer |
---------------------------------------------------------
 */
// onLengthSec and offLengthSec can be changed with a cloud function
// uint32_t heaterOnLengthSec = 0;    // The number of seconds the trace heater should be on
// uint32_t heaterOffLengthSec = 0;   // The number of seconds the trace heater should be off
uint8_t traceHeaterState = TRACE_HEATER_OFF;    // Whether the trace heater is currently on or off
uint32_t lastTraceHeaterToggle = 0; // The unix epoch timestamp of the last time the trace heater toggled states. 0 means it was never toggled.

// Energy Sensor Stuff
#define ENERGY_SENSOR_PRESENT_PIN D6
#define AC_PIN A0           //set arduino signal read pin
#define ACTectionRange 20   //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
#define VREF 3.3            // VREF: Analog reference
// #define UPPER_VOLTAGE_THRESHOLD 3000    // Some value less than ADC_MAX used for detecting a pull down resistor
#define ENERGY_SENSOR_DETECTED LOW  // What the ENERGY_SENSOR_PRESENT_PIN should read if there is an energy sensor

// Battery Stuff
#define BATTERY_POWER_PIN D2  // Controls the relay and boost converter for the battery
#define BATTERY_ON_OUT HIGH  // Write to relay to turn on boost converter
#define BATTERY_OFF_OUT LOW  // Opposite of above
#define POWER_SOURCE_BATTERY 5  // Value returned from diag helper if the system is powered by battery
bool poweringFromBattery = false;

// Counters
#define SEQUENCE_COUNT_ADDRESS 0x00
PersistentCounter sequence(SEQUENCE_COUNT_ADDRESS);

// Sensor configuration
#define CONFIG_ADDRESS 0x04
PersistentConfig config(CONFIG_ADDRESS);

// SD Card
SdFat sd;
const int SD_CHIP_SELECT = A5;  // Pin on sd card reader, needs to be passed to SDFat Lib

// Write data points to SD card and keep track of what has been ackowledged
#define ENTRY_SIZE 300
FileAckTracker fileTracker(sd, SD_CHIP_SELECT, ENTRY_SIZE);
MemoryAckTracker<ENTRY_SIZE, 60> memoryTracker;
AckTracker *currentTracker;
uint32_t pendingPublishes = 0;
bool trackerSetup = true;

// PM Sensor
SPS30 pmSensor;
float pmMeasurement[4]; // PM 1, 2.5, 4, 10
bool pmSensorSetup = true;

// CO2 + Temp + Humidity Sensor
SCD30 airSensor;
bool airSensorSetup = true;

// RTC
RTC_DS3231 rtc;
bool rtcPresent = true;
bool rtcSet = true;

#define TEMP_HUM_I2C_ADDR 0x44

// Temp + Humidity Sensor
Adafruit_SHT31 sht31; // = Adafruit_SHT31();
float tempMeasurement;
float humidityMeasurement;
bool tempHumPresent = true;

// CO Sensor
bool coPresent = true;

// Power Management IC
#if PLATFORM_ID == PLATFORM_BORON
PMIC pmic;
#endif

// Energy Meter Data
#define ENERGY_METER_DATA_SIZE 200
char energyMeterData[ENERGY_METER_DATA_SIZE];
bool newEnergyMeterData = false;

// Serial device
#define SERIAL_TYPE_UNKNOWN 0
#define SERIAL_TYPE_ENERGY 1
#define SERIAL_TYPE_CO 2
char serialDeviceType = SERIAL_TYPE_UNKNOWN; // Assume at first that the device attached is a CO sensor
#define SERIAL_DATA_SIZE 200
char serialData[SERIAL_DATA_SIZE];
bool newSerialData = false;

// Global variables to keep track of state
int resetReason = RESET_REASON_NONE;

// Publishing information
particle::Future<bool> currentPublish;
bool currentlyPublishing = false;

// Flag for publishing status information
bool publishStatus = false;
bool publishingStatus = false;

// LEDs
auto sensorLed = JLed(D3);
auto sdLed = JLed(D4);
auto cloudLed = JLed(D5);

// Timers
bool readDataFlag = true;
Timer readTimer(config.data.readPeriodMs, []() { readDataFlag = true; });  // How often the system reads from the sensors

bool uploadFlag = true;
Timer uploadTimer(config.data.uploadPeriodMs, []() { uploadFlag = true; });  // How often the info is uploaded to the particle cloud --> google cloud --> MQTT --> db

bool printSystemInfoFlag = true;
Timer printSystemInfoTimer(config.data.printSysInfoMs, []() { printSystemInfoFlag = true; });  // Periodically write data usage in the logs

Timer resetTimer(config.data.delayBeforeReboot, resetDevice, true);  // Periodically resets the senspr (like once an hour...?) to avoid weird catches

Timer updateRtcTimer(3600000, []() { rtcSet = false; });  // Same as reset timer for RTC to avoid weirdness...

#define MAX_RECONNECT_COUNT 30
uint32_t connectingCounter = 0;
Timer connectingTimer(60000, checkConnecting);  // Time it will try to connect before reseting the device

#define LENGTH_HEADER_SIZE 2

// Logging
Logger encodeLog("app.encode");
Logger serialLog("app.serial");

#define SD_LOGGING 1
#if SD_LOGGING
SdCardLogHandler<2048> sdLogHandler(sd, SD_CHIP_SELECT, SPI_FULL_SPEED, LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO}, {"app.encode", LOG_LEVEL_INFO}});
#else
SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO},
                                             {"app.encode", LOG_LEVEL_INFO},
                                             {"FileAckTracker", LOG_LEVEL_INFO},
                                             {"MemoryAckTracker", LOG_LEVEL_TRACE}});
#endif

// Particle system stuff
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  int32_t peakVoltage = 0;  
  float voltageVirtualValue = 0;  //Vrms
  for (int i = 0; i < 1000; i++)
  {
    peakVoltage += analogRead(AC_PIN);   //read peak voltage
    delay(1);
  }
  peakVoltage = peakVoltage / 1000;  
  Serial.printf("ADC:%x\t\t", peakVoltage);
  voltageVirtualValue = peakVoltage * 0.707;    //change the peak voltage to the Virtual Value of voltage

  /*The circuit is amplified by 2 times, so it is divided by 2.*/
  voltageVirtualValue = (voltageVirtualValue / 4096 * VREF ) / 2;  

  Log.info("voltageVirtualValue=%f", voltageVirtualValue);

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;

  return ACCurrtntValue;
}

void setup()
{
    // Set up cloud functions
    Particle.function("reset", cloudReset);
    Particle.function("resetCo", cloudResetCoprocessor);
    Particle.function("unack", cloudUnackMeasurement);
    Particle.function("param", cloudParameters);

    // Get reset reason to publish later
    resetReason = System.resetReason();

    // Debugging port
    Serial.begin(9600);

    // Energy meter port
    // Serial1.begin(115200);
    serialDeviceType = SERIAL_TYPE_UNKNOWN;  // Even though we assume we have a CO sensor attached, we don't know yet
    Serial1.begin(9600);
    Serial1.flush();
    delay(1000);
    Serial1.write('A');
    delay(1000);
    Serial1.print(300);
    delay(500);
    Serial1.write('\r');
    delay(2500);


    // delay(5000);

    pinMode(ENERGY_SENSOR_PRESENT_PIN, INPUT_PULLUP);
    if (digitalRead(ENERGY_SENSOR_PRESENT_PIN) == ENERGY_SENSOR_DETECTED)
    {
        Log.info("Energy sensor present!");
    }

    pinMode(BATTERY_POWER_PIN, OUTPUT);
    digitalWrite(BATTERY_POWER_PIN, BATTERY_OFF_OUT);
    poweringFromBattery = false;

    if (!fileTracker.begin())
    {
        Log.error("Could not start file tracker. Using memory tracker");
        trackerSetup = false;
        currentTracker = &memoryTracker;
    }
    else
    {
        Log.info("Using file tracker.");
        currentTracker = &fileTracker;
    }

    if (!rtc.begin())
    {
        Log.error("Could not start RTC!");
        rtcPresent = false;
    }

    // Make sure RTC is really working
    uint32_t first = rtc.now().unixtime();
    delay(1000);
    uint32_t second = rtc.now().unixtime();
    if (first == second)
    {
        Log.error("Could not start RTC!");
        rtcPresent = false;
    }
    else
    {
        uint32_t now = rtc.now().unixtime();

        // Now make sure the time is somewhat right
        if (now < 1560000000)
        {
            Log.warn("RTC is present, but has not been set.");
            rtcSet = false;
        }
    }

    if (!pmSensor.begin())
    {
        Log.error("Could not start PM sensor!");
        pmSensorSetup = false;
    }

    if (!airSensor.begin())
    {
        Log.error("Could not start CO2 sensor!");
        airSensorSetup = false;
    }

    if (! sht31.begin(TEMP_HUM_I2C_ADDR))    // Set to 0x45 for alternate i2c addr
    {   
        Serial.println("Couldn't find SHT31 (temp humidity)!");
        tempHumPresent = false;
    }

    // Set the Trace heeater to be initially off
    pinMode(TRACE_HEATER_PIN, OUTPUT);
    digitalWrite(TRACE_HEATER_PIN, TRACE_HEATER_OFF);

    delay(1000);

    #if PLATFORM_ID == PLATFORM_BORON
    pmic.begin();
    #endif

#if SD_LOGGING
    sdLogHandler.setup();
#endif

    // Start timers
    readTimer.start();
    uploadTimer.start();
    connectingTimer.start();
    updateRtcTimer.start();

    if (config.data.enablePrintSystemInfo)
    {
        printSystemInfoTimer.start();
    }

    // Print out configuration information
    config.print();

    Particle.connect();
}

void loop() // Print out RTC status in loop
{
    // Read sensor task
    if (readDataFlag)
    {
        sensorLed.Blink(1000, 1000).Update();
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
            AckTracker *tracker = getAckTrackerForWriting();
            if (tracker->add(packet.sequence, length, data))
            {
                if (tracker == &fileTracker)
                {
                    sdLed.Blink(1000, 1000).Update();
                }
                else
                {
                    sdLed.Blink(400, 100).Repeat(5);
                }
            }
            else
            {
                Log.warn("Failed to add data to tracker");
                if (tracker == &fileTracker)
                {
                    Log.warn("Switching to MemoryAckTracker");
                    tracker = &memoryTracker;
                    if (tracker->add(packet.sequence, length, data))
                    {
                        Log.info("Data was successfully added to MemoryAckTracker");
                        sdLed.Blink(400, 100).Repeat(5);
                    }
                    else
                    {
                        Log.warn("Failed to add data to memoryTracker");
                        sdLed.Blink(100, 100).Forever();
                    }
                }
                else
                {
                    sdLed.Blink(400, 400).Forever();
                }
            }
        }
        else
        {
            Log.error("Packing measurement FAILED!");
            sdLed.Blink(1600, 1600).Forever();
        }

        sequence.increment();
        readDataFlag = false;
    }

    // Check on message that is currently being published
    if (currentlyPublishing && currentPublish.isDone())
    {
        if (currentPublish.isSucceeded())
        {
            Log.info("Publication was successful!");
            if (publishingStatus)
            {
                Log.info("Published status message");
            }
            else
            {
                AckTracker *tracker = getAckTrackerForReading();
                tracker->confirmNext(pendingPublishes);
                uint32_t unconfirmedCount;
                if (tracker->unconfirmedCount(&unconfirmedCount))
                {
                    Log.info("Unconfirmed count: %ld", unconfirmedCount);
                }
                else
                {
                    Log.error("Unable to get unconfirmed count");
                }
            }

            cloudLed.Off().Update();
        }
        else
        {
            Log.warn("Publication was NOT successful!");
            cloudLed.Blink(250, 250).Forever();
        }

        pendingPublishes = 0;
        currentlyPublishing = false;
        publishingStatus = false;
    }

    if (publishStatus && !currentlyPublishing && Particle.connected())
    {
        StaticJsonDocument<200> doc;
        doc["tracker"] = trackerSetup;
        doc["rtc"] = rtcPresent;
        doc["pm"] = pmSensorSetup;
        doc["air"] = airSensorSetup;

        char output[200];
        serializeJson(doc, output, sizeof(output));

        Log.info("Publishing status data: %s", output);
        currentPublish = Particle.publish("mn/s", output, 60, PRIVATE, WITH_ACK);
        currentlyPublishing = true;
        publishingStatus = true;
        cloudLed.On().Update();

        publishStatus = false;
    }

    // Upload data
    if (uploadFlag)
    {
        AckTracker *tracker = getAckTrackerForReading();
        uint32_t unconfirmedCount = 0;
        tracker->unconfirmedCount(&unconfirmedCount);

        Log.trace("Trying to upload data... (%d, %d, %ld >= %ld (%d))",
                  !currentlyPublishing,
                  Particle.connected(),
                  unconfirmedCount,
                  config.data.uploadBatchSize,
                  unconfirmedCount >= config.data.uploadBatchSize);
        if (!currentlyPublishing && Particle.connected() && (unconfirmedCount >= config.data.uploadBatchSize))
        {
            // TODO: Should do the ceiling of the division just to be safe
            uint32_t maxLength = config.data.maxPubSize - (config.data.maxPubSize / 4); // Account for overhead of base85 encoding
            uint8_t data[maxLength];
            uint32_t dataLength;
            getMeasurements(data, maxLength, &dataLength, &pendingPublishes, tracker);

            uint8_t encodedData[config.data.maxPubSize];
            uint32_t encodedDataLength;
            encodeMeasurements(data, dataLength, encodedData, &encodedDataLength);

            Log.info("Unconfirmed count: %ld", unconfirmedCount);
            Log.info("Publishing data: %s", (char *)encodedData);
            currentPublish = Particle.publish("mn/d", (char *)encodedData, 60, PRIVATE, WITH_ACK);
            currentlyPublishing = true;
            cloudLed.On().Update();
        }

        uploadFlag = false;
    }

    // Update RTC if needed
    if (rtcPresent && !rtcSet && Particle.connected() && Time.isValid())
    {
        Log.info("Setting clock...");
        rtc.adjust(DateTime(Time.now()));
        rtcSet = true;

        delay(500);

        DateTime now = rtc.now();
        uint32_t timestamp = now.unixtime();
        Log.info("Time is set to: %ld", timestamp);
    }

    // Handle Trace heater
    if (config.data.heaterOnLengthSec != 0)    // If the trace heater is NOT disabled
    {
        if (config.data.heaterOffLengthSec == 0 && traceHeaterState == TRACE_HEATER_OFF)   // If the trace heater is set to be ALWAYS ON, but it's off
        {
            traceHeaterState = TRACE_HEATER_ON;
            digitalWrite(TRACE_HEATER_PIN, traceHeaterState);
        }
        else if (config.data.heaterOffLengthSec > 0) //  && lastTraceHeaterToggle == 0 || heaterOnLengthSec != 0 && lastTraceHeaterToggle
        {
            uint32_t timeLimit = (traceHeaterState == TRACE_HEATER_ON) ? config.data.heaterOnLengthSec : config.data.heaterOffLengthSec;
            uint32_t timestamp_now = rtc.now().unixtime();
            if (lastTraceHeaterToggle == 0 || (timestamp_now - lastTraceHeaterToggle) > timeLimit)
            {
                traceHeaterState = !traceHeaterState;
                digitalWrite(TRACE_HEATER_PIN, traceHeaterState);
                lastTraceHeaterToggle = timestamp_now;
            }
        }
    }
    else if (traceHeaterState == TRACE_HEATER_ON)
    {
        traceHeaterState = TRACE_HEATER_OFF;
        digitalWrite(TRACE_HEATER_PIN, traceHeaterState);
    }
    
    // Battery detection and handling, for keeping 5V sensors on with 3V battery
    #if PLATFORM_ID == PLATFORM_BORON // Only for Particle Boron microcontroller
    if (DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE) == POWER_SOURCE_BATTERY)
    {
        if (!poweringFromBattery)
        {
            Log.info("Being powered by battery! Turning on boost converter.");
            digitalWrite(BATTERY_POWER_PIN, BATTERY_ON_OUT);
            poweringFromBattery = true;
        }
    }
    else if (poweringFromBattery)
    {
        Log.info("Being powered by wall! Turning off boost converter!");
        digitalWrite(BATTERY_POWER_PIN, BATTERY_OFF_OUT);
        poweringFromBattery = false;
    }
    #endif 

    if (connectingCounter >= MAX_RECONNECT_COUNT)
    {
        Log.warn("Rebooting myself because I've been connecting for too long.");
        System.reset();
    }

    if (printSystemInfoFlag)
    {
        printSystemInfo();
        printSystemInfoFlag = false;
    }

#if SD_LOGGING
    sdLogHandler.loop();
#endif

    // Update LEDs
    sensorLed.Update();
    sdLed.Update();
    cloudLed.Update();
}

AckTracker *getAckTrackerForWriting()
{
    Log.info("Getting AckTracker for writing...");
    if (fileTracker.begin())
    {
        Log.info("Using fileTracker");
        return &fileTracker;
    }
    else
    {
        Log.info("Using memoryTracker");
        return &memoryTracker;
    }
}

AckTracker *getAckTrackerForReading()
{
    Log.info("Getting AckTracker for reading...");
    if (currentlyPublishing)
    {
        Log.info("Currently publishing so keep the same AckTracker.");
        return currentTracker;
    }

    if (!fileTracker.begin())
    {
        Log.info("Unable to start fileTracker so using memoryTracker.");
        currentTracker = &memoryTracker;
        return currentTracker;
    }

    Log.info("Using fileTracker");
    currentTracker = &fileTracker;

    uint32_t memoryUnconfirmedCount = 0;
    memoryTracker.unconfirmedCount(&memoryUnconfirmedCount);

    if (memoryUnconfirmedCount == 0)
    {
        return currentTracker;
    }

    Log.info("Copying over %lu entries from memoryTracker to fileTracker", memoryUnconfirmedCount);
    uint32_t id;
    uint16_t length;
    uint8_t data[ENTRY_SIZE];
    for (uint32_t i = 0; i < memoryUnconfirmedCount; i++)
    {
        if (!memoryTracker.get(0, &id, &length, data))
        {
            Log.error("Unable to get %lu entry from memoryTracker", i);
            break;
        }

        if (fileTracker.add(id, length, data))
        {
            Log.info("Copied over an entry");
            memoryTracker.confirmNext(1);
        }
        else
        {
            Log.info("Unable to add entry to fileTracker");
            break;
        }
    }

    return currentTracker;
}

void serialEvent1()
{
    serialLog.info("SerialEvent1!");
    if (serialDeviceType == SERIAL_TYPE_CO || serialDeviceType == SERIAL_TYPE_UNKNOWN)
    {
        Serial1.readBytesUntil('\n', serialData, SERIAL_DATA_SIZE);
        serialLog.info("Serial Data received");
        newSerialData = true;
        if (serialDeviceType == SERIAL_TYPE_UNKNOWN)
        {
            uint32_t sensorNum;
            float conc;
            float temp;
            float rh;
            uint32_t conc_c;
            uint32_t temp_c;
            uint32_t rh_c;
            uint32_t days;
            uint32_t hours;
            uint32_t minutes;
            uint32_t seconds;
            int result = sscanf(serialData, "%lu, %f, %f, %f, %lu, %lu, %lu, %lu, %lu, %lu, %lu", &sensorNum, &conc, &temp, &rh, &conc_c, &temp_c, &rh_c, &days, &hours, &minutes, &seconds);
            if (result != EOF)
            {
                serialLog.info("Serial data is from CO sensor");
                serialDeviceType = SERIAL_TYPE_CO;
            }
            else
            {
                newSerialData = false;
                serialDeviceType = SERIAL_TYPE_ENERGY;
                Serial1.flush();
                Serial1.begin(115200);
            }
        }
    }
    else if (serialDeviceType == SERIAL_TYPE_ENERGY)
    {
        Serial1.readBytes(energyMeterData, ENERGY_METER_DATA_SIZE);
        serialLog.info("Energy meter data: %s\n", energyMeterData);
        newEnergyMeterData = true;
    }
}

bool connecting()
{
#if Wiring_WiFi
    return WiFi.connecting();
#elif Wiring_Cellular
    return Cellular.connecting();
#else
    return false;
#endif
}

void getMeasurements(uint8_t *data, uint32_t maxLength, uint32_t *length, uint32_t *count, AckTracker *tracker)
{
    Log.info("Max length: %ld", maxLength);
    uint32_t total = 0;
    uint16_t item_length = 0;
    uint32_t tmpCount = 0;

    // Figure out how many measurements can fit in to buffer
    uint32_t unconfirmedCount = 0;
    tracker->unconfirmedCount(&unconfirmedCount);
    while (total < maxLength && tmpCount < unconfirmedCount)
    {

        tracker->getLength(tmpCount, &item_length);
        Log.info("Getting length of item %ld: %d", tmpCount, item_length);
        total += item_length + LENGTH_HEADER_SIZE;
        tmpCount++;
    }

    if (total > maxLength)
    {
        // This means we have more unconfirmed measurements than we have room for
        // We need to remove the last added measurement because its what put us over the edge
        tmpCount--;
        tracker->getLength(tmpCount, &item_length);
        Log.info("Beyond max size (%ld > %ld), going back one measurement: %d.", total, maxLength, item_length);
        total -= item_length + LENGTH_HEADER_SIZE;
    }

    Log.info("Number of measurements: %ld (%ld)\n", tmpCount, total);

    // Copy over data into data buffers
    uint32_t offset = 0;
    for (uint32_t i = 0; i < tmpCount; i++)
    {
        uint32_t id;
        uint16_t item_length;
        tracker->get(i, &id, &item_length, data + offset + LENGTH_HEADER_SIZE);
        Log.info("Getting data from AckTracker (%lu, %lu, %u)", i, id, item_length);
        data[offset] = (uint8_t)item_length;
        data[offset + 1] = (uint8_t)(item_length >> 8);
        offset += item_length + LENGTH_HEADER_SIZE;
    }

    *length = offset;
    *count = tmpCount;
    Log.info("Length of data: %ld", *length);
}

char getUARTType()
{
    // StaticJsonDocument<ENERGY_METER_DATA_SIZE> doc;
    // DeserializationError error = deserializeJson(doc, energyMeterData);
    return SERIAL_TYPE_UNKNOWN;
}

void readSensors(SensorPacket *packet)
{
    uint32_t timestamp;
    if (rtcPresent)
    {
        Log.info("readSensors(): RTC is present");
        DateTime now = rtc.now();
        timestamp = now.unixtime();
    }
    else
    {
        Log.error("readSensors(): RTC is NOT present!");
        timestamp = Time.now();
    }
    if (rtcSet)
    {
        Log.info("readSensors(): RTC is set");
    }
    else
    {
        Log.error("readSensors(): RTC is NOT set!");
    }
    packet->timestamp = timestamp;

    packet->sequence = sequence.get();

    if (rtcPresent)
    {
        packet->rtc_temperature = rtc.getTemperature();
        packet->has_rtc_temperature = true;
    }

    packet->card_present = currentTracker == &fileTracker;
    packet->has_card_present = true;

    uint32_t unconfirmedCount;
    if (currentTracker->unconfirmedCount(&unconfirmedCount))
    {
        packet->queue_size = unconfirmedCount;
        packet->has_queue_size = true;
    }

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
    else
    {
        // There should always be data available so begin measuring again
        pmSensor.begin();
    }

    if (airSensor.dataAvailable())
    {
        uint32_t co2 = airSensor.getCO2();
        packet->co2 = co2;
        packet->has_co2 = true;

        float temp = airSensor.getTemperature();
        packet->temperature = (int32_t)round(temp * 10);
        packet->has_temperature = true;

        float humidity = airSensor.getHumidity();
        packet->humidity = (uint32_t)round(humidity * 10);
        packet->has_humidity = true;
        Log.info("readSensors(): CO2 - CO2=%ld, temp=%ld, hum=%ld", packet->co2, packet->temperature, packet->humidity);
    }
    else
    {
        Log.error("can't read CO2");
        // There should always be data available so begin measuring again
        airSensor.begin();
    }

    if (tempHumPresent)
    {
        float temp = sht31.readTemperature();
        packet->temperature = (int32_t)round(temp * 10);
        packet->has_temperature = true;

        float humidity = sht31.readHumidity();
        packet->humidity = (uint32_t)round(humidity * 10);
        packet->has_humidity = true;

        Log.info("readSensors(): tempHum - temp=%ld, hum=%ld", packet->temperature, packet->humidity);
    }
    else
    {
        sht31.begin(TEMP_HUM_I2C_ADDR);
    }

    #if PLATFORM_ID == PLATFORM_BORON
    Log.info("readSensors(): InputSourceRegister=0x%x", pmic.readInputSourceRegister());
    #endif

    if (digitalRead(ENERGY_SENSOR_PRESENT_PIN) == ENERGY_SENSOR_DETECTED)
    {
        Log.info("readSensors(): Energy sensor detected.");
        float acCurrentValue = readACCurrentValue(); //read AC Current Value
        float measuredPower = acCurrentValue * config.data.countryVoltage;
        Log.info("countryVoltage: int voltage=%ld", config.data.countryVoltage);
        Log.info("readSensors(): float current=%f", acCurrentValue);
        packet->has_current = true;
        packet->current = (int32_t) (acCurrentValue * 1000);
        Log.info("readSensors(): float power=%f", measuredPower);
        packet->has_power = true;
        packet->power = measuredPower;
        // Log.info("readSensors(): current=%ld", packet->current);
    }

    if (serialDeviceType == SERIAL_TYPE_CO || serialDeviceType == SERIAL_TYPE_UNKNOWN)
    {
        Serial1.write('\r');
    }

    if (newSerialData)
    {
        uint32_t sensorNum;
        float conc;
        float temp;
        float rh;
        uint32_t conc_c;
        uint32_t temp_c;
        uint32_t rh_c;
        uint32_t days;
        uint32_t hours;
        uint32_t minutes;
        uint32_t seconds;
        int result = sscanf(serialData, "%lu, %f, %f, %f, %lu, %lu, %lu, %lu, %lu, %lu, %lu", &sensorNum, &conc, &temp, &rh, &conc_c, &temp_c, &rh_c, &days, &hours, &minutes, &seconds);
        if (result != EOF)
        {
            packet->has_co = true;
            packet->co = (uint32_t) (conc * 100);
            Log.info("readSensors(): Reading co value of %f, transmitting %ld", conc, packet->co);
        }
        else
        {
            Log.error("readSensors(): Could not interpret co value");
        }
        newSerialData = false;
    }
    else if (newEnergyMeterData)
    {
        StaticJsonDocument<ENERGY_METER_DATA_SIZE> doc;
        DeserializationError error = deserializeJson(doc, energyMeterData);

        if (error)
        {
            Log.warn("Unable to parse JSON...");
            Log.warn(error.c_str());
            // serialDeviceType = SERIAL_TYPE_CO;
            // Serial1.flush();
            // Serial1.begin(9600);
            Log.info("Attempting to detect CO device");
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
            packet->has_apparent_power = false;

            packet->reactive_power = doc["r"];
            packet->has_reactive_power = false;

            packet->power_factor = (float)doc["f"] * 100;
            packet->has_power_factor = false;
        }

        newEnergyMeterData = false;
    }

    if (resetReason != RESET_REASON_NONE)
    {
        packet->reset_reason = resetReason;
        packet->has_reset_reason = true;

        if (resetReason == RESET_REASON_PANIC)
        {
            uint32_t resetReasondata = System.resetReasonData();
            packet->reset_reason_data = resetReasondata;
            packet->has_reset_reason_data = true;
        }

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

void checkConnecting()
{
    if (connecting())
    {
        Log.info("Increasing connectingCounter: %ld", connectingCounter);
        connectingCounter++;
    }
    else
    {
        connectingCounter = 0;
    }
}

void resetDevice()
{
    Log.info("Rebooting coprocessor...");
    resetCoprocessor();

    Log.info("Rebooting myself...");
    System.reset();
}

void resetCoprocessor()
{
    if (serialDeviceType == SERIAL_TYPE_CO)
    {
        Serial1.print("R");
    }
    else
    {
        Serial1.printf("reset");
    }
    Serial1.flush();
}

int cloudReset(String arg)
{
    Serial.println("Cloud reset called...");
    resetTimer.start();
    return 0;
}

int cloudResetCoprocessor(String arg)
{
    Serial.println("Cloud reset coprocessor called...");
    resetCoprocessor();
    return 0;
}

int cloudUnackMeasurement(String arg)
{
    // TODO: Put code here...
    return 0;
}

int cloudParameters(String arg)
{
    bool settingValue = true;
    int32_t value = 0;
    uint8_t commandLength = 0;

    const char *argStr = arg.c_str();
    const char *command = argStr;

    // Look for equals sign
    char *loc = strchr(argStr, '=');

    if (loc == NULL)
    {
        // Since there is no equals sign, then we are getting a value, not setting
        settingValue = false;

        commandLength = arg.length();
    }
    else
    {
        // Since there is an equals sign, we are setting a value
        settingValue = true;

        const char *valueStr = loc + 1; // Skip sentinal character

        // Parse value
        value = atoi(valueStr);
        if (value == 0)
        {
            Log.error("Unable to parse value: %s", argStr);
            return -1;
        }

        commandLength = loc - command;
    }

    // Match command
    if (strncmp(command, "setupStatus", commandLength) == 0)
    {
        publishStatus = true;
        return 0;
    }
    if (strncmp(command, "readPeriodMs", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating readPeriodMs (%ld)", value);
            config.data.readPeriodMs = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.readPeriodMs;
        }
    }

    if (strncmp(command, "uploadPeriodMs", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating uploadPeriodMs (%ld)", value);
            config.data.uploadPeriodMs = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.uploadPeriodMs;
        }
    }

    if (strncmp(command, "printSysInfoMs", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating printSysInfoMs (%ld)", value);
            config.data.printSysInfoMs = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.printSysInfoMs;
        }
    }

    if (strncmp(command, "enablePrintSystemInfo", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating enablePrintSystemInfo (%ld)", value);
            // value can't be 0 so we add 1. This means 1 is false and 2 is true
            // To get it back into a real bool, we substract 1
            config.data.enablePrintSystemInfo = value - 1;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            // To stay consistent, we add 1
            return config.data.enablePrintSystemInfo + 1;
        }
    }

    if (strncmp(command, "uploadBatchSize", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating uploadBatchSize (%ld)", value);
            config.data.uploadBatchSize = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.uploadBatchSize;
        }
    }

    if (strncmp(command, "maxPubSize", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating maxPubSize (%ld)", value);
            config.data.maxPubSize = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.maxPubSize;
        }
    }

    if (strncmp(command, "delayBeforeReboot", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating delayBeforeReboot (%ld)", value);
            config.data.delayBeforeReboot = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.delayBeforeReboot;
        }
    }

    if (strncmp(command, "resetConfig", commandLength) == 0)
    {
        Log.info("Resetting configuration");
        config.reset();
        config.print();
        return 0;
    }

    if (strncmp(command, "scd30SetAltitude", commandLength) == 0)
    {
        Log.info("Setting altitude on SCD30");
        airSensor.setAltitudeCompensation(value);
        return 0;
    }

    if (strncmp(command, "scd30SetTemperatureOffset", commandLength) == 0)
    {
        Log.info("Setting temperature offset on SCD30");
        // Equivilant to airSensor.SetTemperatureOffset except that that method requires a float.
        // Rather than casting to a float and dividing by 100, just for that function to multiply
        // by 100 and cast back into a uint_16, I am calling the underlying method directly.
        airSensor.sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, value);
        return 0;
    }

    if (strncmp(command, "startNewAckTracker", commandLength) == 0)
    {
        Log.info("Renaming AckTracker file");
        return fileTracker.startNewFile() ? 0 : -1;
    }

    if (strncmp(command, "resetRTC", commandLength) == 0)
    {
        Log.info("Resetting RTC");
        rtcSet = false;
        return 0;
    }

    if (strncmp(command, "rtc", commandLength) == 0)
    {
        return rtc.now().unixtime();
    }

    if (strncmp(command, "heaterOnLengthSec", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating heaterOnLengthSec (%ld)", value);
            config.data.heaterOnLengthSec = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.heaterOnLengthSec;
        }
    }

    if (strncmp(command, "heaterOffLengthSec", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating heaterOffLengthSec (%ld)", value);
            config.data.heaterOffLengthSec = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.heaterOffLengthSec;
        }
    }

    if(strncmp(command, "countryVoltage", commandLength) == 0)
    {
        if (settingValue)
        {
            Log.info("Updating countryVoltage (%ld)", value);
            config.data.countryVoltage = value;
            config.save();
            config.print();
            return 0;
        }
        else
        {
            return config.data.countryVoltage;
        }
    }

    if (strncmp(command, "powerSource", commandLength) == 0)
    {
        #if PLATFORM_ID == PLATFORM_BORON // Only for Particle Boron microcontroller
        return DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE);
        #else
        Log.error("Non-Boron device cannot determine power source");
        return -1;
        #endif

    }

    Log.error("No matching command: %s", argStr);
    return -1;
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

    if (packet->has_co)
    {
        Log.info("\tCO: %ld", packet->co);
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