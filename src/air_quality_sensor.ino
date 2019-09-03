#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "RTClibrary.h"
#include "sensor_packet.pb.h"
#include "SimpleAckTracker.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SPS30.h"
#include "ArduinoJson.h"
#include "PersistentCounter.h"
#include "PersistentConfig.h"
#include "SdFat.h"
#include "SdCardLogHandlerRK.h"

#define BOOT_LED D3
#define READ_LED D4
#define PUBLISH_LED D5

#if PLATFORM_ID == PLATFORM_ARGON
PRODUCT_ID(9901);
PRODUCT_VERSION(2);
#endif

#if PLATFORM_ID == PLATFORM_BORON
PRODUCT_ID(9861);
PRODUCT_VERSION(1);
#endif

// Counters
#define SEQUENCE_COUNT_ADDRESS 0x00
PersistentCounter sequence(SEQUENCE_COUNT_ADDRESS);

// Sensor configuration
#define CONFIG_ADDRESS 0x04
PersistentConfig config(CONFIG_ADDRESS);

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
bool rtcSet = true;

// Energy Meter Data
#define ENERGY_METER_DATA_SIZE 200
char energyMeterData[ENERGY_METER_DATA_SIZE];
bool newEnergyMeterData = false;

// Global variables to keep track of state
bool saveDataSucess = true;
int resetReason = RESET_REASON_NONE;

// Publishing information
particle::Future<bool> currentPublish;
bool currentlyPublishing = false;

// LEDs
auto bootLED = JLed(BOOT_LED);
auto readLED = JLed(READ_LED);
auto publishLED = JLed(PUBLISH_LED);

// Timers
bool readDataFlag = true;
Timer readTimer(config.data.readPeriodMs, []() { readDataFlag = true; });

bool uploadFlag = true;
Timer uploadTimer(config.data.uploadPeriodMs, []() { uploadFlag = true; });

bool printSystemInfoFlag = true;
Timer printSystemInfoTimer(config.data.printSysInfoMs, []() { printSystemInfoFlag = true; });

Timer resetTimer(config.data.delayBeforeReboot, resetDevice, true);

#define MAX_RECONNECT_COUNT 30
uint32_t connectingCounter = 0;
Timer connectingTimer(60000, checkConnecting);

// SD Card
SdFat sd;
const int SD_CHIP_SELECT = A5;

// Logging
Logger encodeLog("app.encode");
Logger serialLog("app.serial");

#define SD_LOGGING 0

#if SD_LOGGING
SdCardLogHandler<2048> sdLogHandler(sd, SD_CHIP_SELECT, SPI_FULL_SPEED, LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO}, {"app.encode", LOG_LEVEL_INFO}});
#else
SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO},
                                             {"app.encode", LOG_LEVEL_INFO}});
#endif

// Particle system stuff
SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

void setup()
{
    bool success = true;
    resetReason = System.resetReason();

    // Set up cloud functions
    Particle.function("reset", cloudReset);
    Particle.function("resetCo", cloudResetCoprocessor);
    Particle.function("unack", cloudUnackMeasurement);
    Particle.function("setParam", cloudSetParameter);

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
        success = false;
    }

    if (!airSensor.begin())
    {
        Log.error("Could not start CO2 sensor!");
        success = false;
    }

    if (!success)
    {
        bootLED.Blink(500, 500).Forever();
    }
    else
    {
        bootLED.Blink(1000, 1000);
    }

#if SD_LOGGING
    sdLogHandler.setup();
#endif

    // Start timers
    readTimer.start();
    uploadTimer.start();
    connectingTimer.start();

    if (config.data.enablePrintSystemInfo)
    {
        printSystemInfoTimer.start();
    }

    // Print out configuration information
    config.print();
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
            Log.error("Packing measurement FAILED!");
            saveDataSucess = false;
            readLED.Blink(250, 250).Forever();
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
        Log.trace("Trying to upload data... (%d, %d, %ld >= %ld (%d))",
                  !currentlyPublishing,
                  Particle.connected(),
                  tracker.unconfirmedCount(),
                  config.data.uploadBatchSize,
                  tracker.unconfirmedCount() >= config.data.uploadBatchSize);
        if (!currentlyPublishing && Particle.connected() && tracker.unconfirmedCount() >= config.data.uploadBatchSize)
        {
            // TODO: Should do the ceiling of the division just to be safe
            uint32_t maxLength = config.data.maxPubSize - (config.data.maxPubSize / 4); // Account for overhead of base85 encoding
            uint8_t data[maxLength];
            uint32_t dataLength;
            getMeasurements(data, maxLength, dataLength, pendingPublishes);

            uint8_t encodedData[config.data.maxPubSize];
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

void getMeasurements(uint8_t *data, uint32_t maxLength, uint32_t &length, uint8_t &count)
{
    Log.info("Max length: %ld", maxLength);
    uint32_t total = 0;
    count = 0;

    // Figure out how many measurements can fit in to buffer
    while (total < maxLength && count < tracker.unconfirmedCount())
    {
        total += tracker.getLengthOf(count);
        total += 1; // Include length byte
        count++;
    }

    if (total > maxLength)
    {
        // This means we more unconfirmed measurements than we have room for, so take one less
        count--;
        total -= tracker.getLengthOf(count);
        total -= 1; // Include length byte
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
    Log.info("Length of data: %ld", length);
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

    packet->sequence = sequence.get();

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
    Serial1.printf("reset");
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

int cloudSetParameter(String arg)
{
    const char *argStr = arg.c_str();
    char *loc = strchr(argStr, '|');

    if (loc == NULL)
    {
        Log.error("Incorrect format for set paramter: %s", argStr);
        return -1;
    }

    const char *command = argStr;
    const char *valueStr = loc + 1; // Skip sentinal character
    int size = loc - command;

    // Parse value
    int value = atoi(valueStr);
    if (value == 0)
    {
        Log.error("Unable to parse value: %s", argStr);
        return -1;
    }

    // Match command
    if (strncmp(command, "readPeriodMs", size) == 0)
    {
        Log.info("Updating readPeriodMs (%d)", value);
        config.data.readPeriodMs = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "uploadPeriodMs", size) == 0)
    {
        Log.info("Updating uploadPeriodMs (%d)", value);
        config.data.uploadPeriodMs = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "printSysInfoMs", size) == 0)
    {
        Log.info("Updating printSysInfoMs (%d)", value);
        config.data.printSysInfoMs = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "enablePrintSystemInfo", size) == 0)
    {
        Log.info("Updating enablePrintSystemInfo (%d)", value);
        config.data.enablePrintSystemInfo = value - 1;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "uploadBatchSize", size) == 0)
    {
        Log.info("Updating uploadBatchSize (%d)", value);
        config.data.uploadBatchSize = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "maxPubSize", size) == 0)
    {
        Log.info("Updating maxPubSize (%d)", value);
        config.data.maxPubSize = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "delayBeforeReboot", size) == 0)
    {
        Log.info("Updating delayBeforeReboot (%d)", value);
        config.data.delayBeforeReboot = value;
        config.save();
        config.print();
        return 0;
    }

    if (strncmp(command, "resetConfig", size) == 0)
    {
        Log.info("Resetting configuration");
        config.reset();
        config.print();
        return 0;
    }

    if (strncmp(command, "scd30SetAltitude", size) == 0)
    {
        Log.info("Setting altitude on SCD30");
        airSensor.setAltitudeCompensation(value);
        return 0;
    }

    if (strncmp(command, "scd30SetTemperatureOffset", size) == 0)
    {
        Log.info("Setting temperature offset on SCD30");
        // Equivilant to airSensor.SetTemperatureOffset except that that method requires a float.
        // Rather than casting to a float and dividing by 100, just for that function to multiply
        // by 100 and cast back into a uint_16, I am calling the underlying method directly.
        airSensor.sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, value);
        return 0;
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