#include "AckTracker.h"
#include "DiagnosticsHelperRK.h"
#include "FileAckTracker.h"
#include "MemoryAckTracker.h"
#include "PersistentConfig.h"
#include "PersistentCounter.h"
#include "SdCardLogHandlerRK.h"
#include "SdFat.h"
#include "Sensors.h"
#include "base85.h"
#include "jled.h"
#include "pb_encode.h"
#include "sensor_packet.pb.h"

#include "ArduinoJson.h"

#if PLATFORM_ID == PLATFORM_ARGON
PRODUCT_ID(9901);
PRODUCT_VERSION(3);
#endif

#if PLATFORM_ID == PLATFORM_BORON
PRODUCT_ID(9861);
PRODUCT_VERSION(4);
#endif

// Battery Stuff
#define BATTERY_POWER_PIN D2 // Controls the relay and boost converter for the battery
#define BATTERY_ON_OUT HIGH  // Write to relay to turn on boost converter
#define BATTERY_OFF_OUT LOW  // Opposite of above
#define POWER_SOURCE_BATTERY                                                                       \
    5 // Value returned from diag helper if the system is powered by battery
bool poweringFromBattery = false;

// Counters
#define SEQUENCE_COUNT_ADDRESS 0x00
PersistentCounter sequence(SEQUENCE_COUNT_ADDRESS);

// Sensor configuration
#define CONFIG_ADDRESS 0x04
PersistentConfig config(CONFIG_ADDRESS);

// SD Card
SdFat sd;
const int SD_CHIP_SELECT = A5; // Pin on sd card reader, needs to be passed to SDFat Lib

// Write data points to SD card and keep track of what has been ackowledged
#define ENTRY_SIZE 300
FileAckTracker fileTracker(sd, SD_CHIP_SELECT, ENTRY_SIZE);
MemoryAckTracker<ENTRY_SIZE, 60> memoryTracker;
AckTracker *currentTracker;
uint32_t pendingPublishes = 0;
bool trackerSetup = true;

// Power Management IC
#if PLATFORM_ID == PLATFORM_BORON
PMIC pmic;
#endif

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
Timer readTimer(config.data.readPeriodMs,
                []() { readDataFlag = true; }); // How often the system reads from the sensors

bool uploadFlag = true;
Timer uploadTimer(config.data.uploadPeriodMs, []() {
    uploadFlag = true;
}); // How often the info is uploaded to the particle cloud --> google cloud --> MQTT --> db

bool printSystemInfoFlag = true;
Timer printSystemInfoTimer(config.data.printSysInfoMs, []() {
    printSystemInfoFlag = true;
}); // Periodically write data usage in the logs

Timer resetTimer(config.data.delayBeforeReboot, resetDevice,
                 true); // A small delay between calling reset and actually resetting

bool updateRTCFlag = false;
Timer updateRtcTimer(3600000, []() { updateRTCFlag = true; });

// bool handleHeaterFlag = true;
// Timer traceHeaterTimer(TRACE_HEATER_TIMER_PERIOD, []() { handleHeaterFlag = true; });

#define MAX_RECONNECT_COUNT 30
uint32_t connectingCounter = 0;
Timer connectingTimer(60000,
                      checkConnecting); // Time it will try to connect before reseting the device

#define LENGTH_HEADER_SIZE 2

// Logging
Logger encodeLog("app.encode");
Logger serialLog("app.serial");
Logger csvLog("app.csv");

SdCardLogHandler<2048> sdLogHandler(sd, SD_CHIP_SELECT, SPI_FULL_SPEED, LOG_LEVEL_WARN,
                                    {{"app", LOG_LEVEL_INFO},
                                     {"app.Sensor", LOG_LEVEL_INFO},
                                     {"app.encode", LOG_LEVEL_INFO},
                                     {"app.csv", LOG_LEVEL_NONE},
                                     {"FileAckTracker", LOG_LEVEL_INFO},
                                     {"MemoryAckTracker", LOG_LEVEL_TRACE},
                                     {"TraceHeater", LOG_LEVEL_INFO}});

SdCardLogHandler<2048> csvLogHandler(sd, SD_CHIP_SELECT, SPI_FULL_SPEED, LOG_LEVEL_NONE,
                                     {{"app.csv", LOG_LEVEL_INFO}});
STARTUP(csvLogHandler.withDesiredFileSize(1000000UL)
            .withNoSerialLogging()
            .withMaxFilesToKeep(1000)
            .withLogsDirName("CSV"));

STARTUP(sdLogHandler.withMaxFilesToKeep(1000));

// Sensors
Sensors *Sensors::instance = 0;
Sensors *allSensors = allSensors->getInstance();

// Particle system stuff
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

void setup() {
    // Set up cloud functions
    Particle.function("reset", cloudReset);
    // Particle.function("unack", cloudUnackMeasurement);
    Particle.function("param", cloudParameters);

    allSensors->setup(&config);

    // Debugging port
    Serial.begin(9600);

    pinMode(BATTERY_POWER_PIN, OUTPUT);
    digitalWrite(BATTERY_POWER_PIN, BATTERY_OFF_OUT);
    poweringFromBattery = false;

    if (!fileTracker.begin()) {
        Log.error("Could not start file tracker. Using memory tracker");
        trackerSetup = false;
        currentTracker = &memoryTracker;
    } else {
        Log.info("Using file tracker.");
        currentTracker = &fileTracker;
    }

    delay(1000);

#if PLATFORM_ID == PLATFORM_BORON
    pmic.begin();
#endif

    sdLogHandler.setup();
    csvLogHandler.setup();
    csvLog.print(
        "Timestamp,Sequence,Temperature,Humidity,RTC Temperature,PM1,PM2.5,PM4,PM10,SD Card "
        "Present,Queue Size,Battery Charge,CO2,CO,Voltage,Current,Energy,Power,Apparent "
        "Power,Reactive "
        "Power,Power Factor,Free Memory,Reset Reason,Estimated Temperature,Internal Temperature\n");

    // Start timers
    readTimer.start();
    uploadTimer.start();
    connectingTimer.start();
    updateRtcTimer.start();
    // traceHeaterTimer.start();

    if (config.data.enablePrintSystemInfo) {
        printSystemInfoTimer.start();
    }

    // Print out configuration information
    config.print();

    Particle.connect();
}

void loop() // Print out RTC status in loop
{
    allSensors->isCOSetup();
    // Read sensor task
    if (readDataFlag) {
        sensorLed.Blink(1000, 1000).Update();
        Log.info("Reading sensors...");
        SensorPacket packet = SensorPacket_init_zero;
        readSensors(&packet);
        csvLogPacket(&packet);
        printPacket(&packet);

        Log.info("Putting data into a protobuf...");
        uint8_t data[SensorPacket_size];
        uint8_t length;

        if (packMeasurement(&packet, SensorPacket_size, data, &length)) {
            Log.info("Adding data to tracker (%d)...", length);
            AckTracker *tracker = getAckTrackerForWriting();
            if (tracker->add(packet.sequence, length, data)) {
                if (tracker == &fileTracker) {
                    sdLed.Blink(1000, 1000).Update();
                } else {
                    sdLed.Blink(400, 100).Repeat(5);
                }
            } else {
                Log.warn("Failed to add data to tracker");
                if (tracker == &fileTracker) {
                    Log.warn("Switching to MemoryAckTracker");
                    tracker = &memoryTracker;
                    if (tracker->add(packet.sequence, length, data)) {
                        Log.info("Data was successfully added to MemoryAckTracker");
                        sdLed.Blink(400, 100).Repeat(5);
                    } else {
                        Log.warn("Failed to add data to memoryTracker");
                        sdLed.Blink(100, 100).Forever();
                    }
                } else {
                    sdLed.Blink(400, 400).Forever();
                }
            }
        } else {
            Log.error("Packing measurement FAILED!");
            sdLed.Blink(1600, 1600).Forever();
        }

        sequence.increment();
        readDataFlag = false;
    }

    // Check on message that is currently being published
    if (currentlyPublishing && currentPublish.isDone()) {
        if (currentPublish.isSucceeded()) {
            Log.info("Publication was successful!");
            if (publishingStatus) {
                Log.info("Published status message");
            } else {
                AckTracker *tracker = getAckTrackerForReading();
                tracker->confirmNext(pendingPublishes);
                uint32_t unconfirmedCount;
                if (tracker->unconfirmedCount(&unconfirmedCount)) {
                    Log.info("Unconfirmed count: %ld", unconfirmedCount);
                } else {
                    Log.error("Unable to get unconfirmed count");
                }
            }

            cloudLed.Off().Update();
        } else {
            Log.warn("Publication was NOT successful!");
            cloudLed.Blink(250, 250).Forever();
        }

        pendingPublishes = 0;
        currentlyPublishing = false;
        publishingStatus = false;
    }

    if (publishStatus && !currentlyPublishing && Particle.connected()) {
        StaticJsonDocument<200> doc;
        doc["tracker"] = trackerSetup;
        doc["rtc"] = allSensors->getRTCPresent();
        doc["pm"] = allSensors->getPmSensorSetup();
        doc["air"] = allSensors->getAirSensorSetup();
        doc["trace_heater"]["state"] = (int)allSensors->traceHeater.state;
        doc["trace_heater"]["T_H"] = allSensors->traceHeater.T_H;
        doc["trace_heater"]["T_C"] = allSensors->traceHeater.T_C;
        doc["trace_heater"]["current_temp"] = allSensors->traceHeater.internal_temperature;
        doc["trace_heater"]["estimate"] = allSensors->traceHeater.getEstimatedTemperature();

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
    if (uploadFlag) {
        AckTracker *tracker = getAckTrackerForReading();
        uint32_t unconfirmedCount = 0;
        tracker->unconfirmedCount(&unconfirmedCount);

        Log.trace("Trying to upload data... (%d, %d, %ld >= %ld (%d))", !currentlyPublishing,
                  Particle.connected(), unconfirmedCount, config.data.uploadBatchSize,
                  unconfirmedCount >= config.data.uploadBatchSize);
        if (!currentlyPublishing && Particle.connected() &&
            (unconfirmedCount >= config.data.uploadBatchSize)) {
            // TODO: Should do the ceiling of the division just to be safe
            uint32_t maxLength =
                config.data.maxPubSize -
                (config.data.maxPubSize / 4); // Account for overhead of base85 encoding
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

    if (updateRTCFlag) {
        allSensors->setRTCPresent(allSensors->isRTCPresent());
        allSensors->setRTCSet(false); // Allow RTC to sync with time from cloud
        updateRTCFlag = false;
    }

    // Update RTC if needed
    if (allSensors->getRTCPresent() && !allSensors->getRTCSet() && Particle.connected() &&
        Time.isValid()) {
        Log.info("Setting clock...");
        allSensors->rtc.adjust(DateTime(Time.now()));
        allSensors->setRTCSet(true);

        delay(500);

        DateTime now = allSensors->rtc.now();
        uint32_t timestamp = now.unixtime();
        Log.info("Time is set to: %ld", timestamp);
    }

    if (config.data.traceHeaterEnabled && (counter % 500 == 0)) {
        allSensors->traceHeater.trace_heater_loop();
    }
    counter++;

    // Battery detection and handling, for keeping 5V sensors on with 3V battery
#if PLATFORM_ID == PLATFORM_BORON // Only for Particle Boron microcontroller
    if (DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE) == POWER_SOURCE_BATTERY) {
        if (!poweringFromBattery) {
            Log.info("Being powered by battery! Turning on boost converter.");
            digitalWrite(BATTERY_POWER_PIN, BATTERY_ON_OUT);
            poweringFromBattery = true;
        }
    } else if (poweringFromBattery) {
        Log.info("Being powered by wall! Turning off boost converter!");
        digitalWrite(BATTERY_POWER_PIN, BATTERY_OFF_OUT);
        poweringFromBattery = false;
    }
#endif

    if (connectingCounter >= MAX_RECONNECT_COUNT) {
        Log.warn("Rebooting myself because I've been connecting for too long.");
        System.reset();
    }

    if (printSystemInfoFlag) {
        printSystemInfo();
        printSystemInfoFlag = false;
    }

    sdLogHandler.loop();
    csvLogHandler.loop();

    // Update LEDs
    sensorLed.Update();
    sdLed.Update();
    cloudLed.Update();
}

AckTracker *getAckTrackerForWriting() {
    Log.info("Getting AckTracker for writing...");
    if (fileTracker.begin()) {
        Log.info("Using fileTracker");
        return &fileTracker;
    } else {
        Log.info("Using memoryTracker");
        return &memoryTracker;
    }
}

AckTracker *getAckTrackerForReading() {
    Log.info("Getting AckTracker for reading...");
    if (currentlyPublishing) {
        Log.info("Currently publishing so keep the same AckTracker.");
        return currentTracker;
    }

    if (!fileTracker.begin()) {
        Log.info("Unable to start fileTracker so using memoryTracker.");
        currentTracker = &memoryTracker;
        return currentTracker;
    }

    Log.info("Using fileTracker");
    currentTracker = &fileTracker;

    uint32_t memoryUnconfirmedCount = 0;
    memoryTracker.unconfirmedCount(&memoryUnconfirmedCount);

    if (memoryUnconfirmedCount == 0) {
        return currentTracker;
    }

    Log.info("Copying over %lu entries from memoryTracker to fileTracker", memoryUnconfirmedCount);
    uint32_t id;
    uint16_t length;
    uint8_t data[ENTRY_SIZE];
    for (uint32_t i = 0; i < memoryUnconfirmedCount; i++) {
        if (!memoryTracker.get(0, &id, &length, data)) {
            Log.error("Unable to get %lu entry from memoryTracker", i);
            break;
        }

        if (fileTracker.add(id, length, data)) {
            Log.info("Copied over an entry");
            memoryTracker.confirmNext(1);
        } else {
            Log.info("Unable to add entry to fileTracker");
            break;
        }
    }

    return currentTracker;
}

void serialEvent1() {
    serialLog.info("SerialEvent1!");
    size_t numBytes = Serial1.readBytesUntil('\n', allSensors->getSerialData(), SERIAL_DATA_SIZE);
    allSensors->addNullTermSerialData(numBytes);
    // serialLog.info("Serial Data received: %s\n", serialData);
    allSensors->setNewSerialData(true);
}

bool connecting() {
#if Wiring_WiFi
    return WiFi.connecting() || !Particle.connected();
#elif Wiring_Cellular
    return Cellular.connecting() || !Particle.connected();
#else
    return false;
#endif
}

void getMeasurements(uint8_t *data, uint32_t maxLength, uint32_t *length, uint32_t *count,
                     AckTracker *tracker) {
    Log.info("Max length: %ld", maxLength);
    uint32_t total = 0;
    uint16_t item_length = 0;
    uint32_t tmpCount = 0;

    // Figure out how many measurements can fit in to buffer
    uint32_t unconfirmedCount = 0;
    tracker->unconfirmedCount(&unconfirmedCount);
    while (total < maxLength && tmpCount < unconfirmedCount) {

        tracker->getLength(tmpCount, &item_length);
        Log.info("Getting length of item %ld: %d", tmpCount, item_length);
        total += item_length + LENGTH_HEADER_SIZE;
        tmpCount++;
    }

    if (total > maxLength) {
        // This means we have more unconfirmed measurements than we have room for
        // We need to remove the last added measurement because its what put us over the edge
        tmpCount--;
        tracker->getLength(tmpCount, &item_length);
        Log.info("Beyond max size (%ld > %ld), going back one measurement: %d.", total, maxLength,
                 item_length);
        total -= item_length + LENGTH_HEADER_SIZE;
    }

    Log.info("Number of measurements: %ld (%ld)\n", tmpCount, total);

    // Copy over data into data buffers
    uint32_t offset = 0;
    for (uint32_t i = 0; i < tmpCount; i++) {
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

void readQueue(SensorPacket *packet) {
    uint32_t unconfirmedCount;
    if (currentTracker->unconfirmedCount(&unconfirmedCount)) {
        packet->queue_size = unconfirmedCount;
        packet->has_queue_size = true;
    }
}

void readSensors(SensorPacket *packet) {
    allSensors->read(packet, &config);
    packet->sequence = sequence.get();
    packet->card_present = currentTracker == &fileTracker;
    packet->has_card_present = true;
    readQueue(packet);
#if PLATFORM_ID == PLATFORM_BORON
    Log.info("readSensors(): InputSourceRegister=0x%x", pmic.readInputSourceRegister());
#endif
}

bool packMeasurement(SensorPacket *inPacket, uint32_t inLength, uint8_t *out, uint8_t *outLength) {
    pb_ostream_t stream = pb_ostream_from_buffer(out, inLength);

    if (!pb_encode(&stream, SensorPacket_fields, inPacket)) {
        encodeLog.error("Unable to encode data into protobuffer");
        return false;
    }

    *outLength = stream.bytes_written;
    encodeLog.trace("Bytes written to protobuffer: %d", stream.bytes_written);
    return true;
}

bool encodeMeasurements(uint8_t *in, uint32_t inLength, uint8_t *out, uint32_t *outLength) {
    char *end_ptr = bintob85((char *)out, in, inLength);
    *outLength = end_ptr - (char *)out;

    encodeLog.trace("Added bytes for encoding: %ld", *outLength - inLength);

    if (encodeLog.isTraceEnabled()) {
        for (unsigned int i = 0; i < inLength; i++) {
            Serial.printf("%02x ", in[i]);
        }
        Serial.println();

        for (unsigned int i = 0; i < *outLength; i++) {
            Serial.printf("%02x ", out[i]);
        }
        Serial.println();
    }

    return true;
}

void checkConnecting() {
    if (connecting()) {
        Log.info("Increasing connectingCounter: %ld", connectingCounter);
        connectingCounter++;
    } else {
        connectingCounter = 0;
    }
}

void resetDevice() {
    Log.info("Rebooting myself...");
    System.reset();
}

int cloudReset(String arg) {
    Serial.println("Cloud reset called...");
    resetTimer.start();
    return 0;
}

// int cloudUnackMeasurement(String arg) {
//     // TODO: Put code here...
//     return 0;
// }

int cloudCommand(const char *cloudCommand, int32_t value, bool settingValue, uint32_t &configVal) {
    if (settingValue) {
        Log.info("Updating %s (%ld)", cloudCommand, value);
        configVal = value;
        config.save();
        config.print();
        return 0;
    } else {
        return configVal;
    }
}

int cloudParameters(String arg) {
    bool settingValue = true;
    int32_t value = 0;
    uint8_t commandLength = 0;

    const char *argStr = arg.c_str();
    const char *command = argStr;

    // Look for equals sign
    char *loc = strchr(argStr, '=');

    if (loc == NULL) {
        // Since there is no equals sign, then we are getting a value, not setting
        settingValue = false;

        commandLength = arg.length();
    } else {
        // Since there is an equals sign, we are setting a value
        settingValue = true;

        const char *valueStr = loc + 1; // Skip sentinal character

        // Parse value
        value = atoi(valueStr);
        if (value == 0) {
            Log.error("Unable to parse value: %s", argStr);
            return -1;
        }

        commandLength = loc - command;
    }

    // Match command
    if (strncmp(command, "setupStatus", commandLength) == 0) {
        publishStatus = true;
        return 0;
    }

    if (strncmp(command, "readPeriodMs", commandLength) == 0) {
        return cloudCommand("readPeriodMs", value, settingValue, config.data.readPeriodMs);
    }

    if (strncmp(command, "uploadPeriodMs", commandLength) == 0) {
        return cloudCommand("uploadPeriodMs", value, settingValue, config.data.uploadPeriodMs);
    }

    if (strncmp(command, "printSysInfoMs", commandLength) == 0) {
        return cloudCommand("printSysInfoMs", value, settingValue, config.data.printSysInfoMs);
    }

    if (strncmp(command, "enablePrintSystemInfo", commandLength) == 0) {
        if (settingValue) {
            Log.info("Updating enablePrintSystemInfo (%ld)", value);
            // value can't be 0 so we add 1. This means 1 is false and 2 is true
            // To get it back into a real bool, we substract 1
            config.data.enablePrintSystemInfo = value - 1;
            config.save();
            config.print();
            return 0;
        } else {
            // To stay consistent, we add 1
            return config.data.enablePrintSystemInfo + 1;
        }
    }

    if (strncmp(command, "uploadBatchSize", commandLength) == 0) {
        return cloudCommand("uploadBatchSize", value, settingValue, config.data.uploadBatchSize);
    }

    if (strncmp(command, "maxPubSize", commandLength) == 0) {
        return cloudCommand("maxPubSize", value, settingValue, config.data.maxPubSize);
    }

    if (strncmp(command, "delayBeforeReboot", commandLength) == 0) {
        return cloudCommand("delayBeforeReboot", value, settingValue,
                            config.data.delayBeforeReboot);
    }

    if (strncmp(command, "resetConfig", commandLength) == 0) {
        Log.info("Resetting configuration");
        config.reset();
        config.print();
        return 0;
    }

    if (strncmp(command, "scd30SetAltitude", commandLength) == 0) {
        Log.info("Setting altitude on SCD30");
        allSensors->airSensor.setAltitudeCompensation(value);
        return 0;
    }

    if (strncmp(command, "scd30SetTemperatureOffset", commandLength) == 0) {
        Log.info("Setting temperature offset on SCD30");
        // Equivilant to airSensor.SetTemperatureOffset except that that method requires a float.
        // Rather than casting to a float and dividing by 100, just for that function to multiply
        // by 100 and cast back into a uint_16, I am calling the underlying method directly.
        allSensors->airSensor.sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, value);
        return 0;
    }

    if (strncmp(command, "startNewAckTracker", commandLength) == 0) {
        Log.info("Renaming AckTracker file");
        return fileTracker.startNewFile() ? 0 : -1;
    }

    if (strncmp(command, "resetRTC", commandLength) == 0) {
        Log.info("Resetting RTC");
        allSensors->setRTCSet(false);
        return 0;
    }

    if (strncmp(command, "rtc", commandLength) == 0) {
        return allSensors->rtc.now().unixtime();
    }

    if (strncmp(command, "particleTime", commandLength) == 0) {
        return Time.now();
    }

    if (strncmp(command, "resetHeater", commandLength) == 0) {
        Log.info("Resetting Trace Heater");
        allSensors->traceHeater.reset();
        return 0;
    }

    if (strncmp(command, "traceHeaterEnabled", commandLength) == 0) {
        return cloudCommand("traceHeaterEnabled", (value == 1), settingValue,
                            config.data.traceHeaterEnabled);
    }

    if (strncmp(command, "countryVoltage", commandLength) == 0) {
        return cloudCommand("countryVoltage", value, settingValue, config.data.countryVoltage);
    }

    if (strncmp(command, "heaterPowerFactor", commandLength) == 0) {
        return cloudCommand("heaterPowerFactor", value, settingValue,
                            config.data.heaterPowerFactor);
    }

    if (strncmp(command, "powerSource", commandLength) == 0) {
#if PLATFORM_ID == PLATFORM_BORON // Only for Particle Boron microcontroller
        return DiagnosticsHelper::getValue(DIAG_ID_SYSTEM_POWER_SOURCE);
#else
        Log.error("Non-Boron device cannot determine power source");
        return -1;
#endif
    }

    if (strncmp(command, "zeroCO", commandLength) == 0) {
        Serial1.write("Z");
        Log.info("Zero-ing CO sensor");
        Serial1.flush();
        return 0;
    }

    if (strncmp(command, "boardTimeConstant", commandLength) == 0) {
        if (settingValue) {
            Log.info("Updating board time constant");
            config.data.boardTimeConstant = value;
            config.save();
            config.print();
            return 0;
        } else {
            return config.data.boardTimeConstant;
        }
    }

    Log.error("No matching command: %s", argStr);
    return -1;
}

void printSystemInfo() {
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

#define MAX_ATTR_LEN 20
void csvLogToFile(bool has_attr, int32_t reading) {
    char buffSize[MAX_ATTR_LEN];
    if (has_attr) {
        snprintf(buffSize, MAX_ATTR_LEN, "%ld", reading);
        csvLog.print(buffSize);
    }
    csvLog.print(",");
}

void csvLogPacket(SensorPacket *packet) {
    csvLogToFile(true, packet->timestamp);
    csvLogToFile(true, packet->sequence);
    csvLogToFile(packet->has_temperature, packet->temperature);
    csvLogToFile(packet->has_temperature, packet->humidity);
    csvLogToFile(packet->has_rtc_temperature, packet->rtc_temperature);
    csvLogToFile(packet->has_pm1, packet->pm1);
    csvLogToFile(packet->has_pm2_5, packet->pm2_5);
    csvLogToFile(packet->has_pm4, packet->pm4);
    csvLogToFile(packet->has_pm10, packet->pm10);
    csvLogToFile(packet->has_card_present, (int32_t)packet->card_present);
    csvLogToFile(packet->has_queue_size, packet->queue_size);
    csvLogToFile(packet->has_battery_charge, packet->battery_charge);
    csvLogToFile(packet->has_co2, packet->co2);
    csvLogToFile(packet->has_co, packet->co);
    csvLogToFile(packet->has_voltage, packet->voltage);
    csvLogToFile(packet->has_current, packet->current);
    csvLogToFile(packet->has_total_energy, packet->total_energy);
    csvLogToFile(packet->has_power, packet->power);
    csvLogToFile(packet->has_apparent_power, packet->apparent_power);
    csvLogToFile(packet->has_reactive_power, packet->reactive_power);
    csvLogToFile(packet->has_power_factor, packet->power_factor);
    csvLogToFile(packet->has_free_memory, packet->free_memory);
    csvLogToFile(packet->has_reset_reason, packet->reset_reason);
    csvLogToFile(packet->has_estimated_temperature, packet->estimated_temperature);
    csvLogToFile(packet->has_internal_temperature, packet->internal_temperature);
    csvLog.print("\n");
}

void printPacketToLog(const char *attr_name, bool has_attr, int32_t reading) {
    if (has_attr) {
        Log.info("\t%s: %ld", attr_name, reading);
    }
}

void printPacket(SensorPacket *packet) {
    Log.info("Packet:");
    printPacketToLog("Timestamp", true, packet->timestamp);
    printPacketToLog("Sequence", true, packet->sequence);
    printPacketToLog("Temperature", packet->has_temperature, packet->temperature);
    printPacketToLog("Humidity", packet->has_temperature, packet->humidity);
    printPacketToLog("RTC temperature", packet->has_rtc_temperature, packet->rtc_temperature);
    printPacketToLog("PM1", packet->has_pm1, packet->pm1);
    printPacketToLog("PM2.5", packet->has_pm2_5, packet->pm2_5);
    printPacketToLog("PM4", packet->has_pm4, packet->pm4);
    printPacketToLog("PM10", packet->has_pm10, packet->pm10);
    printPacketToLog("Card present", packet->has_card_present, packet->card_present);
    printPacketToLog("Queue size", packet->has_queue_size, packet->queue_size);
    printPacketToLog("Battery Charge (%)", packet->has_battery_charge, packet->battery_charge);
    printPacketToLog("CO2", packet->has_co2, packet->co2);
    printPacketToLog("CO", packet->has_co, packet->co);
    printPacketToLog("Voltage", packet->has_voltage, packet->voltage);
    printPacketToLog("Current", packet->has_current, packet->current);
    printPacketToLog("Total energy", packet->has_total_energy, packet->total_energy);
    printPacketToLog("Power", packet->has_power, packet->power);
    printPacketToLog("Apparent power", packet->has_apparent_power, packet->apparent_power);
    printPacketToLog("Reactive power", packet->has_reactive_power, packet->reactive_power);
    printPacketToLog("Power factor", packet->has_power_factor, packet->power_factor);
    printPacketToLog("Free Memory", packet->has_free_memory, packet->free_memory);
    printPacketToLog("Reset Reason", packet->has_reset_reason, packet->reset_reason);
    printPacketToLog("Estimated Temperature", packet->has_estimated_temperature,
                     packet->estimated_temperature);
    printPacketToLog("Internal Temperature", packet->has_internal_temperature,
                     packet->internal_temperature);
}
