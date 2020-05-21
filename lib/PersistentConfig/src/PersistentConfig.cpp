#include "PersistentConfig.h"

PersistentConfig::PersistentConfig(uint32_t address) : log("persistent_config"), address(address)
{
    log.trace("Address: %ld", address);
    log.trace("Size of structure: %d", sizeof(data));
    log.trace("EEPROM size: %d", EEPROM.length());
    if ((address + sizeof(data)) > EEPROM.length())
    {
        log.error("PersistentConfig does not fit into EEPROM at given address location.");
    }
    else
    {
        load();
    }
}

void PersistentConfig::load()
{
    EEPROM.get(address, data);

    // Check to see if memory has been set before
    if (data.version == 0xFFFFFFFF)
    {
        log.info("Config has not been initialized.");
        log.info("Using default values");
        data = defaultConfig;

        save();
    }

    // Migrate from v1 to v2
    else if (data.version == 1)
    {
        data.heaterOnLengthSec = defaultConfig.heaterOnLengthSec;
        data.heaterOffLengthSec = defaultConfig.heaterOffLengthSec;
        data.countryVoltage = defaultConfig.countryVoltage;
        data.heaterPowerFactor = defaultConfig.heaterPowerFactor;
        data.version = 2;

        save();
    }
}

void PersistentConfig::save()
{
    EEPROM.put(address, data);
}

void PersistentConfig::reset()
{
    data = defaultConfig;
    save();
}

void PersistentConfig::print()
{
    Log.info("~~~~~~~ Configuration ~~~~~~~");
    Log.info("\tversion: %ld", data.version);
    Log.info("\treadPeriodMs: %ld", data.readPeriodMs);
    Log.info("\tuploadPeriodMs: %ld", data.uploadPeriodMs);
    Log.info("\tprintSysInfoMs: %ld", data.printSysInfoMs);
    Log.info("\tenablePrintSystemInfo: %ld", data.enablePrintSystemInfo);
    Log.info("\tuploadBatchSize: %ld", data.uploadBatchSize);
    Log.info("\tmaxPubSize: %ld", data.maxPubSize);
    Log.info("\tdelayBeforeReboot: %ld", data.delayBeforeReboot);
    Log.info("\heaterOnLengthSec: %ld", data.heaterOnLengthSec);
    Log.info("\heaterOffLengthSec: %ld", data.heaterOffLengthSec);
    Log.info("\countryVoltage: %ld", data.countryVoltage);
    Log.info("\heaterPowerFactor: %ld", data.heaterPowerFactor);
    Log.info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}