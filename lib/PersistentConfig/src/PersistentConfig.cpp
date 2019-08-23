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
        data = v1Default;
        save();
        return;
    }

    // TODO: Check to see if we need to migrate to a newer version
}

void PersistentConfig::save()
{
    EEPROM.put(address, data);
}

void PersistentConfig::reset()
{
    data = v1Default;
    save();
}