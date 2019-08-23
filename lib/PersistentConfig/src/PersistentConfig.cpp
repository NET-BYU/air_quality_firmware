#include "PersistentConfig.h"



PersistentConfig::PersistentConfig(uint32_t address) : log("persistent_config"), address(address)
{
    if (address + sizeof(data) > EEPROM.length()) {
        log.error("PersistentConfig does not fit into EEPROM at given address location.");
    }
    else {
        load();
    }
}

void PersistentConfig::load()
{
    EEPROM.get(address, data);
    
    if (data.version == 0xFF) {
        log.info("Config has not been initialized.");
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