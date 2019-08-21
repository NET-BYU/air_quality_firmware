#include "PersistentCounter.h"

PersistentCounter::PersistentCounter(uint32_t address) : log("persistent_counter")
{
    PersistentCounter::address = address;

    log.info("Loading count from %d...", address);
    EEPROM.get(address, count);

    log.info("Count value: %d", count);
    if (count == 0xFFFFFFFF)
    {
        // The value is not initialized
        count = 0;
        log.info("Count has not been initialized. Setting count to 0");
    }
}

void PersistentCounter::increment()
{
    count++;
    EEPROM.put(address, count);
    log.info("Increment counter...");
}

uint32_t PersistentCounter::get()
{
    return count;
}