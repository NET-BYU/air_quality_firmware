#include "PersistentCounter.h"

PersistentCounter::PersistentCounter(uint32_t address)
    : log("persistent_counter"), address(address) {
    log.info("Loading count from %ld...", address);
    EEPROM.get(address, count);

    log.info("Count value: %ld", count);
    if (count == 0xFFFFFFFF) {
        log.info("Count has not been initialized. Setting count to 0.");
        set(0);
    }
}

uint32_t PersistentCounter::get() { return count; }

void PersistentCounter::set(uint32_t value) {
    log.info("Set value to %ld", value);
    count = value;
    EEPROM.put(address, count);
}

void PersistentCounter::increment() {
    log.info("Incrementing counter...");
    set(count + 1);
}
