#ifndef PERSISTENT_ACK_TRACKER_H_
#define PERSISTENT_ACK_TRACKER_H_

#include <stdint.h>
#include <Particle.h>

class PersistentCounter
{
public:
    PersistentCounter(uint32_t address);
    void set(uint32_t value);
    uint32_t get();
    void increment();

private:
    uint32_t count;
    uint32_t address;
    Logger log;
};

#endif