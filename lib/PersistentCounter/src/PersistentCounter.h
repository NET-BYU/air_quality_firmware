#ifndef PERSISTENT_ACK_TRACKER_H_
#define PERSISTENT_ACK_TRACKER_H_

#include <stdint.h>
#include <Particle.h>

class PersistentCounter
{
public:
    PersistentCounter(uint32_t address);
    uint32_t get();
    void increment();

private:
    void set(uint32_t value);

    Logger log;
    uint32_t address;
    uint32_t count;
};

#endif