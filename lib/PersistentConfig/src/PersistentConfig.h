#ifndef PERSISTENT_CONFIG_H_
#define PERSISTENT_CONFIG_H_

#include <stdint.h>
#include <Particle.h>

struct v1
{
    uint32_t version;
    uint32_t readPeriodMs;
    uint32_t uploadPeriodMs;
    uint32_t printSysInfoMs;
    uint32_t enablePrintSystemInfo;
    uint32_t uploadBatchSize;
    uint32_t maxPubSize;
    uint32_t delayBeforeReboot;
    uint32_t heaterOnLengthSec;
    uint32_t heaterOffLengthSec;
};
typedef struct v1 Config;

class PersistentConfig
{
public:
    PersistentConfig(uint32_t address);
    void save();

    void reset();

    void print();

    Config data;

private:
    Logger log;
    uint32_t address;
    void load();

    Config v1Default = {1, 60000, 10000, 10000, 0, 10, 620, 2000, 0, 0};
};

#endif