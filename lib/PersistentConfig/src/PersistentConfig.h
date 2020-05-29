#ifndef PERSISTENT_CONFIG_H_
#define PERSISTENT_CONFIG_H_

#include <Particle.h>
#include <stdint.h>

struct config {
    uint32_t version;

    // v1
    uint32_t readPeriodMs;
    uint32_t uploadPeriodMs;
    uint32_t printSysInfoMs;
    uint32_t enablePrintSystemInfo;
    uint32_t uploadBatchSize;
    uint32_t maxPubSize;
    uint32_t delayBeforeReboot;

    // v2
    uint32_t countryVoltage;
    uint32_t heaterPowerFactor;

    // v3
    uint32_t traceHeaterEnabled;
    uint32_t boardTimeConstant;
};
typedef struct config Config;

class PersistentConfig {
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

    Config defaultConfig = {2, 60000, 10000, 10000, 0, 10, 620, 2000, 120, 990, 0, 1061};
};

#endif