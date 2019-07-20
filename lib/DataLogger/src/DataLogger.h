#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "SdFat.h"

using namespace std;

class DataLogger
{
public:
    DataLogger(uint8_t csPin = SS, uint8_t cdPin = D4, SPISettings spiSettings = SPI_FULL_SPEED,
               String file_name = "data.csv", uint32_t timeout = 20000);
    bool begin();
    bool read();
    bool write(String data);

private:
    bool cardPresent();
    bool waitForCardPresent();

    uint8_t cdPin;
    uint8_t csPin;
    SPISettings spiSettings;
    String file_name;
    uint32_t timeout;

    SdFat sd;
    File file;

    bool card_init;
};

#endif // DATALOGGER_H