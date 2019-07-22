#include "DataLogger.h"

DataLogger::DataLogger(uint8_t csPin, SPISettings spiSettings, String file_name,
                       uint32_t timeout)
{
    DataLogger::csPin = csPin;
    DataLogger::spiSettings = spiSettings;
    DataLogger::file_name = file_name;
    DataLogger::timeout = timeout;

    DataLogger::card_init = false;
}

bool DataLogger::read()
{
    return false;
}

bool DataLogger::write(String data)
{

    Serial.print("Starting card...");
    if (!sd.begin(csPin, spiSettings))
    {
        Serial.println("FAILED.");
        return false;
    }
    else
    {
        Serial.println("done.");
    }

    file = sd.open(file_name, FILE_WRITE);

    Serial.print("Writing to file...");
    if (file)
    {
        file.println(data);
        file.close();
        Serial.println("done.");
        return true;
    }
    else
    {
        Serial.println("FAILED.");
        return false;
    }
}