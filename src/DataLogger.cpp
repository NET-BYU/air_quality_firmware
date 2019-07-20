#include "DataLogger.h"

DataLogger::DataLogger(uint8_t csPin, uint8_t cdPin, SPISettings spiSettings, String file_name,
                       uint32_t timeout)
{
    DataLogger::cdPin = cdPin;
    DataLogger::csPin = csPin;
    DataLogger::spiSettings = spiSettings;
    DataLogger::file_name = file_name;
    DataLogger::timeout = timeout;

    DataLogger::card_init = false;

    pinMode(cdPin, INPUT);
}

bool DataLogger::begin()
{
    // Start card
    if (!sd.begin(csPin, spiSettings))
    {
        Serial.println("Initialization failed!");
        return false;
    }

    Serial.println("Initialization done!");

    return true;
}

bool DataLogger::cardPresent()
{
    return digitalRead(cdPin);
}

bool DataLogger::waitForCardPresent()
{
    uint32_t maxRetries = timeout / 100;
    uint32_t cdRetry = 0;
    while (!cardPresent() && cdRetry < maxRetries)
    {
        cdRetry++;
        delay(100);
        Serial.printf("%d...", cdRetry);
        Particle.process();
    }
    Serial.println();

    if (cdRetry == maxRetries)
    {
        return false;
    }

    return true;
}

bool DataLogger::read()
{
    return false;
}

bool DataLogger::write(String data)
{
    Serial.println("Checking if card is present...");
    if (!waitForCardPresent())
    {
        Serial.println("Card is not inserted...");
        card_init = false;
        return false;
    }

    if (!card_init)
    {
        Serial.println("Card is not already initialized so we need to initialize it!");
        if (!sd.begin(csPin, spiSettings))
        {
            Serial.println("Initialization failed!");
            return false;
        }
        else
        {
            Serial.println("Initialization succeeded!");
            card_init = true;
        }
    }

    file = sd.open(file_name, FILE_WRITE);

    if (file)
    {
        Serial.print("Writing to file...");
        file.println(data);
        file.close();
        Serial.println("done.");
        return true;
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("Error opening file");
        return false;
    }
}