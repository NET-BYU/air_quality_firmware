// Example usage for SPS30 library by Clemson_NESL.

#include "SPS30.h"

SPS30 Sensor;

int led = D7;

void setup() {
    pinMode(led,OUTPUT);
    digitalWrite(led,HIGH);
    Particle.function("PM-Led",pmT);
    
    
    Serial.begin(9600);
    Serial.println("SPS30 Example");
    
    if( !Sensor.begin() ) {
        Serial.println("SENSOR NOT DETECTED");
        delay(500);
//        System.reset();
    }
}

float mass_concen[4];
float num_concen[5];

char *pm[5] = {"PM0.5", "PM1.0", "PM2.5", "PM4.0", "PM10"};

int i=0;

void loop() {
    if (Sensor.dataAvailable()) { 
        Sensor.getMass(mass_concen);
        Sensor.getNum(num_concen);

        Serial.println("--Mass Concentration--");
        for(i=0; i<4;i++) {
            Serial.printf("%s: %0.2f\n", pm[i+1],mass_concen[i]);
        }
        
        Serial.println("--Number Concentration--");
        for(i=0; i<5;i++) {
            Serial.printf("%s: %0.2f\n", pm[i],num_concen[i]);
        }
        
    }
    else { Serial.println("NA"); }
    delay(2000);
}

int pmT(String command) {
    digitalWrite(led, digitalRead(led)^1 );
    return digitalRead(led);
}


