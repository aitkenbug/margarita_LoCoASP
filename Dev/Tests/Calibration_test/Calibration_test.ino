// ------------------------------------------------------------------------------------------------------------------
//      __  ___                            _ __              __          ______      ___   _____ ____ 
//     /  |/  /___ __________ _____ ______(_) /_____ _      / /   ____  / ____/___  /   | / ___// __ \
//    / /|_/ / __ `/ ___/ __ `/ __ `/ ___/ / __/ __ `/_____/ /   / __ \/ /   / __ \/ /| | \__ \/ /_/ /
//   / /  / / /_/ / /  / /_/ / /_/ / /  / / /_/ /_/ /_____/ /___/ /_/ / /___/ /_/ / ___ |___/ / _, _/ 
//  /_/  /_/\__,_/_/   \__, /\__,_/_/  /_/\__/\__,_/     /_____/\____/\____/\____/_/  |_/____/_/ |_|  
//                    /____/                                                                          
// ------------------------------------------------------------------------------------------------------------------
// Second Generation Low-Cost Automatic Sun Photometer for scale-oriented measurements of Aerosol Optical Depth (AOD)
// Space and Planetary Exploration Laboratory, Faculty of Physical and Matemathical Sciences, University of Chile
// ------------------------------------------------------------------------------------------------------------------
// First generation developement by Cristobal Garrido and MCI Electronics
// Second generation developement by Benjamín Santelices, Vicente Aitken, José Ferrada and Matías Vidal
// ------------------------------------------------------------------------------------------------------------------
// INSTRUMENT.INO > Photometer Instrument Unit
// Firmware for Instrument Unit - Arduino Uno/ESP32


//---LIBRARIES---
#include <Arduino.h>        // For using the ESP32 with the Arduino IDE.
#include <TinyGPSPlus.h>    // GPS Library, to be replaced by raw NMEA commands.
#include <SFE_BMP180.h>     // BMP180 for pressure, altitude, and temperature of turret assembly.
#include <Wire.h>           // I2C bus for BMP180
#include <SD.h>             // SD Card Library
#include <SPI.h>            // Hardware SPI library for MCP3204 ADC.

#ifdef ESP32
    #define trackerTrigger 21
#else
    #define trackerTrigger A2
#endif
#define CS_ADC 4         // ADC chip select.
#define CS_SD 10         //SD chip select. Matches hardware SPI bus implementation on 328P.

static const PROGMEM uint32_t GPSBaud = 9600; // GPS software UART speed. To be hard-coded, as it does not change.
struct instrumentStructure {
    int led1 = 0;
    int led2 = 0;
    int led3 = 0;
    int led4 = 0;
    float gps_lat = 0.0;
    float gps_lng = 0.0;
    int gps_day = 0;
    int gps_month = 0;
    int gps_year = 0;
    int gps_hour = 0;
    int gps_minute = 0;
    int gps_second = 0;
    float gps_alt = 0.0;
    double bmp_temp = 0.0;
    double bmp_pres = 0.0;
    float bmp_alt = 0.0;
};

void setup() {
    struct instrumentStructure instrumentData[3];
    pinMode(trackerTrigger, INPUT); //stop trigger from Tracker Unit init
    pinMode(CS_ADC, OUTPUT); // pinMode!!!
    digitalWrite(CS_ADC, HIGH); //turn off ADC
    Serial.begin(115200);
    delay(1000);

    //---MEASURING STARTS HERE---
    while (digitalRead(trackerTrigger) == 0) {
        //do nothing while trackerTrigger is low
        //WARN: isn't trackerTrigger being checked twice? here and on data() function. possible efficiency boost here.
    }
    measurement(&instrumentData[0]);
    measurement(&instrumentData[1]);
    measurement(&instrumentData[2]);
}

void loop() {//nothing happens here.
}

//---DATA ACQUISITION FUNCTIONS---

void measurement(struct instrumentStructure *instrumentData) {
    SPI.begin();
    Serial.print(F("Measuring sensors..."));
    data(instrumentData); //ADC data
    Serial.println(F("          Done."));
    SPI.end();
}

void data(struct instrumentStructure *instrumentData) {
    //Sensor data processing and collation.
    int readvalue = 0;
    //Sensor readout, keep highest value of each sensor.
    unsigned long timeout = millis() + 30000; //El tiempo de inicio para marcar
    while (millis() < timeout) {//Second check of trackerTrigger (?)
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
        readvalue = read_ADC(0);
        if (instrumentData->led1 <= readvalue) {
            instrumentData->led1 = readvalue;
        }

        readvalue = read_ADC(1);
        if (instrumentData->led2 <= readvalue) {
            instrumentData->led2 = readvalue;
        }

        readvalue = read_ADC(2);
        if (instrumentData->led3 <= readvalue) {
            instrumentData->led3 = readvalue;
        }

        readvalue = read_ADC(3);
        if (instrumentData->led4 <= readvalue) {
            instrumentData->led4 = readvalue;
        }
        SPI.endTransaction();
    }
    Serial.println(F("Calibration results:"));
    Serial.print(F("LED 1: "));
    is_saturated(instrumentData->led1);
    Serial.print(F("LED 2: "));
    is_saturated(instrumentData->led2);
    Serial.print(F("LED 3: "));
    is_saturated(instrumentData->led3);
    Serial.print(F("LED 4: "));
    is_saturated(instrumentData->led4);
}

void is_saturated(int led_value) {
    Serial.print(led_value);
    if (led_value > 2200) {
        Serial.println(F("  => The value is saturated, increase the resistance."));
    }
    else if (led_value < 1800) {
        Serial.println(F("  => The value is too low, reduce the resistance."));
    }
    else {
        Serial.println(F("  => This LED is calibrated."));
    }
}

int read_ADC(int channel) {
    //ADC SPI interface
    const int byte8 = 0x06; //setup byte
    int adcValue = 0;
    int byte16 = channel << 14; //bitshifted channel for second block.
    //digitalWrite(CS_ADC, LOW); //select MCP3204
    PORTD &= B11101111;
    SPI.transfer(byte8);
    adcValue = SPI.transfer16(byte16) & 0x0FFF; //ADC sample bitmasking.
    //digitalWrite(CS_ADC, HIGH); //turn off device
    PORTD |= B00010000;
    return adcValue;
}
