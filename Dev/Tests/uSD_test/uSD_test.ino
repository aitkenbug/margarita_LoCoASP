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
#include <SD.h>             // SD Card Library
#include <SPI.h>            // Hardware SPI library for MCP3204 ADC.

#define CS_ADC 4         // ADC chip select.
#define CS_SD 10         //SD chip select. Matches hardware SPI bus implementation on 328P.

static const PROGMEM uint32_t GPSBaud = 9600; // GPS software UART speed. To be hard-coded, as it does not change.
struct instrumentStructure {
    int led1 = 0;
    int led2 = 0;
    int led3 = 0;
    int led4 = 0;
    double gps_lat = 0.0;
    double gps_lng = 0.0;
    int gps_day = 0;
    int gps_month = 0;
    int gps_year = 0;
    int gps_hour = 0;
    int gps_minute = 0;
    int gps_second = 0;
    float gps_alt = 0.0;
    double bmp_temp = 0.0;
    double bmp_pres = 0.0;
    double bmp_alt = 0.0;
};

void setup() {
    delay(1000);
    struct instrumentStructure instrumentData;
    pinMode(CS_ADC, OUTPUT); // pinMode!!!
    digitalWrite(CS_ADC, HIGH); //turn off device
    Serial.begin(115200);
    Serial.println(F("Testing the structure..."));
    for (int i=0; i < 10; i++) {
        SPI.begin();
        data(&instrumentData); //ADC data
        SPI.end();
        delay(100);

        GPS(&instrumentData); //GPS data
        delay(100);

        BMP(&instrumentData); //BMP180 data
        delay(100);
        Serial.println(data2csv(&instrumentData));
    }
    Serial.println(F("Done."));

    SPI.begin();
    Serial.print(F("Initiating the uSD card..."));
    SD.begin(CS_SD); //SD init
    Serial.println(F("           Done."));
    delay(100);

    //---DATA STORAGE---
    File dataFile = SD.open("Data.txt", FILE_WRITE);
    if (dataFile) { //check availability
        dataFile.seek(EOF);
        dataFile.println(data2csv(&instrumentData));
        dataFile.close();
        Serial.println(F("Data saved successfully."));
    }
    else {
        Serial.println(F("Error saving to Data.txt")); //WARN: good idea to implement some sort of debugging here.
    }
    delay(100);

    Serial.println(F("\nCurrently saved data:\n"));
    dataFile = SD.open("Data.txt");
    if (dataFile) {
        while (dataFile.available()) {
            Serial.write(dataFile.read());
        }
    }
    dataFile.close();
}

void loop() {//nothing happens here.
}

//---DATA ACQUISITION FUNCTIONS---

void data(struct instrumentStructure *instrumentData) {
    //Sensor data processing and collation.
    int readvalue = 0;
    //Sensor readout, keep highest value of each sensor.
    unsigned long timeout = millis() + 1;
    while (millis() < timeout) {
        readvalue = random(0, 4096);
        if (instrumentData->led1 <= readvalue) {
            instrumentData->led1 = readvalue;
        }

        readvalue = random(0, 4096);
        if (instrumentData->led2 <= readvalue) {
            instrumentData->led2 = readvalue;
        }

        readvalue = random(0, 4096);
        if (instrumentData->led3 <= readvalue) {
            instrumentData->led3 = readvalue;
        }

        readvalue = random(0, 4096);
        if (instrumentData->led4 <= readvalue) {
            instrumentData->led4 = readvalue;
        }
    }
}

void GPS(struct instrumentStructure *instrumentData) {
    instrumentData->gps_lat = random(-3400, -3200)*0.01;
    instrumentData->gps_lng = random(-7200, -6900)*0.01;
    instrumentData->gps_day = random(1, 32);
    instrumentData->gps_month = random(1, 13);
    instrumentData->gps_year = random(2023, 2026);
    instrumentData->gps_hour = random(0, 24);
    instrumentData->gps_minute = random(0, 61);
    instrumentData->gps_second = random(0, 61);
    instrumentData->gps_alt = random(0, 550000)*0.01;
}

void BMP(struct instrumentStructure *instrumentData) {
    //BMP180 data gathering. IC out of production, would be wise to replace.
    uint8_t wait = 0;

    //memset(&bmp_data[0], 0, sizeof(bmp_data));
    wait = random(0,300);
    delay(wait);

    instrumentData->bmp_temp = random(0, 3000)*0.01;
    wait = random(0,300);
    delay(wait);

    instrumentData->bmp_pres = random(40000, 101300)*0.01;
    instrumentData->bmp_alt = random(0, 550000)*0.01;
}

String data2csv(struct instrumentStructure *instrumentData) {
    char data_CSV[110] = {0};
    char lat_str[8], lng_str[8], gps_alt_str[8];
    char temp_str[6], pres_str[7], bmp_alt_str[8];

    dtostrf(abs(instrumentData->gps_lat), 7, 4, lat_str);
    if (abs(instrumentData->gps_lng) >= 100.0)
        dtostrf(abs(instrumentData->gps_lng), 8, 4, lng_str);
    else
        dtostrf(abs(instrumentData->gps_lng), 7, 4, lng_str);
    if (instrumentData->gps_alt >= 1000)
        dtostrf(instrumentData->gps_alt, 7, 2, gps_alt_str);
    else
        dtostrf(instrumentData->gps_alt, 6, 2, gps_alt_str);

    if (abs(instrumentData->bmp_temp) < 10)
        dtostrf(instrumentData->bmp_temp, 4, 2, temp_str);
    else
        dtostrf(instrumentData->bmp_temp, 5, 2, temp_str);
    dtostrf(instrumentData->bmp_pres, 6, 2, pres_str);
    if (instrumentData->bmp_alt >= 1000.0)
        dtostrf(instrumentData->bmp_alt, 7, 2, bmp_alt_str);
    else
        dtostrf(instrumentData->bmp_alt, 6, 2, bmp_alt_str);

    sprintf(data_CSV, "007,%d,%d,%d,%d,%s,%c,%s,%c,%d,%d,%d,%d,%d,%d,%s,%s,%s,%s", instrumentData->led1,
                                                                                   instrumentData->led2,
                                                                                   instrumentData->led3,
                                                                                   instrumentData->led4,
                                                                                   lat_str,
                                                                                   'S'-5*(instrumentData->gps_lat > 0),
                                                                                   lng_str,
                                                                                   'W'-18*(instrumentData->gps_lng > 0),
                                                                                   instrumentData->gps_day,
                                                                                   instrumentData->gps_month,
                                                                                   instrumentData->gps_year,
                                                                                   instrumentData->gps_hour,
                                                                                   instrumentData->gps_minute,
                                                                                   instrumentData->gps_second,
                                                                                   gps_alt_str,
                                                                                   temp_str,
                                                                                   pres_str,
                                                                                   bmp_alt_str);
    return data_CSV;
}
