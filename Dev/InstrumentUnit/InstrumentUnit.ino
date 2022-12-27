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
// Firmware for Instrument Unit - Arduino Uno


//---LIBRARIES---
#include <Arduino.h>        // For using the ESP32 with the Arduino IDE.
#include <TinyGPSPlus.h>    // GPS Library, to be replaced by raw NMEA commands.
#include <SFE_BMP180.h>     // BMP180 for pressure, altitude, and temperature of turret assembly.
#include <Wire.h>           // I2C bus for BMP180
#include <SoftwareSerial.h> // Serial port for non-UART pins. For use in NMEA-GPS.
#include <SD.h>             // SD Card Library
#include <SPI.h>            // Hardware SPI library for MCP3204 ADC.

#ifdef ESP32
    #define trackerTrigger 21
    SoftwareSerial ss(33, 32); // Serial connection for the GPS ss(rx,tx)
#else
    #define trackerTrigger A2
    SoftwareSerial ss(8, 9);   // Serial connection for the GPS ss(rx,tx)
#endif
#define CS_ADC 4         // ADC chip select.
#define CS_SD 10         //SD chip select. Matches hardware SPI bus implementation on 328P.

SFE_BMP180 pressure;     // BMP180 object
TinyGPSPlus gps;         // GPS object.

char filename[20];
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
    struct instrumentStructure instrumentData[3];
    pinMode(trackerTrigger, INPUT); //stop trigger from Tracker Unit init
    pinMode(CS_ADC, OUTPUT); // pinMode!!!
    digitalWrite(CS_ADC, HIGH); //turn off ADC
    Serial.begin(115200);
    delay(1000);
    Serial.print(F("\nInitiating software serial..."));
    #ifdef ESP32
        ss.begin(GPSBaud,SWSERIAL_8N1,12,13,false,256);
    #else
        ss.begin(GPSBaud);
    #endif 
    Serial.print(F(" Done.\nInitiating the BMP180..."));
    pressure.begin();
    Serial.println(F("      Done."));

    //---MEASURING STARTS HERE---
    while (digitalRead(trackerTrigger) == 0) {
        //do nothing while trackerTrigger is low
        //WARN: isn't trackerTrigger being checked twice? here and on data() function. possible efficiency boost here.
    }
    measurement(&instrumentData[0]);
    data2csv(&instrumentData[0]);
    measurement(&instrumentData[1]);
    data2csv(&instrumentData[1]);
    measurement(&instrumentData[2]);
    data2csv(&instrumentData[2]);
    current_saved_data();
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

    Serial.print(F("Reading the GPS module..."));
    GPS(instrumentData); //GPS data
    Serial.println(F("     Done."));

    Serial.print(F("Measuring with the BMP180..."));
    BMP(instrumentData); //BMP180 data
    Serial.println(F("  Done."));
    delay(100);
    int data_year = instrumentData->gps_year - 2000*(instrumentData->gps_year > 0);
    snprintf(filename, 20, "000/%d%d%d%d.csv", data_year,
                                               instrumentData->gps_month,
                                               instrumentData->gps_day,                                                                   
                                               instrumentData->gps_hour);
    Serial.println(filename);
}

void data(struct instrumentStructure *instrumentData) {
    //Sensor data processing and collation.
    int readvalue = 0;
    //Sensor readout, keep highest value of each sensor.
    unsigned long timeout = millis() + 30000; //El tiempo de inicio para marcar
    while (digitalRead(trackerTrigger) && millis() < timeout) {//Second check of trackerTrigger (?)
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
}

void GPS(struct instrumentStructure *instrumentData) {
    //GPS data parsing and collation, hugely inneficient. To be replaced by straight NMEA communication.
    unsigned long timeout = millis() + 20000; //El tiempo de inicio para marcar
    while (millis() < timeout) {
        while (ss.available() > 0) {
            if (gps.encode(ss.read())) {
                if (gps.location.isValid()) {
	            // isValid checks for the complete GPRMC frame.
                    instrumentData->gps_lat = gps.location.lat();
                    instrumentData->gps_lng = gps.location.lng();
                    instrumentData->gps_day = gps.date.day();
                    instrumentData->gps_month = gps.date.month();
                    instrumentData->gps_year = gps.date.year();
                    instrumentData->gps_hour = gps.time.hour();
                    instrumentData->gps_minute = gps.time.minute();
                    instrumentData->gps_second = gps.time.second();
                    instrumentData->gps_alt = (float)gps.altitude.meters();
	            break;
                }
            }
        }
    }
}

void BMP(struct instrumentStructure *instrumentData) {
    //BMP180 data gathering. IC out of production, would be wise to replace.
    uint8_t wait = 0;
    wait = pressure.startTemperature();
    delay(wait);

    if (!pressure.getTemperature(instrumentData->bmp_temp))
        instrumentData->bmp_temp = 0.0;
    wait = pressure.startPressure(3);
    delay(wait);

    if (!pressure.getPressure(instrumentData->bmp_pres, instrumentData->bmp_temp))
        instrumentData->bmp_pres = 0.0;
    instrumentData->bmp_alt = pressure.altitude(instrumentData->bmp_pres, 1013); // P0 = 1013
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

void data2csv(struct instrumentStructure *instrumentData) {
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

    sprintf(data_CSV,
            "000,%d,%d,%d,%d,%s,%c,%s,%c,%d,%d,%d,%d,%d,%d,%s,%s,%s,%s",
            instrumentData->led1,
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
    Serial.println(data_CSV);
    SPI.begin();
    Serial.print(F("Initiating the uSD card..."));
    SD.begin(CS_SD); //SD init
    Serial.println(F("    Done."));
    delay(100);

    //---DATA STORAGE---
    PORTD |= B00010000; // Turn off ADC
    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) { //check availability
        dataFile.seek(EOF);
        dataFile.println(data_CSV);
        Serial.println(F("Data saved successfully."));
    }
    else {
        Serial.print(F("Error saving to "));
        Serial.println(filename);
    }
    dataFile.close();
}

void current_saved_data() {
    Serial.println(F("\nCurrently saved data:\n"));
    File dataFile = SD.open(filename);
    if (dataFile) {
        while (dataFile.available()) {
            Serial.write(dataFile.read());
        }
    }
    dataFile.close();
}
