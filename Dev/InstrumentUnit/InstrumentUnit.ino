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
#include <SFE_BMP180.h> // BMP180 for pressure, altitude, and temperature of turret assembly.
#include <Wire.h> // I2C bus for BMP180
#include <TinyGPS++.h> // GPS Library, to be replaced by raw NMEA commands.
#include <SoftwareSerial.h> // Serial port for non-UART pins. For use in NMEA-GPS.
#include <SD.h> // SD Card Library
#include <SPI.h> // Hardware SPI library for MCP3204 ADC.

char ID[4] = "007"; // Instrument ID for tracking, to be ported on config.h
char bmp_data[30] = {0};
char gps_data[50] = {0};
char sensor_data[30] = {0};
static const uint32_t GPSBaud = 9600; // GPS software UART speed. To be hard-coded, as it does not change.

SFE_BMP180 pressure; // BMP180 object
TinyGPSPlus gps; // GPS object.
SoftwareSerial ss(3, 2); // Conexion serial para conectarse al GPS

#define CS_ADC 4 // ADC chip select.
#define CS_SD 8  //SD chip select. Matches hardware SPI bus implementation on 328P.

void setup() {
    String zz, zz2, zz3, zz4;
    pinMode(A2,INPUT); //stop trigger from Tracker Unit init
    pinMode(CS_ADC, OUTPUT); // pinMode!!!
    //debug UART, GPS softUART, BMP init
    Serial.begin(115200);
    SPI.begin();
    delay(1500);
    Serial.println(F("Begin."));
    ss.begin(GPSBaud); 
    pressure.begin();

    //---MEASURING STARTS HERE---
    zz = ID;
    while (digitalRead(A2) == 0) {
        //do nothing while A2 is low
        //WARN: isn't A2 being checked twice? here and on data() function. possible efficiency boost here.
    }

    delay(100);
    zz2 = data(); //ADC data

    delay(100);
    zz3 = GPS(); //GPS data
    delay(100);

    SD.begin(CS_SD); //SD init
    delay(100);

    zz4 = BMP(); //BMP180 data
    delay(100);

    //---DATA STORAGE---
    File dataFile = SD.open("Data.txt", FILE_WRITE);
    if (dataFile) { //check availability
        dataFile.print(zz);
        dataFile.print(zz2);
        dataFile.print(zz3);
        dataFile.println(zz4); //WARN: maybe better to assemble string internally? not really relevant tho.
        dataFile.close();
        Serial.println(F("Data saved successfully."));
    }
    else {
        Serial.println(F("Error saving to Data.txt")); //WARN: good idea to implement some sort of debugging here.
    }
    delay(100);
}

void loop() {//nothing happens here.
}

//---DATA ACQUISITION FUNCTIONS---

String data() {
    //Sensor data processing and collation.
    int readvalue = 0, data1 = 0, data2 = 0, data3 = 0, data4 = 0;
    memset(&sensor_data[0], 0, sizeof(sensor_data));
    //Sensor readout, keep highest value of each sensor.
    while (digitalRead(A2)) {//Second check of A2 (?)
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
        readvalue = read_ADC(1);
        if (data1 <= readvalue) {
            data1 = readvalue;
        }

        readvalue = read_ADC(2);
        if (data2 <= readvalue) {
            data2 = readvalue;
        }

        readvalue = read_ADC(3);
        if (data3 <= readvalue) {
            data3 = readvalue;
        }

        readvalue = read_ADC(4);
        if (data4 <= readvalue) {
            data4 = readvalue;
        }
        SPI.endTransaction();
    }
    sprintf(sensor_data, ",%d,%d,%d,%d", data1, data2, data3, data4);
    Serial.print(F("Sensor data: "));
    Serial.println(sensor_data);
    //WARN: maybe some debugging here?
    return sensor_data;
}

String GPS() {
    //GPS data parsing and collation, hugely inneficient. To be replaced by straight NMEA communication.
    char lat_str[8], lng_str[8], alt_str[8];
    float lat = 0.0, lng = 0.0, alt = 0.0;
    memset(&gps_data[0], 0, sizeof(gps_data));
    unsigned long tiempo = millis(); //El tiempo de inicio para marcar
    while (millis() < tiempo + 30000) {
        while (ss.available() > 0) {
            if (gps.encode(ss.read())) {
                if (gps.location.isValid()) {
	            // isValid checks for the complete GPRMC frame.
                    lat = gps.location.lat();
                    lng = gps.location.lng();
                    alt = gps.altitude.meters();
                    dtostrf(abs(lat), 7, 4, lat_str);
                    if (abs(lng) >= 100.0)
                        dtostrf(abs(lng), 8, 4, lng_str);
                    else
                        dtostrf(abs(lng), 7, 4, lng_str);
                    if (alt >= 1000)
                        dtostrf(alt, 7, 2, alt_str);
                    else
                        dtostrf(alt, 6, 2, alt_str);
                    sprintf(gps_data, ",%s,%c,%s,%c,%d,%d,%d,%d,%d,%d,%s", lat_str, 'S'-5*(lat > 0),
                                                                           lng_str, 'W'-18*(lng > 0),
                                                                           gps.date.day(),
                                                                           gps.date.month(),
                                                                           gps.date.year(),
                                                                           gps.time.hour(),
                                                                           gps.time.minute(),
                                                                           gps.time.second(),
                                                                           alt_str);
	                break;
                }
            }
        }
    }
    Serial.print(F("GPS data: "));
    Serial.println(gps_data);
    return gps_data;
}

String BMP() {
    //BMP180 data gathering. IC out of production, would be wise to replace.
    char temp_str[6], pres_str[7], alt_str[8];
    double temp = 0.0, pres = 0.0, alt = 0.0;
    uint8_t wait = 0;

    memset(&bmp_data[0], 0, sizeof(bmp_data));
    wait = pressure.startTemperature();
    delay(wait);

    if (!pressure.getTemperature(temp))
        temp=0.0;
    wait = pressure.startPressure(3);
    delay(wait);

    if (!pressure.getPressure(pres, temp))
        pres = 0.0;
    alt = pressure.altitude(pres, 1013); // P0 = 1013

    if (abs(temp) < 10)
        dtostrf(temp, 4, 2, temp_str);
    else
        dtostrf(temp, 5, 2, temp_str);
    dtostrf(pres, 6, 2, pres_str);
    if (alt >= 1000.0)
        dtostrf(alt, 7, 2, alt_str);
    else
        dtostrf(alt, 6, 2, alt_str);
    sprintf(bmp_data, ",%s,%s,%s", temp_str, pres_str, alt_str);
    Serial.print(F("BMP data: "));
    Serial.println(bmp_data);
    return bmp_data;
}

int read_ADC(int channel) {
    //ADC SPI interface
    const int byte8 = 0x06; //setup byte
    int adcValue = 0;
    int byte16 = (channel - 1) << 14; //bitshifted channel for second block.

    digitalWrite(CS_ADC, LOW); //select MCP3204
    SPI.transfer(byte8);
    adcValue = SPI.transfer16(byte16) & 0x0FFF; //ADC sample bitmasking.

    digitalWrite(CS_ADC, HIGH); //turn off device
    return adcValue;
}
