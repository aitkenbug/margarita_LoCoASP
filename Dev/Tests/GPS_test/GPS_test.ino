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
#include <TinyGPS++.h> // GPS Library, to be replaced by raw NMEA commands.
#include <SoftwareSerial.h> // Serial port for non-UART pins. For use in NMEA-GPS.

#ifdef ESP32
    #define trackerTrigger 21
    SoftwareSerial ss(33, 32); // Conexion serial para conectarse al GPS ss(rx,tx)
#else
    #define trackerTrigger A2
    SoftwareSerial ss(8, 9); // Conexion serial para conectarse al GPS ss(rx,tx)
#endif

TinyGPSPlus gps; // GPS object.

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
    delay(1500);
    Serial.begin(115200);
    struct instrumentStructure instrumentData;
    Serial.println(F("Initializing the GPS module..."));
    ss.begin(GPSBaud);//,SWSERIAL_8N1,12,13,false,256);
    Serial.println(F("Done.\nTesting the code..."));
    for (int i=0; i < 10; i++) {
        GPS_sw_test(&instrumentData);
        Serial.println(data2csv(&instrumentData));
    }
    Serial.println(F("Done.\nTesting the GPS module..."));
}

void loop() {
    struct instrumentStructure instrumentData;
    GPS(&instrumentData);
    Serial.println(data2csv(&instrumentData));
    delay(500);
}

void GPS_sw_test(struct instrumentStructure *instrumentData) {
    instrumentData->gps_lat = random(-340000, -320000)*0.0001;
    instrumentData->gps_lng = random(-720000, -690000)*0.0001;
    instrumentData->gps_day = random(1, 32);
    instrumentData->gps_month = random(1, 13);
    instrumentData->gps_year = random(2023, 2026);
    instrumentData->gps_hour = random(0, 24);
    instrumentData->gps_minute = random(0, 61);
    instrumentData->gps_second = random(0, 61);
    instrumentData->gps_alt = random(0, 550000)*0.01;
}

void GPS(struct instrumentStructure *instrumentData) {
    //GPS data parsing and collation, hugely inneficient. To be replaced by straight NMEA communication.
    unsigned long timeout = millis() + 1000;
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

    sprintf(data_CSV,
            "007,%d,%d,%d,%d,%s,%c,%s,%c,%d,%d,%d,%d,%d,%d,%s,%s,%s,%s",
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
    return data_CSV;
}
