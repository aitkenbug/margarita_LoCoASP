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

SFE_BMP180 pressure; // BMP180 object

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
    Serial.begin(115200);
    Serial.println(F("Starting the software test..."));
    for (int i=0; i < 10; i++) {
        BMP_test(&instrumentData);
        Serial.println(data2csv(&instrumentData));
    }
    Serial.println(F("Done."));
    Serial.println(F("Initializing the BMP180 pressure sensor..."));
    pressure.begin();
    Serial.println(F("Done."));
    Serial.println(F("Testing the BMP180 module..."));
}

void loop() {
    struct instrumentStructure instrumentData;
    BMP(&instrumentData);
    Serial.println(data2csv(&instrumentData));
    delay(500);
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

void BMP_test(struct instrumentStructure *instrumentData) {
    //BMP180 data gathering. IC out of production, would be wise to replace.
    uint8_t wait = 0;
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
