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

char bmp_data[30] = {0};

SFE_BMP180 pressure; // BMP180 object

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println(F("Starting the software test..."));
    BMP_test();
    Serial.println(F("Done."));
    Serial.println(F("Initializing the BMP180 pressure sensor..."));
    pressure.begin();
    Serial.println(F("Done."));
    Serial.println(F("Testing the BMP180 module..."));
}

void loop() {
    BMP();
    delay(500);
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

void BMP_test() {
    char temp_str[6], pres_str[7], alt_str[8];
    float temp[5] = {28.49, 24.82, 25.58, 19.14, 3.28};
    float pres[5] = {946.10, 948.93, 949.24, 948.96, 927.40};
    float alt[5] = {572.66, 547.77, 545.00, 547.49, 738.57};
    memset(&bmp_data[0], 0, sizeof(bmp_data));
    for (int i=0; i<5; i++) {
        if (abs(temp[i]) < 10)
            dtostrf(temp[i], 4, 2, temp_str);
        else
            dtostrf(temp[i], 5, 2, temp_str);
        dtostrf(pres[i], 6, 2, pres_str);
        if (alt[i] >= 1000.0)
            dtostrf(alt[i], 7, 2, alt_str);
        else
            dtostrf(alt[i], 6, 2, alt_str);
        sprintf(bmp_data, ",%s,%s,%s", temp_str, pres_str, alt_str);
        Serial.print(F("BMP data: "));
        Serial.println(bmp_data);
    }
}
