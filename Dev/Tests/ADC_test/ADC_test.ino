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
#include <SPI.h> // Hardware SPI library for MCP3204 ADC.

#ifdef ESP32
    #define trackerTrigger 21
#else
    #define trackerTrigger A2
#endif
#define CS_ADC 4 // ADC chip select.

#define SELPIN 4 //Pin de activación del ADC
#define DATAOUT 11 // MOSI
#define DATAIN 12 // MISO
#define SPICLOCK 13 // CLK
#define CTRL_Z 26 // termino

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
    pinMode(trackerTrigger, INPUT);
    pinMode(CS_ADC, OUTPUT);
    Serial.begin(115200);
    delay(1500);
    Serial.println(F("Testing the ADC MCP3204-BVSL..."));
    SPI.begin();
    Serial.println(F("Reading individual channels..."));
    Serial.print(F("ADC CH1: "));
    Serial.print(read_ADC(1));
    Serial.print(F("  CH2: "));
    Serial.print(read_ADC(2));
    Serial.print(F("  CH3: "));
    Serial.print(read_ADC(3));
    Serial.print(F("  CH4: "));
    Serial.println(read_ADC(4));
    Serial.println(F("Done."));
    Serial.println(F("Starting full test..."));
}

void loop() {
    struct instrumentStructure instrumentData;
    data(&instrumentData);
    Serial.println(data2csv(&instrumentData));
    delay(500);
}

void data(struct instrumentStructure *instrumentData) {
    //Sensor data processing and collation.
    int readvalue = 0;
    //Sensor readout, keep highest value of each sensor.
    while (digitalRead(trackerTrigger)) {
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
        readvalue = read_ADC(1);
        if (instrumentData->led1 <= readvalue) {
            instrumentData->led1 = readvalue;
        }

        readvalue = read_ADC(2);
        if (instrumentData->led2 <= readvalue) {
            instrumentData->led2 = readvalue;
        }

        readvalue = read_ADC(3);
        if (instrumentData->led3 <= readvalue) {
            instrumentData->led3 = readvalue;
        }

        readvalue = read_ADC(4);
        if (instrumentData->led4 <= readvalue) {
            instrumentData->led4 = readvalue;
        }
        SPI.endTransaction();
    }
}

int read_ADC(int channel) {
    //ADC SPI interface
    const int byte8 =0x06; //setup byte
    int adcValue = 0;
    int byte16 = (channel - 1) << 14; //bitshifted channel for second block.

    digitalWrite(CS_ADC, LOW); //select MCP3204
    SPI.transfer(byte8);
    adcValue = SPI.transfer16(byte16) & 0x0FFF; //ADC sample bitmasking.
    digitalWrite(CS_ADC, HIGH); //turn off device

    return adcValue;
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
