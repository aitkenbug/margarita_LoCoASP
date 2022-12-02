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

static const uint32_t GPSBaud = 9600; // GPS software UART speed. To be hard-coded, as it does not change.
char gps_data[50] = {0};

TinyGPSPlus gps; // GPS object.
SoftwareSerial ss; // Conexion serial para conectarse al GPS

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    delay(1500);
    Serial.println(F("Initializing the GPS module..."));
    ss.begin(GPSBaud,SWSERIAL_8N1,12,13,false,256);
    Serial.println(F("Done.\nTesting the code..."));
    test_GPS();
    Serial.println(F("Done.\nTesting the GPS module..."));
}

void loop() {
    Serial.println(GPS());
    delay(500);
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

void test_GPS() {
    char lat_str[8], lng_str[8], alt_str[8];
    float lat = 0.0, lng = 0.0, alt = 0.0;
    Serial.println(F("Testing the GPS string generator: "));
    memset(&gps_data[0], 0, sizeof(gps_data));
    // A sample NMEA stream.
    const char *gpsStream =
      "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
      "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
      "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
      "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
      "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
      "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";
    while (*gpsStream) {
        if (gps.encode(*gpsStream++)) {
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
            Serial.println(gps_data);
        }
    }
}
