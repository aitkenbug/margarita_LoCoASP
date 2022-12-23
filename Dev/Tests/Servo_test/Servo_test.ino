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

#include <Servo.h>     // Servo library
#include <Wire.h>      // I2C library

Servo az_servo;
Servo el_servo;

int az_angles[25] = {  0,   0,   0,   0,   0,
                      90,  90,  90,  90,  90,
                     180, 180, 180, 180, 180,
                     270, 270, 270, 270, 270,
                     360, 360, 360, 360, 360};
int el_angles[25] = {  0,  45,  90, 135, 180,
                       0,  45,  90, 135, 180,
                       0,  45,  90, 135, 180,
                       0,  45,  90, 135, 180,
                       0,  45,  90, 135, 180};

void setup() {
    Serial.begin(115200);
    az_servo.attach(9);
    el_servo.attach(10);
    az_servo.writeMicroseconds(sec(00));
    el_servo.write(180);
    delay(3000);
    DDRD |= B11110000; // Sets Ports 4, 5, 6 and 7 as OUTPUT.
    DDRB |= B00100001; // Sets Ports 8 and 13 as OUTPUT.
    PORTD |= B11110000;
    PORTB |= B00100001;
    Serial.println(F("Checking the servos..."));
    for (int i=0; i<25; i++) {
        track_the_sun(az_angles[i], el_angles[i]);
        Serial.print(F("Azimuth: "));
        Serial.print(az_angles[i]);
        Serial.print(F("    Elevation: "));
        Serial.println(el_angles[i]);
        delay(1000);
    }
    track_the_sun(90, 90);
    delay(2000);
    az_servo.detach();
    el_servo.detach();
    Serial.println(F("Done."));
}

void loop() {
}

void track_the_sun(float azimuth, float elevation) {
    float correctaz = 0.0, correctel = 0.0;
    int sensor0 = 0, sensor1 = 0, sensor2 = 0, sensor3 = 0;
    int suma = 0, comb01 = 0, comb32 = 0, comb03 = 0, comb12 = 0;
    int factor_s = 1;
    int x = 0, x0 = 0, x1 = 0, x2 = 0, x3 = 0;
    int y = 0, y0 = 0, y1 = 0, y2 = 0, y3 = 0;
    int hor = 0, ver = 0, hor0 = 0, ver0 = 0;

    Serial.println(F("Attaching servos"));
    az_servo.attach(9);
    el_servo.attach(10);
  
    if ((azimuth >= 0) && (azimuth <= 90)) {
        correctaz = int(90 - azimuth);
        correctel = int(elevation);
    }
    else if (((azimuth > 90) && (azimuth <= 180)) || ((azimuth > 180) and (azimuth < 270))) {
        correctaz = int(270 - azimuth);
        correctel = int(180 - elevation);
        factor_s = -1;
    }
    else {
        correctaz = int(450-azimuth);
        correctel = elevation;
    }

    az_servo.writeMicroseconds(sec(int(correctaz)));
    el_servo.writeMicroseconds(sec(int(correctel)));
    Serial.println(F("Rough position calculated"));
}

int sec(int in) {
    return map(in, -45, 225, 500, 2500);   
}
