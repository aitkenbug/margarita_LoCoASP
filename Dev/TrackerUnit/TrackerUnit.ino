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
// Firmware for Instrument Unit - Arduino Uno / ESP32

#include <avr/sleep.h> // Libreria para Sleep
#include <Servo.h> // Libreria Servo
#include <DS3231.h> // Libreria reloj con alarma
#include <Wire.h> // Libreria control I2C

//Declaracion Servos
Servo myservo1;
Servo myservo2; 

//Declaracion Reloj
DS3231 Clock;
RTClib myRTC; //Raro... investigar como usar .now()@pp
// Variables del reloj
uint8_t second = 0, minute = 0, hour = 0,
        day = 0, month = 0, year = 0; //pa que float? xD // pa que int? xD

//Pin usado para depertarse
int wakePin = 2;                 // pin used for waking up
int sleepStatus = 0;             // variable to store a request for sleep

unsigned long tiempo = 0;
bool convergencia = true;
float tupper_lat = -33.458017, tupper_lng = -70.661989;
struct coordinates {
   float azimuth;
   float elevation;
};

void wakeUpNow() {       // here the interrupt is handled after wakeup
    // execute code here after wake-up before returning to the loop() function
    // timers and code using timers (serial.print and more...) will not work here.
    // we don't really need to execute any special functions here, since we
    // just want the thing to wake up
}

void setup() {
    // put your setup code here, to run once:
    pinMode(wakePin, INPUT);
    Wire.begin();
    Serial.begin(115200);

    //Serial.println(F("Attaching servos"));
    //myservo1.attach(9); //Inicializamos los motores (1 horizontal / 2 vertical)
    //myservo2.attach(10);

    /* Now it is time to enable an interrupt. In the function call
       attachInterrupt(A, B, C)
       A   can be either 0 or 1 for interrupts on pin 2 or 3.

       B   Name of a function you want to execute while in interrupt A.

       C   Trigger mode of the interrupt pin. can be:
                   LOW        a low level trigger
                   CHANGE     a change in level trigger
                   RISING     a rising edge of a level trigger
                   FALLING    a falling edge of a level trigger

       In all but the IDLE sleep modes only LOW can be used.
    */

    //attachInterrupt(1, wakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
    // wakeUpNow when pin 3 gets LOW
}

void sleepNow() {        // here we put the arduino to sleep
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
       http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
       there is a list of sleep modes which explains which clocks and
       wake up sources are available in which sleep mode.

       In the avr/sleep.h file, the call names of these sleep modes are to be found:

       The 5 different modes are:
           SLEEP_MODE_IDLE         -the least power savings
           SLEEP_MODE_ADC
           SLEEP_MODE_PWR_SAVE
           SLEEP_MODE_STANDBY
           SLEEP_MODE_PWR_DOWN     -the most power savings

       For now, we want as much power savings as possible, so we
       choose the according
       sleep mode: SLEEP_MODE_PWR_DOWN

    */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
    // so sleep is possible. just a safety pin

    /* Now it is time to enable an interrupt. We do it here so an
       accidentally pushed interrupt button doesn't interrupt
       our running program. if you want to be able to run
       interrupt code besides the sleep function, place it in
       setup() for example.

       In the function call attachInterrupt(A, B, C)
       A   can be either 0 or 1 for interrupts on pin 2 or 3.

       B   Name of a function you want to execute at interrupt for A.

       C   Trigger mode of the interrupt pin. can be:
                   LOW        a low level triggers
                   CHANGE     a change in level triggers
                   RISING     a rising edge of a level triggers
                   FALLING    a falling edge of a level triggers

       In all but the IDLE sleep modes only LOW can be used.
    */

    attachInterrupt(1, wakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
    // wakeUpNow when pin 3 gets LOW

    sleep_mode();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
    // disable sleep...
    detachInterrupt(1);      // disables interrupt 1 on pin 3 so the
    // wakeUpNow code will not be executed
    // during normal running time.
}

void loop() {
    //////////////////////////////////////////////////  
    //PUT YOUR LATITUDE, LONGITUDE, AND TIME ZONE HERE
    // TUPPER 2007
    //float latitude = -33.458017;float longitude = -70.661989;
    // ROSSINI 10620
    //float latitude = -33.5622523;float longitude = -70.603821799; float timezone = 0;
    // Valle Nevado
    //float latitude = -33.44888969;//float longitude = -70.6692655;
    update_date_and_time();
    set_next_alarm(minute);
    //START OF THE CODE THAT CALCULATES THE POSITION OF THE SUN
    Serial.println(F("Sun Check Start"));
    struct coordinates sun;
    sun = get_sun_position(tupper_lat, tupper_lng, month, day, hour, minute);
    //END OF THE CODE THAT CALCULATES THE POSITION OF THE SUN
    track_the_sun(sun.azimuth, sun.elevation);
}

void update_date_and_time() {
    //////////////////////////////////////////////////  
    //codigo mágico que pone en high la alarma
    //Revision precencia de alarma
    Serial.println(F("Alarm Check"));
    delay(1000);

    bool fse PROGMEM = false; //functions require bools to be passed as reference.
    second = (uint8_t)Clock.getSecond(); // Is it used?
    minute = (uint8_t)Clock.getMinute();
    hour = (uint8_t)Clock.getHour(fse, fse);
    day = (uint8_t)Clock.getDate();
    month = (uint8_t)Clock.getMonth(fse);
    year = (uint8_t)Clock.getYear(); //get current time  // Is the year used? 
}

void set_next_alarm(uint8_t minute) {
    Clock.checkIfAlarm(1);
    Clock.checkIfAlarm(2); //clears all alarm registers

    Clock.setA1Time(byte(0), byte(0), byte((minute + 5) % 60), byte(0), 0b00001100, false, false, false);
    Clock.turnOnAlarm(1);

    Serial.print(F("Next alarm set: "));
    Serial.print((minute + 5) % 60);
    Serial.print(F("/"));
    Serial.println(Clock.checkAlarmEnabled(1));

    delay(100);
}

struct coordinates get_sun_position(float latitude, float longitude, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute) {
    //If you live in the southern hemisphere, it would probably be easier
    //for you if you make north as the direction where the azimuth equals
    //0 degrees. To do so, switch the 0 below with 180.  
    float northOrSouth = 180;
    float pi = 3.14159265;
    struct coordinates angles;
    int daynum[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    float delta = 0.0, h = 0.0, timezone = 0.0;
    float two_pi = 2*pi;
    float rad2deg = 180/pi;
    float deg2rad = pi/180;
    latitude = latitude * deg2rad;
    float sin_lat = sin(latitude);
    float cos_lat = cos(latitude);
    float n = daynum[month-1] + day;//NUMBER OF DAYS SINCE THE START OF THE YEAR. 
    delta = .409279 * sin(two_pi * (284 + n)/365.25);//SUN'S DECLINATION.
    float sin_delta = sin(delta);
    float cos_delta = cos(delta);
    day = dayToArrayNum(day);//TAKES THE CURRENT DAY OF THE MONTH AND CHANGES IT TO A LOOK UP VALUE ON THE HOUR ANGLE TABLE.
    h = FindH(day,month) + longitude + (timezone * -15);//FINDS THE NOON HOUR ANGLE ON THE TABLE AND MODIFIES IT FOR THE USER'S OWN LOCATION AND TIME ZONE.
    h = ((((hour + minute/60) - 12) * 15) + h)*deg2rad;//FURTHER MODIFIES THE NOON HOUR ANGLE OF THE CURRENT DAY AND TURNS IT INTO THE HOUR ANGLE FOR THE CURRENT HOUR AND MINUTE.
    float cos_h = cos(h);
    angles.elevation = asin(sin_lat * sin_delta + cos_lat * cos_delta * cos_h)*rad2deg;//FINDS THE SUN'S ALTITUDE.
    angles.azimuth = atan2(sin(h), (cos_h * sin_lat) - sin_delta/cos_delta * cos_lat)*rad2deg + northOrSouth;//FINDS THE SUN'S AZIMUTH.
    return angles;
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
    myservo1.attach(9); //Inicializamos los motores (1 horizontal / 2 vertical)
    myservo2.attach(10);
  
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

    myservo1.writeMicroseconds(sec(int(correctaz)));
    myservo2.writeMicroseconds(sec(int(correctel)));
    Serial.println(F("Rough position calculated"));
    delay(3000);

    hor = int(correctaz);
    ver = int(correctel);
    hor0 = int(correctaz);
    ver0 = int(correctel);
    Serial.println(ver0);

    Serial.println(F("Starting Peripherals"));
//********************************************************************************************************************************************
//CODIGO DEL SEGUIDOR AQUI
//********************************************************************************************************************************************
    // Iniciamos las conexiones de energia a los motores, arduino uno y shield M2M
    /*
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(13, OUTPUT);
    */
    DDRD |= B11110000; // Sets Ports 4, 5, 6 and 7 as OUTPUT.
    DDRB |= B00100001; // Sets Ports 8 and 13 as OUTPUT.
    //Pin de conexion con arduino uno
    pinMode(2, OUTPUT);

    Serial.println(elevation);

    if (elevation >= 7) {
        /*
        digitalWrite(4, HIGH); // Desconexion Motores
        digitalWrite(5, HIGH); // Motor 1
        digitalWrite(6, HIGH); // Motor 2
        digitalWrite(7, HIGH); // Desconexion arduino UNO y M2M
        digitalWrite(8, HIGH); // Arduino UNO
        digitalWrite(13,HIGH); // M2M shield
        */
        PORTD |= B11110000;
        PORTB |= B00100001;
        x0 = 1; // Inicializamos el elemento derivativo
        y0 = 1;
        tiempo = millis();
        Serial.println(F("Starting Tracker"));

        bool test_found = false;

        convergencia = true;
        //Serial.println(millis());
        //Serial.println(tiempo+150000); 
        while(millis() < tiempo + 150000) {
            // Lectura de los sensores:
            sensor0 = analogRead(A0);
            delay(1);
            sensor1 = analogRead(A1);
            delay(1);
            sensor2 = analogRead(A2);
            delay(1);
            sensor3 = analogRead(A3);
            delay(1);

            suma = sensor0 + sensor1 + sensor2 + sensor3;
            comb01 = sensor0 + sensor1;
            comb32 = sensor3 + sensor2;
            comb03 = sensor0 + sensor3;
            comb12 = sensor1 + sensor2;
            /*
            Serial.print(F("Sensor 0: "));
            Serial.print(sensor0);
            Serial.print(F("    Sensor 1: "));
            Serial.print(sensor1);
            Serial.print(F("    Sensor 2: "));
            Serial.print(sensor2);
            Serial.print(F("    Sensor 3: "));
            Serial.println(sensor3);
            */
            char buffer[100] = {0};
            sprintf(buffer, "Sensor 0: %d    Sensor 1: %d    Sensor2: %d    Sensor 3: %d", sensor0, sensor1, sensor2, sensor3);
            Serial.println(buffer);

            // Comparamos entradas opuesta y desacoplamos las respuestas
            if (sensor0 >= sensor2) {
                x1 = -factor_s;
                y1 = 1;
            }
            else {
                x1 = factor_s;
                y1 = -1;
            }

            if (sensor1 >= sensor3) {
                x2 = factor_s;
                y2 = 1;
            }
            else {
                x2 = -factor_s;
                y2 = -1;
            }

            if (comb01 >= comb32) {
                y3 = 1;
            }
            else {
                y3 = -1;
            }

            if (comb03 >= comb12){x3 = -factor_s;}
            else{x3 = factor_s;}

            x = x1 + x2 + x3;
            y = y1 + y2 + y3;
            //Agregamos el elemento integral para evitar el error en estado estaiconario
     
            if (x == 0){x = x0;}else{x0 = x;}
            if (y == 0){y = y0;}else{y0 = y;}

            // Eliminamos cuando el valor es de 2
            x = 2*(x > 0) - 1;// Original: x/abs(x);
            y = 2*(y > 0) - 1;// Original: y/abs(y);

            hor0 = hor0 + x;
            ver0 = ver0 + y;

            if ((hor0 >= 225) or (hor0 <= -45)) {
                hor0 = hor;
            }

            if ((ver0 >= 225) or (ver0 <= -45)) {
                ver0 = ver;
            }

            if (suma >= 0) {
            //if (suma >= 200)
                if (convergencia) {
                    //señal arduino uno de medir A ESTA PARTE NO ACCEDE CUANDO SE HACEN PRUEBAS DE LABORATORIO
                    //LEA LA BIBLIA
                    digitalWrite(2, HIGH); 
                    convergencia = false; 
                }
            }
            //Ejecutamos el seguimiento
            //PARTE IMPORTANTE PARA REGULAR LAS VELOCIDADES
            myservo2.writeMicroseconds(sec(ver0));
            myservo1.writeMicroseconds(sec(hor0));
            delay(20);
            // waits for the servo to get there
        }
        // Ramp-down
        myservo1.writeMicroseconds(sec(00));
        myservo2.write(180);
        delay(5000);

        myservo1.detach(); //Inicializamos los motores (1 horizontal / 2 vertical)
        myservo2.detach();
        // Apagamos la energia de los motores
        digitalWrite(4, LOW);
        //informamos al arduino UNO que termina la medición
        digitalWrite(2, LOW);
        delay(35000);
        //pinMode(2,INPUT);
        digitalWrite(7, LOW);
    }
    Serial.println(F("Tracking Completed"));
    //Serial.println("OK");
    delay(1000);
    //esperamos que arduino UNO termine su transferencia
    // SEGMENTO DESACTIVADO PARA PRUEBAS
    //if (digitalRead(2)==0)
    //while(digitalRead(2)==0)
    //{
    //}
    //Apagamos arduino Uno y M2M
//********************************************************************************************************************************************
//FIN CODIGO|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//********************************************************************************************************************************************
    sleepNow();     // sleep function called here
}

bool found() {
    int sensor0 = analogRead(A0);
    delay(1);
    int sensor1 = analogRead(A1);
    delay(1);
    int sensor2 = analogRead(A2);
    delay(1);
    int sensor3 = analogRead(A3);
    delay(1);
    // Comparamos entradas opuesta y desacoplamos las respuestas
    // Basta que se cumpla una condición para que se justifique
    // Se usa para la busqueda aproximada del sol
    return abs(sensor0-sensor2)>=4||(abs(sensor1-sensor3)>=4);
}

int sec(int in) {
    return map(in, -45, 225, 500, 2500);   
}
