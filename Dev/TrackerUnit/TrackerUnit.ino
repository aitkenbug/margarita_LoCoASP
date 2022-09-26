//---LIBRARIES---
#include <avr/sleep.h> //deep sleep libraries
#include <DS3231.h> //DS3231 RTC library
#include <Wire.h> //I2C Library
#include <Servo.h> //Servo library

//---OBJECT DECLARATIONS---
Servo azServo;
Servo elServo;

DS3231 rtc;


