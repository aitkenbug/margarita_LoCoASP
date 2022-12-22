/*
Program for set the time for the DS321 module
Build by Vicente Aitken
August 2022
*/
#include <DS3231.h>
#include <Wire.h>

DS3231 Clock;
String ISO;
int Second, Minute, Hour, WeekDay, Date, Month, Year; // clock set parameters

void setup() {
    Serial.begin(115200); // the serial monitor needs to be set at 19200 baud in order to work properly, dont ask why
    Serial.println(F("Welcome to the rtc_set Program"));
    
    Serial.println(F("Insert the date in ISO 8601 format YYYYMMDDTHHMMSS:")); // the time can be interpreted in 24h format and also the "T" is just a string to separate date from time
    Wire.begin(); // important to initialize the I2C interface
}

void loop() {
  if (Serial.available() > 0){ // waits to the date is written on the buffer
        ISO = String(Serial.readString()); // read the buffer
        Serial.println(ISO); //Print the date sended for validation
        Serial.println(F("Insert the number of the day of the week, from 1 to 7"));
        while (Serial.available() == 0){
          }; // this while waits for the weekday to be sended by the serial console
		// next all the date format is manipulated and formated to Int for the RTC time set
        WeekDay = Serial.parseInt(); // gets the number for the week
        Year = ISO.substring(2,4).toInt(); // gets the Year from the ISO string
        Month = ISO.substring(4,6).toInt();// gets the Month from the ISO string
        Date = ISO.substring(6,8).toInt(); // gets the Day from the ISO string
        Hour = ISO.substring(9,11).toInt(); // gets the Hour from the ISO string
        Minute = ISO.substring(11,13).toInt(); // gets the Minute from the ISO string
        Second = ISO.substring(13,15).toInt(); // gets the Second from the ISO string

        Clock.setSecond(Second);//Set the second
        Clock.setMinute(Minute);//Set the minute
        Clock.setHour(Hour);  //Set the hour
        Clock.setDoW(WeekDay);    //Set the day of the week
        Clock.setDate(Date);  //Set the date of the month
        Clock.setMonth(Month);  //Set the month of the year
        Clock.setYear(Year);  //Set the year (Last two digits of the year)
        Serial.print("Time Set to: ");
        gettime(); //print the actual time of the RTC for validation
    };
  

}

void gettime(){
  bool century = false;
  bool h12Flag;
  bool pmFlag;
  byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
  bool alarmDy, alarmH12Flag, alarmPmFlag;

  Serial.print("2");
	if (century) {			// Won't need this for 89 years.
		Serial.print("1");
	} else {
		Serial.print("0");
	}
	Serial.print(Clock.getYear(), DEC);
	Serial.print(' ');
	
	// then the month
	Serial.print(Clock.getMonth(century), DEC);
	Serial.print(" ");
  
	// then the date
	Serial.print(Clock.getDate(), DEC);
	Serial.print(" ");
  
	// and the day of the week
	Serial.print(Clock.getDoW(), DEC);
	Serial.print(" ");
  
	// Finally the hour, minute, and second
	Serial.print(Clock.getHour(h12Flag, pmFlag), DEC);
	Serial.print(" ");
	Serial.print(Clock.getMinute(), DEC);
	Serial.print(" ");
	Serial.print(Clock.getSecond(), DEC);
 
	// Add AM/PM indicator
	if (h12Flag) {
		if (pmFlag) {
			Serial.print(" PM ");
		} else {
			Serial.print(" AM ");
		}
	} else {
		Serial.print(" 24h ");
	}
};
