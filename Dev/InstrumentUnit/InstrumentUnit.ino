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
// Second generation developement by Benjamín Santelices, Vicente Aitken, and José Ferrada
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

char ID[] = "007"; // Instrument ID for tracking, to be ported on config.h
static const uint32_t GPSBaud = 9600; // GPS software UART speed. To be hard-coded, as it does not change.
char dato; // ?

SFE_BMP180 pressure; // BMP180 object
TinyGPSPlus gps; // GPS object.
SoftwareSerial ss(3, 2); // Conexion serial para conectarse al GPS


#define CS_ADC 4 // ADC chip select.
#define CS_SD 8  //SD chip select. Matches hardware SPI bus implementation on 328P.


void setup()
{
  String zz, zz2, zz3, zz4;
  pinMode(A2,INPUT); //stop trigger from Tracker Unit init
  //debug UART, GPS softUART, BMP init
  Serial.begin(9600); 
  ss.begin(GPSBaud); 
  pressure.begin(); 

 //---MEASURING STARTS HERE---
  zz = ID;
  while (digitalRead(A2)== 0)
  {
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
  }
  else {
    //Serial.println(F("Error en Data.txt")); //WARN: good idea to implement some sort of debugging here.
  }
  delay(100);
}

void loop() //nothing happens here.
{
}

//---DATA ACQUISITION FUNCTIONS---

String data() //Sensor data processing and collation.
{
  const char coma = ','; //WARN: is this necessary?
  int readvalue;
  int data1, data2, data3, data4;
  String zz;
  unsigned long tiempo;
  tiempo = millis(); //WARN: why?
  data1 = 0;
  data2 = 0;
  data3 = 0;
  data4 = 0;
  //Sensor readout, keep highest value of each sensor.
  while (digitalRead(A2)) //Second check of A2 (?)
  {
    readvalue = read_ADC(1);

    if (data1 <= readvalue)
    {
      data1 = readvalue;
    }

    readvalue = read_ADC(2);

    if (data2 <= readvalue)
    {
      data2 = readvalue;
    }
    readvalue = read_ADC(3);

    if (data3 <= readvalue)
    {
      data3 = readvalue;
    }

    readvalue = read_ADC(4);

    if (data4 <= readvalue)
    {
      data4 = readvalue;
    }
  }
  zz = coma;
  zz += data1;
  zz += coma;
  zz += data2;
  zz += coma;
  zz += data3;
  zz += coma;
  zz += data4;
  //WARN: maybe some debugging here?
  return zz;
}

String GPS() //GPS data parsing and collation, hugely inneficient. To be replaced by straight NMEA communication.
{
  const char coma = ',';
  String zz;
  unsigned long tiempo;
  boolean latlong, diamesagno, hrminseg, alt;
  latlong = false;
  diamesagno = false;
  hrminseg = false;
  alt = false;

  tiempo = millis(); //El tiempo de incicio para marcar

  while (millis() < tiempo + 30000)
  {
    while (ss.available() > 0)
    {
      //Serial.print(F("Location: "));
      if (gps.encode(ss.read()))
      {
        if (gps.location.isValid() && latlong)
        {
          latlong = true;
        }

        if (gps.date.isValid())
        {
          diamesagno = true;
        }
        if (gps.time.isValid())
        {
          hrminseg = true;
        }
        //Serial.print(F(","));
        if (gps.altitude.isValid())
        {
          alt = true;
        }
        if (latlong && diamesagno && hrminseg && alt)
        {
          break;
        }
      }
    }
  }
  if (latlong)
  {
    zz = coma;
    zz += gps.location.lat();
    zz += coma;
    zz += gps.location.lng();
  }
  else
  {
    zz = coma;
    zz += coma;
  }

  //, Dia, Mes, Agno
  if (diamesagno)
  {
    zz += coma;
    zz += gps.date.day();
    zz += coma;
    zz += gps.date.month();
    zz += coma;
    zz += gps.date.year();

  }
  else
  {
    zz += coma;
    zz += coma;
    zz += coma;
  }

  //, Hora, Minuto, Segundo
  if (hrminseg)
  {
    zz += coma;
    zz += gps.time.hour();
    zz += coma;
    zz += gps.time.minute();
    zz += coma;
    zz += gps.time.second();
  }
  else
  {
    zz += coma;
    zz += coma;
    zz += coma;
  }

  //, Altura_GPS
  if (alt)
  {
    zz += coma;
    zz += gps.altitude.meters();
  }
  else
  {
    zz += coma;
  }
  return zz;
}


String BMP() //BMP180 data gathering. IC out of production, would be wise to replace.
{
  char status;
  const char coma = ',';
  String zz;
  double T, P, a;
  //,Temperatura
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
  }
  if (status != 0)
  {
    zz = coma;
    zz += T;

    //, Presion, Altura_BMP
    status = pressure.startPressure(3);
  }
  else
  {
    zz = coma;
  }

  if (status != 0)
  {
    delay(status);

    status = pressure.getPressure(P, T);
    if (status != 0)
    {
      const int P0 = 1013;
      zz += coma;
      zz += P;

      a = pressure.altitude(P, P0);
      zz += coma;
      zz += a;
    }
  }
  else
  {
    zz += coma;
    zz += coma;
  }
  return zz;
}

int read_ADC(int channel) { //ADC SPI interface

  int adcValue = 0;
  const int byte8 = 0x06; //setup byte
  int byte16 = (channel - 1) << 14; //bitshifted channel for second block.

  digitalWrite(CS_ADC, LOW); //select MCP3204
  SPI.transfer(byte8);
  adcValue = SPI.transfer16(byte16) & 0x0FFF; //ADC sample bitmasking.

  digitalWrite(CS_ADC, HIGH); //turn off device
  return adcValue;
}