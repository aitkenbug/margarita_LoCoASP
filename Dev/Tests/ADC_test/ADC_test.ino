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

#define CS_ADC 4 // ADC chip select.

#define SELPIN 4 //Pin de activación del ADC
#define DATAOUT 11 // MOSI
#define DATAIN 12 // MISO
#define SPICLOCK 13 // CLK
#define CTRL_Z 26 // termino

void setup() {
    pinMode(CS_ADC, OUTPUT);
    Serial.begin(115200);
    delay(1500);
    Serial.println(F("Testing the ADC MCP3204-BVSL..."));
    SPI.begin();
}

void loop() {
    Serial.print(F("ADC CH1: "));
    Serial.print(read_ADC(1));
    Serial.print(F("  CH2: "));
    Serial.print(read_ADC(2));
    Serial.print(F("  CH3: "));
    Serial.print(read_ADC(3));
    Serial.print(F("  CH4: "));
    Serial.println(read_ADC(4));
    delay(100);
}

String data() {
    //Sensor data processing and collation.
    int readvalue = 0, data1 = 0, data2 = 0, data3 = 0, data4 = 0;
    char zz[30];
    //Sensor readout, keep highest value of each sensor.
    while (digitalRead(A2)) {//Second check of A2 (?)
        readvalue = read_ADC(1);
        if (data1 <= readvalue) {
            data1 = readvalue;
        }

        readvalue = read_ADC(2);
        if (data2 <= readvalue) {
            data2 = readvalue;
        }

        readvalue = read_ADC(3);
        if (data3 <= readvalue) {
            data3 = readvalue;
        }

        readvalue = read_ADC(4);
        if (data4 <= readvalue) {
            data4 = readvalue;
        }
    }
    sprintf(zz, ",%d,%d,%d,%d", data1, data2, data3, data4);
    Serial.print(F("Sensor data: "));
    Serial.println(zz);
    return zz;
}
/*
int read_ADC(int channel) {
    byte adcPrimaryConfig =0x06; //setup byte
    byte adcSecondaryConfig = channel << 6;
    byte adcPrimaryByteMask = 0b00001111;      // b00001111 isolates the 4 LSB for the value returned. 
    noInterrupts(); // disable interupts to prepare to send address data to the ADC.  
    digitalWrite(CS_ADC, LOW); // take the Chip Select pin low to select the ADC.
    SPI.transfer(adcPrimaryConfig); //  send in the primary configuration address byte to the ADC.  
    byte adcPrimaryByte = SPI.transfer(adcSecondaryConfig); // read the primary byte, also sending in the secondary address byte.  
    byte adcSecondaryByte = SPI.transfer(0x00); // read the secondary byte, also sending 0 as this doesn't matter. 
    digitalWrite(CS_ADC, HIGH); // take the Chip Select pin high to de-select the ADC.
    interrupts(); // Enable interupts.
    adcPrimaryByte &= adcPrimaryByteMask; // Limits the value of the primary byte to the 4 LSB:
    int digitalValue = (adcPrimaryByte << 8) | adcSecondaryByte; // Shifts the 4 LSB of the primary byte to become the 4 MSB of the 12 bit digital value, this is then ORed to the secondary byte value that holds the 8 LSB of the digital value.
    return digitalValue; // Returns the value from the function
}
*/

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

/*
int read_ADC(int channel) {
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  //allow channel selection
  commandbits |= ((channel - 1) << 3);

  digitalWrite(SELPIN, LOW); //Select adc
  // setup bits to be written
  for (int i = 7; i >= 3; i--) {
    digitalWrite(DATAOUT, commandbits & 1 << i);
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  digitalWrite(SPICLOCK, HIGH);   //ignores 2 null bits
  digitalWrite(SPICLOCK, LOW);
  digitalWrite(SPICLOCK, HIGH);
  digitalWrite(SPICLOCK, LOW);

  //read bits from adc
  for (int i = 11; i >= 0; i--) {
    adcvalue += digitalRead(DATAIN) << i;
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }
  digitalWrite(SELPIN, HIGH); //turn off device
  return adcvalue;
}
*/
