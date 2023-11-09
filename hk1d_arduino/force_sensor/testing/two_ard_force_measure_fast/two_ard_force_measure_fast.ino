// This version is used to measure the force with high frequency and take the average (it should be uploaded to the arduino with force sensor, and then we should connect both arduinos with I2C)

#define CUSTOM
#include "ForceSensor.h"
// #include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
// #include <SPI.h>                    // SPI library
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

// EasyCAT EASYCAT(8);                 // EasyCAT istantiation

const uint8_t AnalogPin = A1;                // A1 compares to 1.1 Volt, A0 compares to 1 Volt
unsigned long CurrentMicros = 0;
unsigned long PreviousMicros = 0;

uint8_t ind;
short unsigned int Analog0;
//---- setup ---------------------------------------------------------------------------------------

void setup()
{
TCCR1B = (TCCR1B & 0xF8) | 0x01; // Set prescaler to 1 (no prescaling), this makes the arduino clock run faster
  Serial.begin(9600);                                             // serial line initialization
  Serial.print ("\Starting Force Measurements\n");          // print the banner

  analogReference(2);  // 1=VCC , 2=1.1 Volt,
  ind = 0;

  Wire.begin();

  PreviousMicros = micros();
  // Wire.begin(9);
}


//---- main loop ----------------------------------------------------------------------------------------

void loop()
{
  short unsigned int  i = 0;
  uint16_t SUM = 0;
  for(i=0;i<50;i++)
  {
    while(micros() - PreviousMicros < 5)
    {
    }
    PreviousMicros = micros();
    SUM = SUM + analogRead(A1);
  }
  Analog0 = SUM/50;
  CurrentMicros = micros();

  Wire.beginTransmission(4); // transmit to device #4
  uint8_t msb = Analog0 >> 8; // Most significant byte
  uint8_t lsb = Analog0 & 0xFF; // Least significant byte

  Wire.write(msb);              // sends one byte
  Wire.write(lsb);
  Wire.endTransmission();
}

//---- user application ------------------------------------------------------------------------------

//uint16_t median(uint16_t x)
//  {
//    ind = (ind+1)%w;
//    values[ind] = x;
//
//     for (int i = 0; i < w - 1; i++) {
//               for (int j = i + 1; j < w; j++) {
//                   if (values[i] > values[j]) {
//                      int temp = values[i];
//                      values[i] = values[j];
//                      values[j] = temp;
//                   }
//              }
//     }
//   return values[w/2];
//  }
