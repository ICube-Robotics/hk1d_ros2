#define CUSTOM
#include "ForceSensor.h"
// #include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
// #include <SPI.h>                    // SPI library
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

// EasyCAT EASYCAT(8);                 // EasyCAT istantiation

const int Ana0 = A0;                // analog input 0
const int Ana1 = A1;                // analog input 1
const int Ana5 = A5;
unsigned long Millis = 0;
unsigned long PreviousMillis = 0;

uint8_t ind;
const uint8_t w = 3;
uint16_t  values[w] = {0};

//---- setup ---------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);                                             // serial line initialization
  Serial.print ("\Starting Force Measurements\n");          // print the banner

  analogReference(1);  // 1=VCC , 2=1.1 Volt, 3 = Vin (Iguess)
  ind = 0;

  Wire.begin();

  PreviousMillis = millis();
  // Wire.begin(9);
}


//---- main loop ----------------------------------------------------------------------------------------

void loop()
{

  Application();                                        // user applications

}

//---- user application ------------------------------------------------------------------------------

void Application ()

{
  uint16_t Analog0;
  uint16_t Analog1;

  Millis = millis();                                    // As an example for this application
  if (Millis - PreviousMillis >= 1)                    // we choose a cycle time of 10 mS
  {                                                     //
    PreviousMillis = Millis;                            //

                                                        // --- analog inputs management ---

    Analog0 = analogRead(A0);                         // read analog input 0
    // Analog0 = median(Analog0) << 6 ;
    // Analog0 = (uint16_t) 100*(1+sin(Millis/1000.0 * 3.14));

    Wire.beginTransmission(4); // transmit to device #4

    uint8_t msb = Analog0 >> 8; // Most significant byte
    uint8_t lsb = Analog0 & 0xFF; // Least significant byte

    Wire.write(msb);              // sends one byte
    Wire.write(lsb);
    Wire.endTransmission();
                              // normalize it on 16 bits
    // EASYCAT.BufferIn.Cust.force_0 = (uint16_t) Analog0;           // and put the result into
                                               // input Byte 0
    // Analog0 = Millis;
    // Analog1 = analogRead(Ana1);                         // read analog input 1
    // Analog1 = Analog1;                             // normalize it on 16 bits
    // EASYCAT.BufferIn.Cust.force_1 = Analog1;           // and put the result into

    Serial.println(Analog0);
     // Serial.println(Analog1);

  }
}

//  uint16_t median(uint16_t x)
//  {
//    ind = (ind+1)%w;
//    values[ind] = x;

//     for (int i = 0; i < w - 1; i++) {
//               for (int j = i + 1; j < w; j++) {
//                   if (values[i] > values[j]) {
//                       int temp = values[i];
//                       values[i] = values[j];
//                       values[j] = temp;
//                   }
//               }
//     }
//   return values[w/2];
//  }
