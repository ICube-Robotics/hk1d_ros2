// This version is used to measure the force with high frequency and take the average (it should be uploaded to the arduino with force sensor, and then we should connect both arduinos with I2C)

#define CUSTOM
#include "ForceSensor.h"
#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

EasyCAT EASYCAT(8);                 // EasyCAT istantiation

//const uint8_t AnalogPin = A1;          /      // A1 compares to 1.1 Volt, A0 compares to 1 Volt
//unsigned long CurrentMicros = 0;
unsigned long PreviousMicros = 0;

uint8_t ind;
const uint8_t w = 1;
short unsigned int val;
short unsigned int values[w]={0};
//---- setup ---------------------------------------------------------------------------------------

void setup()
{
TCCR1B = (TCCR1B & 0xF8) | 0x01; // Set prescaler to 1 (no prescaling), this makes the arduino clock run faster
  Serial.begin(9600);                                             // serial line initialization
  Serial.print ("\Starting Force Measurements\n");          // print the banner

  analogReference(1);  // 1=VCC , 2=1.1 Volt,
  ind = 0;
  PreviousMicros = micros();


  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // successfully completed
    Serial.print ("initialized");                                 //
  }                                                               //

  else                                                            // initialization failed
  {                                                               // the EasyCAT board was not recognized
    Serial.print ("initialization failed");                       //
                                                                  // The most common reason is that the SPI
                                                                  // chip select chosen on the board doesn't
                                                                  // match the one chosen by the firmware
  }
}


//---- main loop ----------------------------------------------------------------------------------------

void loop()
{

  EASYCAT.MainTask();
  short unsigned int  i = 0;
  uint32_t SUM = 0;

  for(i=0;i<50;i++)
  {
    // while(micros() - PreviousMicros < 1)
    // {
    // }
    PreviousMicros = micros();
    // SUM = SUM + median(analogRead(A0));
    SUM = SUM + analogRead(A0);
  }
  val = SUM/50;
  PreviousMicros = micros();

  EASYCAT.BufferIn.Cust.force_0 = (uint16_t) val;

  // Serial.println(val);

}

//---- user application ------------------------------------------------------------------------------

uint16_t median(uint16_t x)
 {
   ind = (ind+1)%w;
   values[ind] = x;
    //sort the matrix
    for (int i = 0; i < w - 1; i++) {
              for (int j = i + 1; j < w; j++) {
                  if (values[i] > values[j]) {
                     int temp = values[i];
                     values[i] = values[j];
                     values[j] = temp;
                  }
             }
    }
  return values[w/2];
 }
