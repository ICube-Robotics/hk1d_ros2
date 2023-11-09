#define CUSTOM
#include "ForceSensor.h"
#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

EasyCAT EASYCAT(8);                 // EasyCAT istantiation

//---- setup ---------------------------------------------------------------------------------------

void setup()

{

  Wire.begin(4);                // join i2c bus with address #4

  Wire.onReceive(receiveEvent); // register event

  Serial.begin(9600);           // start serial for output

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

    pinMode(13, OUTPUT);                                          // stay in loop for ever
                                                                  // with the Arduino led blinking
                                                          //
  }

}


void loop()
{

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()

void receiveEvent(int howMany)
{
  EASYCAT.MainTask();
  if(Wire.available()>=2)
  {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t value = (msb << 8) | lsb;
    EASYCAT.BufferIn.Cust.force_0 = (uint16_t) value;
    // Serial.println(value);
  }
  else
  {
    Serial.print("received less than 2 Bytes!");
  }
}
