#define CUSTOM
#include "ForceSensor.h"
#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library
#include <stdlib.h>

EasyCAT EASYCAT(8);                 // EasyCAT istantiation

const int Ana0 = A0;                // analog input 0
const int Ana1 = A1;                // analog input 1

unsigned long Millis = 0;
unsigned long PreviousMillis = 0;

uint8_t ind;
const uint8_t w = 3;
uint16_t  values[w] = {0};

//---- setup ---------------------------------------------------------------------------------------

void setup()
{

  Serial.begin(9600);                                             // serial line initialization
  Serial.print ("\nEasyCAT - Generic EtherCAT slave\n");          // print the banner

  // ADMUX |= (1 << REFS1) | (1 << REFS0);
  ind = 0;

                                                                  //---- initialize the EasyCAT board -----

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
    // while(1)                                                      //
    // {                                                             //
    //   digitalWrite (13, LOW);                                     //
    //   delay(500);                                                 //
    //   digitalWrite (13, HIGH);                                    //
    //   delay(500);                                                 //
    // }                                                             //
  }

  PreviousMillis = millis();
}


//---- main loop ----------------------------------------------------------------------------------------

void loop()                                             // In the main loop we must call ciclically the
{                                                       // EasyCAT task and our application
                                                        //
                                                        // This allows the bidirectional exachange of the data
                                                        // between the EtherCAT master and our application
                                                        //
                                                        // The EasyCAT cycle and the Master cycle are asynchronous
                                                        //

  // EASYCAT.MainTask();                                   // execute the EasyCAT task

  Application();                                        // user applications

}

//---- user application ------------------------------------------------------------------------------

void Application ()

{
  uint16_t Analog0;
  uint16_t Analog1;

  Millis = millis();                                    // As an example for this application
  if (Millis - PreviousMillis >= 10)                    // we choose a cycle time of 10 mS
  {                                                     //
    PreviousMillis = Millis;                            //

                                                        // --- analog inputs management ---

    Analog0 = analogRead(Ana0);                         // read analog input 0
    // Analog0 = median(Analog0) << 6 ;
    Analog0 = Analog0;
                              // normalize it on 16 bits
    // EASYCAT.BufferIn.Cust.force_0 = (uint16_t) Analog0;           // and put the result into
                                                        // input Byte 0

    Analog1 = analogRead(Ana1);                         // read analog input 1
    Analog1 = Analog1;                             // normalize it on 16 bits
    // EASYCAT.BufferIn.Cust.force_1 = Analog1;           // and put the result into

      Serial.println(Analog0);
     // Serial.println(Analog1);

  }
}

 uint16_t median(uint16_t x)
 {
   ind = (ind+1)%w;
   values[ind] = x;

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
