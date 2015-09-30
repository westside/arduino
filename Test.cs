#include <HMC58X3.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
//#include "IMUduino.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>

// Adafruit nRF8001 Library
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
//      On Leo & compatible: CLK = 15, MISO = 14, MOSI = 16
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 7     // This should be an interrupt pin, on Uno thats #2 or #3. IMUduino uses D7
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t status = laststatus;

float ypr[3];
char chrData[17]; // Yaw (5 bytes), Pitch (5 bytes), Roll (5 bytes) ...delimeter is a pipe '|'
char sendbuffersize;
    
// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();


void setup() {
  
  Serial.begin(115200);
  
  while(!Serial); // Comment this out if you don't want to open up the Serial Monitor to start initialization
  
  Wire.begin();
  
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 + FreeIMU Print echo demo"));
  
  delay(500);
  // Initialize the IMU components.
  Serial.println(F("...Initializing IMU"));
  my3IMU.init(true);
  // Initialize the BLE component.
  Serial.println(F("...Initializing BTLE"));
  BTLEserial.begin();
  Serial.println(F("...Ok! Starting main loop."));
}


void loop() {
  my3IMU.getYawPitchRoll(ypr);
  Serial.print(ypr[0]); Serial.print(" "); Serial.print(ypr[1]); Serial.print(" "); Serial.println(ypr[2]);
  delay(20);
//  btleLoop();
//  if (status == ACI_EVT_CONNECTED) {
//    
//    
//    my3IMU.getYawPitchRoll(ypr);
//    
//    btleWriteYPR(ypr[0], ypr[1], ypr[2]);
//  }
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/

void btleLoop() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); 
      Serial.print(BTLEserial.available()); 
      Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }
  }
}


void btleWriteYPR(float Y, float P, float R) {
  dtostrf(Y, 1, 1, &chrData[0]);
  dtostrf(P, 1, 1, &chrData[6]);
  dtostrf(R, 1, 1, &chrData[11]);
  chrData[5] = '|';
  chrData[10] = '|';
  sendbuffersize = min(20, sizeof(chrData));
  
  BTLEserial.write((byte*)chrData, sendbuffersize);
}

byte * float2str(float arg) {
  // get access to the float as a byte-array:
  byte * data = (byte *) &arg;
  return data;
}
