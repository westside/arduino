#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;
int PWM_PIN = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("DRV test");
    drv.begin();
    drv.setMode(DRV2605_MODE_PWMANALOG);
    drv.writeRegister8(0x1A, 0x76);
    pinMode(PWM_PIN, OUTPUT);   // sets the pin as output
}

void loop() {
  // open loop p26 
  drv.writeRegister8(0x1D, 0x01); // N_PWM_ANALOG = 0, LRA_OPEN_LOOP=1
  drv.writeRegister8(0x1A, 0xB6);
  readRegister();
  Serial.println("open loop");
  beep();
  
  // close loop
  drv.writeRegister8(0x1D, 0x00); // N_PWM_ANALOG = 0, LRA_OPEN_LOOP=0 (auto resonance mode)
  drv.writeRegister8(0x1A, 0x36);
  readRegister();
  Serial.println("close loop");
  beep(); 
}
void readRegister() {
    Serial.print(drv.readRegister8(0x1A));
    Serial.print(" ");
    Serial.print(drv.readRegister8(0x1D));
    Serial.println();
}

void beep() {

   analogWrite(PWM_PIN, 130); 
   delay(1000);
  analogWrite(PWM_PIN, 50); 
   delay(1000);
   
}
