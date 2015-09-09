#include <Wire.h>

#define PIN_DECODE_D0  4  
#define PIN_DECODE_D1  5
#define PIN_DECODE_D2  6
#define PIN_DECODE_D3  7

#define PIN_DECODER_EN  8

#define DRV2605_ADDRESS  0x5A
#define DRV2605_ADDR_RTP_INPUT  0x02
#define DRV2605_REG_GO  0x0C
#define DRV2605_ADDR_STATUS  0x00
#define DRV2605_ADDR_MODE  0x01
#define DRV2605_ADDR_RATED_VOLT  0x16
#define DRV2605_ADDR_OD_CLAMP  0x17
#define DRV2605_ADDR_FEEDBACK  0x1A
#define DRV2605_ADDR_CTRL1  0x1B
#define DRV2605_ADDR_CTRL2  0x1C
#define DRV2605_ADDR_CTRL3  0x1D


float frequency = 175;
float rated_rms_volt = 2;
float sample_time = 0.0003; // 300 us
float avg_abs_volt = rated_rms_volt * sqrt(1-(4 * sample_time + 0.0003) * frequency);
float peak_volt = 2.05; //2.05;
uint8_t rV = round(avg_abs_volt * 255 / 5.3);
uint8_t oV= round(peak_volt * 255 / 5.6);


void setup() {
  Wire.begin();
  Wire.setClock(400000);  
  
  Serial.begin(9600);
  
  pinMode(PIN_DECODE_D0, OUTPUT);
  pinMode(PIN_DECODE_D1, OUTPUT);
  pinMode(PIN_DECODE_D2, OUTPUT);
  pinMode(PIN_DECODE_D3, OUTPUT);
  
  pinMode(PIN_DECODER_EN, OUTPUT);
  
  enable_decoder(true);
  
  for (int i = 0 ; i < 16 ; i++) {
    select(i);
    
    auto_calibration();
    setup_lra_close_loop();
    set_RTP_input(0);  
  }
  
   select(7);
   set_RTP_input(100);
}

void loop() {

  for (int i = 0 ; i < 16 ; i++) {
    select(i);
    set_RTP_input(100);
    delay(100);  
    set_RTP_input(0);
    delay(100);  
  }
}

void setup_lra_close_loop() {
  Serial.println("LRA CLOSE LOOP");
  drvWriteRegister(DRV2605_ADDR_MODE, 0x05); // RTP MODE // default : 0x40
  
  drvWriteRegister(DRV2605_ADDR_CTRL1, 0x13); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL2, 0xF5); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL3, 0xA0); // default : A0
  
  // 0x1A : 0xB6(10110110)  (N_ERM_LRA-1, FB_BRAKE_FACTOR - 011, LOOP_GAIN - 01, BEMF_GAIN - 10)
  drvWriteRegister(DRV2605_ADDR_FEEDBACK, 0xB6); // LRA 
}

void auto_calibration() {
  uint8_t currentMode =  drvReadRegister(DRV2605_ADDR_MODE);
  Serial.print("current mode : ");
  Serial.println(currentMode);
  // mode to auto-calibration
  drvWriteRegister(DRV2605_ADDR_MODE, 0x07);  

  // register setting
  Serial.print("rated voltage : ");
  Serial.println(rV, HEX);
  drvWriteRegister(DRV2605_ADDR_RATED_VOLT, rV);  
  
  Serial.print("overdrive voltage : ");
  Serial.println(oV, HEX);
  drvWriteRegister(DRV2605_ADDR_OD_CLAMP, oV);  
  
  Serial.println("Feedback Control : B6 (LRA, Brake Factor : 4x, Loop Gain: medium, BEMFGain 1.8)"); 
  drvWriteRegister(DRV2605_ADDR_FEEDBACK, 0xB6);  

  Serial.println("ADDR_CTRL1 : 0x93"); 
  drvWriteRegister(DRV2605_ADDR_CTRL1, 0x93);  
  
  Serial.println("ADDR_CTRL2 : 0xF5 (default)"); 
  drvWriteRegister(DRV2605_ADDR_CTRL2, 0xF5);  
  
  Serial.println("ADDR_CTRL3 : 0x80 (default)"); 
  drvWriteRegister(DRV2605_ADDR_CTRL3, 0x80);  

  // Set the GO bit (write 0x01 to register 0x0C)
  drvWriteRegister(DRV2605_REG_GO, 0x01);
  // polling
  Serial.println("auto calibration completion polling start.");
  
  uint8_t isComplete = drvReadRegister(DRV2605_REG_GO);  
  while (isComplete != 0) {
    Serial.println("calibration undone. check again after 200ms.");
    delay(200);
    isComplete = drvReadRegister(DRV2605_REG_GO);  
  }
  Serial.print("calibration done : ");
  Serial.println(isComplete);
  
  uint8_t staus = drvReadRegister(DRV2605_ADDR_STATUS) & 0x04;  // DIAG_RESULT check
  if (staus == 0) {
    // success
    Serial.print("calibration : "); Serial.print(staus); Serial.println(" success");
  } else { 
    // fail
    Serial.print("calibration : "); Serial.print(staus); Serial.println(" fail");
  }
  
  // back to previus mode
  Serial.print("back to mode : ");
  Serial.println(currentMode);
  drvWriteRegister(DRV2605_ADDR_MODE, currentMode);  
}

void set_RTP_input(uint8_t val) {
  drvWriteRegister(DRV2605_ADDR_RTP_INPUT, val);
}

void enable_decoder(bool val) {
  digitalWrite(PIN_DECODER_EN, val ? LOW : HIGH);  
}

void select(uint8_t value) {
  selectDigitalWrite(PIN_DECODE_D0, value&1);
  selectDigitalWrite(PIN_DECODE_D1, value&2);
  selectDigitalWrite(PIN_DECODE_D2, value&4);
  selectDigitalWrite(PIN_DECODE_D3, value&8);
}

void selectDigitalWrite(int pin, uint8_t value) {
    if (value == 0x0) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

void drvWriteRegister(uint8_t reg, uint8_t val) {
  writeRegister(DRV2605_ADDRESS, reg, val);
}

void writeRegister(uint8_t device, uint8_t reg, uint8_t val){
    Wire.beginTransmission(device);
    Wire.write((byte)reg);
    Wire.write((byte)val);
    Wire.endTransmission();
}

uint8_t drvReadRegister(uint8_t reg) {
  return readRegister(DRV2605_ADDRESS, reg);
}

uint8_t readRegister(uint8_t device, uint8_t reg) {
   // use i2c
    Wire.beginTransmission(device);
    Wire.write((byte)reg);
    Wire.endTransmission();
    Wire.requestFrom(device, (byte)1);
    uint8_t x = Wire.read();
    return x;
}
