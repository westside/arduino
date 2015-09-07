#include <Wire.h>

#define DRV2605_ADDRESS  0x5A

#define DRV2605_ADDR_RTP_INPUT		0x02
#define DRV2605_REG_GO  0x0C
#define DRV2605_ADDR_STATUS  0x00
#define DRV2605_ADDR_MODE  0x01
#define DRV2605_ADDR_RATED_VOLT  0x16
#define DRV2605_ADDR_OD_CLAMP  0x17
#define DRV2605_ADDR_FEEDBACK		0x1A
#define DRV2605_ADDR_CTRL1		0x1B
#define DRV2605_ADDR_CTRL2		0x1C
#define DRV2605_ADDR_CTRL3		0x1D


//ADXL345 Register Addresses
#define ADXL345_ADDRESS                 (0x53)
#define ADXL345_REG_DATA_FORMAT         (0x31)
#define ADXL345_REG_BW_RATE             (0x2C) 
#define ADXL345_REG_POWER_CTL           (0x2D)
#define ADXL345_REG_DATAX0              (0x32)
#define ADXL345_REG_DATAY0              (0x34)
#define ADXL345_REG_DATAZ0              (0x36)

#define DEBUG 1

enum MotorType {
  ERM,
  LRA
};


// 310 103 
float ratedVoltage = 3;
float ermRatedVoltage = ratedVoltage * 255 / 5.36;
float overdriveVoltage = 3.6;



MotorType motorType = LRA;
float frequency = 175;
float ratedRmsVoltage = 2;
float sampleTime = 0.0003; // 300 micro sec
float averageAbsVoltage = ratedRmsVoltage * sqrt(1-(4 * sampleTime + 0.0003) * frequency);
float peakVoltage = 3; //2.05;
uint8_t rV = round( averageAbsVoltage * 255 / 5.3);
uint8_t oV= round(peakVoltage * 255 / 5.6);


int x,y,z;
double xg, yg, zg;

String value = "";

unsigned long timeData = 0; 

int operationMode = 1; // 1 - real time, 2 - test

bool readEnable = false;
int intensity = 0;
int testIntensity = 0;
int upTriggerTime;
int downTriggerTime;
int samplingDuration;


unsigned long startMillis = 0;

void setup(){ 
  //Create a serial connection to display the data on the terminal.\
  Serial.begin(250000);

  // motor drive
  Wire.begin();
  Wire.setClock(400000);  // Arduino DUE: upto 400kHz (Atmel datasheet)
  
  auto_calibration();
    

  setup_lra_close_loop();
//  setup_lra_open_loop();
  set_RTP_input(0);  
  // ADXL345
  writeRegister(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, 0x01); // -4g~4g
  writeRegister(ADXL345_ADDRESS, ADXL345_REG_POWER_CTL, 0x08);  //Measurement mode
  writeRegister(ADXL345_ADDRESS, ADXL345_REG_BW_RATE, 0x0F); // sampling rate to 1600HZ;
}

void serialEvent() {
  
  if (Serial.available() > 0) {
    int mode = Serial.parseInt(); 
    if (mode == 1) {
      Serial.flush();
      operationMode = mode;
      while(Serial.available()) {
        Serial.read();
      }
    } else if (mode == 2) {
      testIntensity = Serial.parseInt();
      upTriggerTime = Serial.parseInt();
      downTriggerTime = Serial.parseInt();
      samplingDuration = Serial.parseInt();
      while(Serial.available()) {
        Serial.read();
      }
      
      operationMode = mode;
      
      startMillis = millis();
      readEnable = true;
    
    } 
  } 
}

void loop(){
  if (operationMode == 1) {
    // real time mode
    read_acc();
  } else if (operationMode == 2) {
    // test mode
    if (readEnable) {
      unsigned long elapsedTime = millis() - startMillis;
//      Serial.println(elapsedTime);
      unsigned long microsBefore = micros();
      if (elapsedTime > samplingDuration) {
        readEnable = false;
        if (intensity > 0) {
          set_RTP_input(0);
          intensity = 0;
        }
      } else if (elapsedTime > downTriggerTime) { 
        if (intensity > 0) {
          set_RTP_input(0x0);
          intensity = 0;
        }
      } else if (elapsedTime > upTriggerTime) {
        if (testIntensity != intensity) {
          set_RTP_input(testIntensity);
          intensity = testIntensity;
        }
      } 
      read_acc();
    } else {
      Serial.flush();
    }
   
  }   
}

void read_acc() {
      
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write((byte)ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 6);
    
    x = (short)((Wire.read() | (Wire.read() << 8))); 
    y = (short)((Wire.read() | (Wire.read() << 8))); 
   z = (short)((Wire.read() | (Wire.read() << 8)));  
  
  timeData = micros(); // String(timeData, DEC)
  Serial.println(String(timeData, DEC) + "," + String(intensity) + "," + String(x) + "," + String(y) + "," + String(z));
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

void setup_lra_close_loop() {
  Serial.println("LRA CLOSE LOOP");
  drvWriteRegister(DRV2605_ADDR_MODE, 0x05); // RTP MODE // default : 0x40
  
  drvWriteRegister(DRV2605_ADDR_CTRL1, 0x13); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL2, 0xF5); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL3, 0xA0); // default : A0
  
  // 0x1A : 0xB6(10110110)  (N_ERM_LRA-1, FB_BRAKE_FACTOR - 011, LOOP_GAIN - 01, BEMF_GAIN - 10)
  drvWriteRegister(DRV2605_ADDR_FEEDBACK, 0xB6); // LRA 
}

void setup_lra_open_loop() {
  Serial.println("LRA OPEN LOOP");
  drvWriteRegister(DRV2605_ADDR_MODE, 0x05); // RTP MODE // default : 0x40
  
  drvWriteRegister(DRV2605_ADDR_CTRL1, 0x13); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL2, 0xF5); // default : F5
  drvWriteRegister(DRV2605_ADDR_CTRL3, 0xA1); // default : A0
  
  // 0x1A : 0xB6(10110110)  (N_ERM_LRA-1, FB_BRAKE_FACTOR - 011, LOOP_GAIN - 01, BEMF_GAIN - 10)
  drvWriteRegister(DRV2605_ADDR_FEEDBACK, 0xB6); // LRA 
}

void RTP_ERM() {
  drvWriteRegister(DRV2605_ADDR_MODE, 0x05); // RTP MODE // default : 0x40
  
  // 0x1D : 10100000 (NG_THRESH - 10, ERM_OPEN_LOOP - 1, SUPPLY_COMP_DIS - 0, DATA_FORMAT_RTP - 0, LRA_DRIVE_MODE - 0, N_PWM_ANALOG-0, LRA_OPEN_LOOP - 0)
  drvWriteRegister(DRV2605_ADDR_CTRL3, 0xA0); // default : A0
  
  // 0x1A : 0x36(00110110)  (N_ERM_LRA-0, FB_BRAKE_FACTOR - 011, LOOP_GAIN - 01, BEMF_GAIN - 10)
  drvWriteRegister(DRV2605_ADDR_FEEDBACK, 0x36); // ERM  // default : 0x36
}

void set_RTP_input(uint8_t val) {
  drvWriteRegister(DRV2605_ADDR_RTP_INPUT, val);
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
