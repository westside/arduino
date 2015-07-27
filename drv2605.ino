//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <Wire.h>

//Assign the Chip Select signal to pin 10.
int CS=10;

//ADXL345 Register Addresses
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status

#define DEBUG 1


//This buffer will hold values read from the ADXL345 registers.
char values[10];
char output[20];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;
char tapType=0;
int intType;
int MOTOR_PIN = 5;

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
  
  pinMode(MOTOR_PIN, OUTPUT);
  
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.\
  Serial.begin(1843200);
//  Serial.begin(921600);
//  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Create an interrupt that will trigger when a tap is detected.
  attachInterrupt(0, tap, RISING);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Send the Tap and Double Tap Interrupts to INT1 pin
  writeRegister(INT_MAP, 0x9F);
  //Look for taps on the Z axis only.
  writeRegister(TAP_AXES, 0x01);
  //Set the Tap Threshold to 3g
  writeRegister(THRESH_TAP, 0x38);
  //Set the Tap Duration that must be reached
  writeRegister(DURATION, 0x10);
  
  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, 0x50);
  writeRegister(WINDOW, 0xFF);
  
  //Enable the Single and Double Taps.
  writeRegister(INT_ENABLE, 0xE0);  
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.
  
  writeRegister(BW_RATE, 0x0F); // sampling rate;
  
  readRegister(DATAX0, 6, values);
  
  // motor drive
  Wire.begin();
  writeI2CRegister8(0x01, 0x05); // RTP MODE 
  analogWrite(MOTOR_PIN, 0x00); 
//  writeI2CRegister8(0x16, 0xff);
writeI2CRegister8(0x1D, 0xA0);
//  writeI2CRegister8(0x1D, 0xA8); // readI2CRegister8(0x1D) | 0x08);
//  writeI2CRegister8(0x02, 0x7F);
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

void setMotorValue(uint8_t value) {
  writeI2CRegister8(0x02, value);
}

void loop(){
  if (operationMode == 1) {
    // real time mode
    readAcc();
  } else if (operationMode == 2) {
    // test mode
    if (readEnable) {
      unsigned long elapsedTime = millis() - startMillis;
//      Serial.println(elapsedTime);
      unsigned long microsBefore = micros();
      if (elapsedTime > samplingDuration) {
        readEnable = false;
        if (intensity > 0) {
          setMotorValue(0);
          intensity = 0;
        }
      } else if (elapsedTime > downTriggerTime) { 
        if (intensity > 0) {
          setMotorValue(0);
          intensity = 0;
        }
      } else if (elapsedTime > upTriggerTime) {
        if (testIntensity != intensity) {
          setMotorValue(testIntensity);
          intensity = testIntensity;
        }
      } 
      readAcc();
    } else {
      Serial.flush();
    }
   
  } 
}

void readAcc() {
      //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
      //The results of the read operation will get stored to the values[] buffer.
      readRegister(DATAX0, 6, values);
    
      //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
      //The X value is stored in values[0] and values[1].
      x = ((int)values[1]<<8)|(int)values[0];
      //The Y value is stored in values[2] and values[3].
      y = ((int)values[3]<<8)|(int)values[2];
      //The Z value is stored in values[4] and values[5].
      z = ((int)values[5]<<8)|(int)values[4];
      
      //Convert the accelerometer value to G's. 
      //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
      // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  //    xg = x * 0.0078;;
  //    yg = y * 0.0078;
  //    zg = z * 0.0078;
  
  
      timeData = micros();
      Serial.println(String(timeData, DEC) + "," + String(intensity) + "," + String(x) + "," + String(y) + "," + String(z));
      
//      delay(3);
      
}


//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

void tap(void){
  //Clear the interrupts on the ADXL345
  readRegister(INT_SOURCE, 1, values); 
  if(values[0] & (1<<5))tapType=2;
  else tapType=1;;
}

void writeI2CRegister8(uint8_t reg, uint8_t val) {
   // use i2c
    Wire.beginTransmission(0x5A); // DRV2605_ADDR
    Wire.write((byte)reg);
    Wire.write((byte)val);
    Wire.endTransmission();
}

uint8_t readI2CRegister8(uint8_t reg) {
    uint8_t x ;
   // use i2c
    Wire.beginTransmission(0x5A);
    Wire.write((byte)reg);
    Wire.endTransmission();
    Wire.requestFrom(0x5A, (byte)1);
    x = Wire.read();
    return x;
}
