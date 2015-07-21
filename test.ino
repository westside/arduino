//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

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
int MOTOR_PIN = 3;


int count = 0;
String value = "";

const int SIZE = 1; // send to serial every 3 times
unsigned long timeData[SIZE] = {0}; 
int data[SIZE][3] = {{0,0,0}}; //, {0,0,0}, {0,0,0}};
bool readEnable = false;
int duration;
int sampleDuration;
int intensity = 0;
unsigned long startMillis = 0;
int delayMicros = 0;

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
  
  
  
  readRegister(DATAX0, 6, values);
}



void serialEvent() {
  if (Serial.available() > 0) {
    // get the new byte:
    Serial.flush();
    int mode = Serial.parseInt(); 
    if (mode == 2) {
      intensity = Serial.parseInt();
      duration = Serial.parseInt();
      sampleDuration = Serial.parseInt();
      delayMicros = Serial.parseInt();
//      Serial.flush();
      while(Serial.available()) {
        Serial.read();
      }
      #ifdef DEBUG
      Serial.print(millis());
      Serial.print(",");
      Serial.print(intensity);
      Serial.print(",");
      Serial.print(duration);
      Serial.print(",");
      Serial.println(sampleDuration);
      #endif      
      startMillis = millis();
      readEnable = true;
      readAcc();
      analogWrite(MOTOR_PIN, intensity);

    }
  } 
}
void loop(){
  
  if (readEnable) {
    unsigned long elapsedTime = millis() - startMillis;
//    Serial.println(elapsedTime);
    
    unsigned long microsBefore = micros();
    if (elapsedTime > sampleDuration) {
      readEnable = false;
    } else if (elapsedTime > duration) {
      if (intensity > 0) {
        analogWrite(MOTOR_PIN, 0);  
        intensity = 0;
      }
    } 
      readAcc();    
//      delayMicroseconds(1500);
//      Serial.println(micros() - microsBefore);
//      delayMicroseconds(delayMicros);
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
  
  
      int mode = count % SIZE;
      timeData[mode] = micros();
      data[mode][0] = x;
      data[mode][1] = y;
      data[mode][2] = z;
      count++;
      if (mode == SIZE-1) {
        unsigned long first = timeData[0];
        Serial.print(first,DEC);
        Serial.print(",");
        Serial.print(data[0][0]);
        Serial.print(",");
        Serial.print(data[0][1]);
        Serial.print(",");
        Serial.print(data[0][2]);
//        Serial.print(String(first, DEC) + "," + String(data[0][0]) + "," + String(data[0][1]) + "," + String(data[0][2]));
        for (int i = 1 ; i < SIZE ; i++) {
           Serial.print("," + String(timeData[i] - first, DEC) + "," + String(data[i][0]) + "," + String(data[i][1]) + "," + String(data[i][2]));
        }
        Serial.println();
      } 
      Serial.flush();
      
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
