//#define DEBUG

//include Wire.h needed for I2C communication
#include <Wire.h>
#include <SoftwareSerial.h>
#define BL_RX 0
#define BL_TX 1

#define PCA9685_SLAVEADR1_OEbPIN 5
#define PCA9685_SLAVEADR1 0x60
#define PCA9685_GENERALCALLADR 0x0
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_SWRSTADR 0x6
#define PCA9685_ALLCALLADR 0xE

// PCA9685 register addresses
#define PCA9685_MODE1 0x0
#define PCA9685_MODE2 0x1
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6      // dec: 6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA    // dec: 250, same as 61th motor
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

//#define TRITRANSFORM_PARAM 0.366025403784439  // sqrt(2)*cosd(75)
#define TRITRANSFORM_PARAM 0.366  // sqrt(2)*cosd(75)

#define TIMEPOINT1    7450              // park waterglass
#define TIMEPOINT2    TIMEPOINT1+16867  // park rain
#define TIMEPOINT3    TIMEPOINT1+36400  // twilight hand
#define TIMEPOINT4    TIMEPOINT1+38800  // twilight handtohand
#define TIMEPOINT5    TIMEPOINT1+41300
#define TIMEPOINT6    TIMEPOINT1+63300
#define TIMEPOINT7    TIMEPOINT1+89600
#define TIMEPOINT8    1
#define TIMEPOINT9    1
#define TIMEPOINT10   1
SoftwareSerial blSerial =  SoftwareSerial(BL_RX, BL_TX);

const float INTENSITY_PARAM = 4/3;

const int numTimePoint = 10;
int timepointflag[numTimePoint] = {0};

float logvalue[] = {1, 3};
const float ln10 = log(10);
const float logDiff = log(logvalue[1] / logvalue[0]) / ln10;
int maxVoltage = 2458;         // 100% duty x 4096
int minPerceptVoltageThr = 1229; // 40% duty x 4096

const int maxNumMotor = 16;   // motorarraysize[0]*motorarraysize[1]
int motorArraySize[] = {4, 4}; // row x column
int numMotor = maxNumMotor;
int motorPoints[maxNumMotor][2] = {0};
int motorPlacement[maxNumMotor] = {0};
int activeMotor[maxNumMotor] = {0};
int motorVoltage[maxNumMotor] = {0};

int resolution[] = {11, 11};  // row x column, number of points between two adjacent motors (including themselves)
int blockidx[2] = {0};        // block index of the input point
float pointx = 0;
float pointy = 0;
float tmpValue = 0;

int startflag = 0;    // BOOL. check whether the stimulation is on going or not
int CWflag = 0;
int movieflag = 0;
const unsigned int dt = 10;

unsigned long startMillis = 0;
unsigned long pastMillis = 0;
unsigned long previousMillis = 0;

int randpattern[6][20]={{14,10,11,6,10,3,11,0,4,1},
                       {1,12,10,5,14,1,7,6,11,12},
                       {14,11,13,2,12,3,5,14,6,8},
                       {12,9,11,14,15,14,13,12,4,9},
                       {5,4,7,2,5,0,1,6,3,13},
                       {8,10,1,12,7,12,6,14,2,8}};
int randpatternNum=0;

int randpatternMovie[4][20]={{11,8,5,2,12,1,2,10,9,15,6,6,3,10,2,1,12,6,10,4},
                            {12,6,9,8,11,12,11,5,11,13,10,3,6,7,4,6,4,10,2,12},
                            {4,1,11,7,1,14,2,10,4,1,9,1,8,2,4,2,9,11,14,7},
                            {10,12,2,13,1,10,2,11,11,5,12,12,3,12,8,2,14,6,3,12}};

int circleCnt = 0;
float circlePeriod = 30;
float circlePts[2][20]={{0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1},
                       {0.5, 0.2, 0.1, 0.0417, 0.0101, 0, 0.0101, 0.0417, 0.1, 0.2, 0.5, 0.8, 0.9, 0.9583, 0.9899, 1, 0.9899, 0.9583, 0.9, 0.8}};

void setup() {
  blSerial.begin(9600);
  pinMode(BL_RX, INPUT);
  pinMode(BL_TX, OUTPUT);
  Wire.begin();

  Wire.setClock(400000);  // Arduino DUE: upto 400kHz (Atmel datasheet)
  write8(PCA9685_SLAVEADR1, PCA9685_MODE1, 0x0);  // init
  int pwmfreq = 1500;
  setPWMFreq(PCA9685_SLAVEADR1, pwmfreq);    //pwm frequency setting (40~1600Hz)
  setPin(PCA9685_SLAVEADR1, 61, 0);    // stop currently active motors
  startflag = 0;
  movieflag = 0;
  for (int i=0;i<numTimePoint;i++) 
    timepointflag[i]=0;  

  pinMode(PCA9685_SLAVEADR1_OEbPIN, OUTPUT);   // enable
  digitalWrite(PCA9685_SLAVEADR1_OEbPIN, LOW);
  motorPoints[0][0] = 0;  motorPoints[0][1] = 0;  motorPoints[1][0] = 1;  motorPoints[1][1] = 0;  motorPoints[2][0] = 2;  motorPoints[2][1] = 0;  motorPoints[3][0] = 3;  motorPoints[3][1] = 0;
  motorPoints[4][0] = 0;  motorPoints[4][1] = 1;  motorPoints[5][0] = 1;  motorPoints[5][1] = 1;  motorPoints[6][0] = 2;  motorPoints[6][1] = 1;  motorPoints[7][0] = 3;  motorPoints[7][1] = 1;
  motorPoints[8][0] = 0;  motorPoints[8][1] = 2;  motorPoints[9][0] = 1;  motorPoints[9][1] = 2;  motorPoints[10][0] = 2;  motorPoints[10][1] = 2;  motorPoints[11][0] = 3;  motorPoints[11][1] = 2;
  motorPoints[12][0] = 0;  motorPoints[12][1] = 3;  motorPoints[13][0] = 1;  motorPoints[13][1] = 3;  motorPoints[14][0] = 2;  motorPoints[14][1] = 3;  motorPoints[15][0] = 3;  motorPoints[15][1] = 3;

  motorPlacement[0] = 8;  motorPlacement[1] = 9;  motorPlacement[2] = 6;   motorPlacement[3] = 7;
  motorPlacement[4] = 10;   motorPlacement[5] = 11;   motorPlacement[6] = 4;    motorPlacement[7] = 5;
  motorPlacement[8] = 13;   motorPlacement[9] = 12;   motorPlacement[10] = 3;   motorPlacement[11] = 2;
  motorPlacement[12] = 15;  motorPlacement[13] = 14;  motorPlacement[14] = 1;  motorPlacement[15] = 0;

}

uint8_t readByte() {
  while (true) {
    uint8_t val = blSerial.read();
    if (val != 255) {
      return val;
    }
  }
}

uint8_t mode;
void loop() {
  while (blSerial.available() > 0) {      // when serial data is availabe (i.e. user command)
    uint8_t mode = blSerial.read();
    switch(mode) {   
      case 0:  
        setPin(PCA9685_SLAVEADR1, 61, 0);    // stop currently active motors
        startflag = 0;
        movieflag = 0;
        for (int i=0;i<numTimePoint;i++) 
          timepointflag[i]=0;          
        break;
      case 100:    
        maxVoltage=3686  * INTENSITY_PARAM;
        minPerceptVoltageThr=2458 * INTENSITY_PARAM;        
        startMillis = millis();
        previousMillis = startMillis;        
          movieflag = 1;        
        if (startflag == 0) {
          startflag = 1;
        } else {
          setPin(PCA9685_SLAVEADR1, 61, 0);
        }
        break;
      case 101: // ping pong
       blSerial.write(101); 
        break;
      default:
        break;
    }
  }
  if (startflag) {
    unsigned long currentMillis = millis();
    unsigned long dtMillis = currentMillis - previousMillis;
    if (dtMillis >= dt) {
      previousMillis = previousMillis + dtMillis;
      if (movieflag) {
        if ((!timepointflag[0])&&((previousMillis-startMillis)>TIMEPOINT1)) {
          // 7450
          // 80
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*0.4 * INTENSITY_PARAM); 
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*0.4  * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[9],4096*0.4  * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[10],4096*0.4 * INTENSITY_PARAM);
          delay(300);
          
          // 7750
          setPin(PCA9685_SLAVEADR1, 61, 0);
          delay(2700);
          
          // 10450
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*0.7 * INTENSITY_PARAM); // 140
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[9],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[10],4096*0.7 * INTENSITY_PARAM);        
          delay(400);
          
          // 10850
          setPin(PCA9685_SLAVEADR1, 61, 0);
          delay(5550);
          
          // 16400
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*0.55 * INTENSITY_PARAM); // 100
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*0.55 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[9],4096*0.55 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[10],4096*0.55 * INTENSITY_PARAM);
          delay(400);
          
          // 16800
          setPin(PCA9685_SLAVEADR1, 61, 0);
          delay(2450);
          
          // 19250
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*0.7 * INTENSITY_PARAM); // 140
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[9],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[10],4096*0.7 * INTENSITY_PARAM);        
          delay(400);
          
          // 19650
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          timepointflag[0]=1;
        } else if ((!timepointflag[1])&&((previousMillis-startMillis)>TIMEPOINT2)) {
          
          // 24317
          for (int i=0;i<20;i++) {
            for (int j=0;j<3;j++) {
              setPin(PCA9685_SLAVEADR1,motorPlacement[randpatternMovie[j][i]],4096*0.5 * INTENSITY_PARAM); // 100
            }
            delay(80);
            setPin(PCA9685_SLAVEADR1, 61, 0);                  
            delay(120);          
          }
          // 28137
          delay(20);
          
          // 28157
          // 1,2
          // 5,6
          // 2458 - 98
          // 1229 - 49
          for (int i=0;i<19;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt((float)i/19.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt((float)i/19.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/19.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/9.0));
            delay(20);
          }
          
          // 28537
          // 98
          // 49
          // 5,6
          // 8,9,10,11
          setPin(PCA9685_SLAVEADR1, 61, 0);
          for (int i=0;i<9;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[8],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt(1-(float)i/9.0));
            delay(40);
          }     
         // 28897   
         // 98
         // 49
         // 8,9,10,11
         // 12,13,14,15
          setPin(PCA9685_SLAVEADR1, 61, 0);
          for (int i=0;i<9;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[8],disttovolt((float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt((float)i/9.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[12],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt(1-(float)i/9.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[15],disttovolt(1-(float)i/9.0));
            delay(40);
          }        
          setPin(PCA9685_SLAVEADR1,motorPlacement[12],disttovolt(0));
          setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt(0));
          setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt(0));
          setPin(PCA9685_SLAVEADR1,motorPlacement[15],disttovolt(0));
          delay(40);   
     
         //  29297
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          timepointflag[1]=1;         
        } else if ((!timepointflag[2])&&((previousMillis-startMillis)>TIMEPOINT3)) {
          maxVoltage=2200 * INTENSITY_PARAM;         
          minPerceptVoltageThr=1100  * INTENSITY_PARAM;
          for (int i=0;i<4;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/4.0));
            delay(40);
          }
          setPin(PCA9685_SLAVEADR1, 61, 0);
          for (int i=0;i<4;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/4.0));
            delay(40);
          }
          setPin(PCA9685_SLAVEADR1, 61, 0);
          for (int i=0;i<4;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt(1-(float)i/4.0));
            delay(30);
          }
          setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt(0));
          setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt(0));
          delay(30);
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          timepointflag[2]=1;
        } else if ((!timepointflag[3])&&((previousMillis-startMillis)>TIMEPOINT4)) {
          maxVoltage=1500 * INTENSITY_PARAM;           
          minPerceptVoltageThr=900 * INTENSITY_PARAM;  
          for (int i=0;i<4;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[4],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[8],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt((float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/4.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/4.0));          
            delay(100);
          }
          setPin(PCA9685_SLAVEADR1, 61, 0);
          for (int i=0;i<8;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/7.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/7.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/7.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/7.0));
            delay(50);
          }
          setPin(PCA9685_SLAVEADR1, 61, 0);
          timepointflag[3]=1;
        } else if ((!timepointflag[4])&&((previousMillis-startMillis)>TIMEPOINT5)) {
          
          // 48750
          maxVoltage=1000 * INTENSITY_PARAM; // 40            
          minPerceptVoltageThr=800 * INTENSITY_PARAM; // 32       
          // 12,13,14,15
          // 8,9,10,11   
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[12],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[15],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[8],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt(1-(float)i/10.0));          
            delay(100);
          }
          
          // 49850
          maxVoltage=1100 * INTENSITY_PARAM;  // 44         
          minPerceptVoltageThr=900 * INTENSITY_PARAM; // 36        
          setPin(PCA9685_SLAVEADR1, 61, 0);
          // 8,9,10,11
          // 4,5,6,7
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[8],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[4],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt(1-(float)i/10.0));          
            delay(100);
          }
          
          // 50950
          setPin(PCA9685_SLAVEADR1, 61, 0);
          maxVoltage=1200 * INTENSITY_PARAM; // 48          
          minPerceptVoltageThr=900 * INTENSITY_PARAM; // 36   
          // 4,5,6,7
          // 0,1,2,3
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[4],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[0],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[3],disttovolt(1-(float)i/10.0));          
            delay(100);
          }
          
          // 52050
          maxVoltage=1300 * INTENSITY_PARAM;  // 52       
          minPerceptVoltageThr=1000 * INTENSITY_PARAM; // 40
          // 0,1,2
          // 5,6,7          
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[0],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt((float)i/10.0));
  //          setPin(PCA9685_SLAVEADR1,motorPlacement[3],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt(1-(float)i/10.0));
            delay(100);
          }
          
          // 53150
          maxVoltage=1400 * INTENSITY_PARAM; // 56           
          minPerceptVoltageThr=1000 * INTENSITY_PARAM; // 40
          // 5,6
          // 11          
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/10.0));
  //          setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt(1-(float)i/10.0));
            delay(100);
          }
          
          // 54250
          // 3,7,11
          // interval 80
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[3],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[7],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt((float)i/10.0));
            delay(80);
          } 
   
                
          setPin(PCA9685_SLAVEADR1, 61, 0);       
          delay(50);
          // 55180
          // 0,1,2
          for (int i=0;i<3;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[0],(1000+100*i)  * INTENSITY_PARAM);
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],(1000+100*i) * INTENSITY_PARAM);
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],(1000+100*i) * INTENSITY_PARAM);        
            delay(50);  
          }
          
          // 55330
          maxVoltage=1300 * INTENSITY_PARAM; // 52           
          minPerceptVoltageThr=900 * INTENSITY_PARAM; // 36
          // 0,1,2
          // 5,6,8
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[0],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt((float)i/10.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/10.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/10.0));                    
            delay(100);
          }
          
          // 56430
          // 5,6
          // 9,10,11
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/10.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt(1-(float)i/10.0));                    
            delay(100);
          }
          
          // 57530
          // 9,10,11
          // 13,14,15
          for (int i=0;i<11;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[11],disttovolt((float)i/10.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt(1-(float)i/10.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt(1-(float)i/10.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[15],disttovolt(1-(float)i/10.0));                    
            delay(100);
          }
          
          // 58630
          // 13,14,15
          for (int i=0;i<13;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt((float)i/12.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt((float)i/12.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[15],disttovolt((float)i/12.0));
            delay(100);
          }        
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          timepointflag[4]=1;
        } else if ((!timepointflag[5])&&((previousMillis-startMillis)>TIMEPOINT6)) {
          // 70750
          maxVoltage=3277 * INTENSITY_PARAM; // 131
          minPerceptVoltageThr=2458 * INTENSITY_PARAM; // 98 (200)    
          // 13,14
          // 9,10
          for (int i=0;i<7;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[13],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[14],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt(1-(float)i/6.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt(1-(float)i/6.0));                    
            delay(80);
            setPin(PCA9685_SLAVEADR1, 61, 0);
            delay(170);
          }
          
          // 72500
          // 9,10
          // 5,6
          for (int i=0;i<7;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[9],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[10],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt(1-(float)i/6.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt(1-(float)i/6.0));                    
            delay(80);
            setPin(PCA9685_SLAVEADR1, 61, 0);
            delay(170);
          }
          
          // 74250
          for (int i=0;i<7;i++) {
            setPin(PCA9685_SLAVEADR1,motorPlacement[5],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[6],disttovolt((float)i/6.0));
            setPin(PCA9685_SLAVEADR1,motorPlacement[1],disttovolt(1-(float)i/6.0));          
            setPin(PCA9685_SLAVEADR1,motorPlacement[2],disttovolt(1-(float)i/6.0));                    
            delay(80);
            setPin(PCA9685_SLAVEADR1, 61, 0);
            delay(170);
          }
          
          // 76000
          delay(130);
          
          // 76130
          setPin(PCA9685_SLAVEADR1,motorPlacement[8],4096*0.7 * INTENSITY_PARAM); // 115
          setPin(PCA9685_SLAVEADR1,motorPlacement[12],4096*0.7 * INTENSITY_PARAM); // 115
          delay(200);
          // 76330
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(420);
          
          // 76750
          setPin(PCA9685_SLAVEADR1,motorPlacement[11],4096*0.7 * INTENSITY_PARAM); // 115
          setPin(PCA9685_SLAVEADR1,motorPlacement[15],4096*0.7 * INTENSITY_PARAM);        
          delay(200);
          
          // 76950
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(400);
          
          // 77350
          setPin(PCA9685_SLAVEADR1,motorPlacement[1],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[2],4096*0.7 * INTENSITY_PARAM);        
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*0.7 * INTENSITY_PARAM);
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*0.7 * INTENSITY_PARAM);        
          delay(250);
          
          // 77600
          setPin(PCA9685_SLAVEADR1, 61, 0);
          timepointflag[5]=1;
        } else if ((!timepointflag[6])&&((previousMillis-startMillis)>TIMEPOINT7)) {
          // 97050
          setPin(PCA9685_SLAVEADR1,motorPlacement[1],4096*1.0);
          setPin(PCA9685_SLAVEADR1,motorPlacement[2],4096*1.0 );        
          delay(80);
          
          // 97130
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(90);   
    
          // 97220      
          setPin(PCA9685_SLAVEADR1,motorPlacement[5],4096*1.0);
          setPin(PCA9685_SLAVEADR1,motorPlacement[6],4096*1.0);        
          delay(80);
          
          // 97300
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(90);  
   
         // 97390       
          setPin(PCA9685_SLAVEADR1,motorPlacement[9],4096*1.0);
          setPin(PCA9685_SLAVEADR1,motorPlacement[10],4096*1.0);        
          delay(80);
          
          //97470
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(80);    
     
         // 97550     
          setPin(PCA9685_SLAVEADR1,motorPlacement[13],4096*1.0);
          setPin(PCA9685_SLAVEADR1,motorPlacement[14],4096*1.0);        
          delay(80);
          
          // 97630
          setPin(PCA9685_SLAVEADR1, 61, 0);        
          delay(80);                
          setPin(PCA9685_SLAVEADR1,motorPlacement[14],4096*1.0);
          setPin(PCA9685_SLAVEADR1,motorPlacement[15],4096*1.0);        
          delay(100);
          setPin(PCA9685_SLAVEADR1, 61, 0);
//          timepointflag[6]=1;  
          startflag = 0;
          movieflag = 0;
          for (int i=0;i<numTimePoint;i++) 
            timepointflag[i]=0;
          maxVoltage=2458;
          minPerceptVoltageThr=1229;            
        }
      } else {
        if ((previousMillis - pastMillis) > circlePeriod) {
          pastMillis = previousMillis;
          if (circleCnt > 20) {
            circleCnt = 0;
          } else if (circleCnt < 0) {
            circleCnt = 20;
          }
          pointx = circlePts[0][circleCnt];
          pointy = circlePts[1][circleCnt];        
          pointQuantization(&pointx, &pointy, 1);
          updateMotorVoltage(pointx, pointy, 1);
          if (CWflag) {
            circleCnt = circleCnt + 1;
          } else {
            circleCnt = circleCnt - 1;
          }
        }
      }
    }
  }  
}



// master -> slave data transmission func. (slave address, register address, data value for the register)
void write8(uint8_t addr, uint8_t regaddr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(regaddr);
  Wire.write(data);
  Wire.endTransmission();
}

// master <- slave data receiption func. (slave address, register address)
uint8_t read8(uint8_t addr, uint8_t regaddr) {
  Wire.beginTransmission(addr);
  Wire.write(regaddr);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)addr, (uint8_t)1);
  return Wire.read();
}

//PWM Frequency setting func. (slave address, freqency)
void setPWMFreq(uint8_t addr, float freq) {
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);


  uint8_t oldmode = read8(addr, PCA9685_MODE1); //read mode1 for selected slave
  uint8_t newmode = (oldmode & 0x7F) | 0x10; //sleep mode
  write8(addr, PCA9685_MODE1, newmode); //go to sleep
  write8(addr, PCA9685_PRESCALE, prescale); //set the prescaler
  write8(addr, PCA9685_MODE1, oldmode); //return to old mode
  delay(50);

  write8(addr, PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
}

void setPWM(uint8_t addr, uint8_t num, uint16_t on, uint16_t off) {
  Wire.beginTransmission(addr);
  Wire.write(LED0_ON_L + 4 * num);
  Wire.write(on);
  Wire.write(on >> 8);
  Wire.write(off);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void setPin(uint8_t addr, uint8_t num, uint16_t val) {
  if (val >= 4095) {
    // special value for signal fully on.
    setPWM(addr, num, 4096, 0);
  } else if (val == 0) {
    // special value for signal fully off.
    setPWM(addr, num, 0, 4096);
  } else {
    uint16_t tmpval = round(val / 2.0);
    setPWM(addr, num, 2047 - tmpval, 2047 + tmpval);
  }
}

void pointQuantization(float *ptx, float *pty, int modetype) {
  int i, j;
  float stepsize[] = {1.0 / (resolution[0] - 1), 1.0 / (resolution[1] - 1)};
  float tmpptx = *ptx;
  float tmppty = *pty;
  if (tmpptx < 0)  tmpptx = 0;
  else if (tmpptx > 1)  tmpptx = 1;
  if (tmppty < 0)  tmppty = 0;
  else if (tmppty > 1)  tmppty = 1;

  tmpptx = tmpptx * (motorArraySize[1] - 1);
  tmpptx = tmpptx - stepsize[1] / 2.0;
  tmppty = tmppty * (motorArraySize[0] - 1);
  tmppty = tmppty - stepsize[0] / 2.0;

  for (i = 0; i < ((resolution[1] - 1) * (motorArraySize[1] - 1)); i++) {
    if (tmpptx < 0)
      break;
    tmpptx = tmpptx - stepsize[1];
  }
  for (j = 0; j < ((resolution[0] - 1) * (motorArraySize[0] - 1)); j++) {
    if (tmppty < 0)
      break;
    tmppty = tmppty - stepsize[0];
  }

  blockidx[0] = i / (resolution[1] - 1);
  blockidx[1] = j / (resolution[0] - 1);
  if (blockidx[0] > (motorArraySize[1] - 2)) {
    blockidx[0] = (motorArraySize[1] - 2);
  }
  if (blockidx[1] > (motorArraySize[0] - 2)) {
    blockidx[1] = (motorArraySize[0] - 2);
  }
  if (modetype == 1) {
    tmpptx = i * stepsize[1];
    tmpptx = tmpptx - blockidx[0];
    tmppty = j * stepsize[0];
    tmppty = tmppty - blockidx[1];
    *ptx = (1 + TRITRANSFORM_PARAM) * tmpptx + TRITRANSFORM_PARAM * tmppty - TRITRANSFORM_PARAM;
    *pty = (1 + TRITRANSFORM_PARAM) * tmppty + TRITRANSFORM_PARAM * tmpptx - TRITRANSFORM_PARAM;
  } else {
    *ptx = i * stepsize[1];
    *pty = j * stepsize[0];
  }
}

void updateMotorVoltage(float x, float y, int modetype) {
  float motordists[maxNumMotor] = {0};
  int motorChanged = 0;
  int tmpmotoridx = 0;
  if (modetype == 0) {
    for (int i = 0; i < numMotor; i++) {
      motordists[i] = sqrt((motorPoints[i][0] - x) * (motorPoints[i][0] - x) + (motorPoints[i][1] - y) * (motorPoints[i][1] - y));
      motorVoltage[i] = disttovolt(motordists[i]);
      if (motorVoltage[i] > 0) {
        if (activeMotor[i] == 0)
          motorChanged = 1;
        activeMotor[i] = 1;
      } else {
        if (activeMotor[i] == 1)
          motorChanged = 1;
        activeMotor[i] = 0;
      }
    }
  } else if (modetype == 1) {
    for (int i = 0; i < numMotor; i++)
      motorVoltage[i] = 0;
    tmpmotoridx = blockidx[0] + blockidx[1] * motorArraySize[1];
    motordists[tmpmotoridx] = sqrt((-TRITRANSFORM_PARAM - x) * (-TRITRANSFORM_PARAM - x) + (-TRITRANSFORM_PARAM - y) * (-TRITRANSFORM_PARAM - y)) / sqrt(2);
    motordists[tmpmotoridx + motorArraySize[1] + 1] = sqrt((1 + TRITRANSFORM_PARAM - x) * (1 + TRITRANSFORM_PARAM - x) + (1 + TRITRANSFORM_PARAM - y) * (1 + TRITRANSFORM_PARAM - y)) / sqrt(2);
    if (motordists[tmpmotoridx + motorArraySize[1] + 1] >= motordists[tmpmotoridx]) {
      motorVoltage[tmpmotoridx] = disttovolt(motordists[tmpmotoridx]);
    } else {
      motorVoltage[tmpmotoridx + motorArraySize[1] + 1] = disttovolt(motordists[tmpmotoridx + motorArraySize[1] + 1]);
    }
    motordists[tmpmotoridx + 1] = sqrt((1.0 - x) * (1.0 - x) + y * y) / sqrt(2);
    motorVoltage[tmpmotoridx + 1] = disttovolt(motordists[tmpmotoridx + 1]);
    motordists[tmpmotoridx + motorArraySize[1]] = sqrt((1.0 - y) * (1.0 - y) + x * x) / sqrt(2);
    motorVoltage[tmpmotoridx + motorArraySize[1]] = disttovolt(motordists[tmpmotoridx + motorArraySize[1]]);
    for (int i = 0; i < numMotor; i++) {
      if (motorVoltage[i] > 0) {
        if (activeMotor[i] == 0)
          motorChanged = 1;
        activeMotor[i] = 1;
      } else {
        if (activeMotor[i] == 1)
          motorChanged = 1;
        activeMotor[i] = 0;
      }
    }
  }


  if (motorChanged) {
    setPin(PCA9685_SLAVEADR1, 61, 0);
  }
  for (int i = 0; i < numMotor; i++) {
    if (activeMotor[i] == 1) {
      setPin(PCA9685_SLAVEADR1, motorPlacement[i], motorVoltage[i]);
    }
  }
}

float disttovolt(float distance) {
  if (distance >= 1) {
    return 0;
  } else {
    distance = 1 - distance;
    distance = distance * (logvalue[1] - logvalue[0]) + logvalue[0];
    return (log(distance / logvalue[0]) / ln10 / logDiff * (maxVoltage - minPerceptVoltageThr) + minPerceptVoltageThr);
  }
}

void tritransform(float *ptx, float *pty) {
  float tmpptx = *ptx - blockidx[0];
  float tmppty = *pty - blockidx[1];
  *ptx = (1 + TRITRANSFORM_PARAM) * tmpptx + TRITRANSFORM_PARAM * tmppty - TRITRANSFORM_PARAM;
  *pty = (1 + TRITRANSFORM_PARAM) * tmppty + TRITRANSFORM_PARAM * tmpptx - TRITRANSFORM_PARAM;
}
