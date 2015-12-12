#ifndef QUADARDU
#define QUADARDU

#include <Servo.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <Wire.h>



//#define DEBUG
#define DEBUGMODE
#define DEBUGYPR
//#define DEBUGPID
//#define DEBUGRADIO
//#define DEBUGSPEED
//#define DEBUGFUNCTIONS


#define CE_PIN   9
#define CSN_PIN 10

#define ESC_FL 5
#define ESC_RL 4
#define ESC_FR 7
#define ESC_RR 6

#define ESC_MIN 30
#define ESC_MAX 140
#define ESC_TAKEOFF_OFFSET 50
#define ESC_ARM_DELAY 3000

#define RC_THROTTLE_MAX 255
#define RC_THROTTLE_MIN 0

#define PITCH_P_VAL 0.13
#define PITCH_I_VAL 0
#define PITCH_D_VAL 0.005

#define ROLL_P_VAL 0.13
#define ROLL_I_VAL 0
#define ROLL_D_VAL 0.005

#define YAW_P_VAL 0.13
#define YAW_I_VAL 0
#define YAW_D_VAL 0.005

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 15
#define PID_ROLL_INFLUENCE 15
#define PID_YAW_INFLUENCE 15
#define PITCH_CORRECTION 4.5


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object

float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};


const uint64_t pipe = 0xE8E8F0F0E1LL;
//const uint64_t pipe1 = 0xF0F0F0F0D2LL;
RF24 radio(CE_PIN, CSN_PIN);

float remote[7] = {0,0,0,0};
float remoteLast[7];
int velocity;                          // global velocity
int velocityLast;
float bal_roll, bal_pitch;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
Servo fr,fl,rr,rl;


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
Quaternion q; 
volatile bool mpuInterrupt = false; 
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];
int time;

PID pitchReg(&ypr[1], &bal_pitch, &remote[2], PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, DIRECT);
PID rollReg(&ypr[2], &bal_roll, &remote[1], ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, DIRECT);
PID yawReg(&ypr[0], &bal_axes, &remote[3], YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
    #ifdef DEBUGMODE                          // Device tests go here
  Serial.begin(115200);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
  Serial.println("EOS!");
  #endif
  radioSetup();
  initESCs();
  initMPU();
  initBalancing();
  initRegulators();
  
  

}

/* loop function
 *
 */

void loop(){
  time = millis();
  if (!dmpReady) return;
  while(!mpuInterrupt && fifoCount < packetSize) {
   // #ifdef DEBUGYPR
    //Serial.println("Waiting for DMP");
   // #endif
  }
    #ifdef DEBUGFUNCTIONS
    Serial.println("LOOP!");
    #endif
    rx();
    getYPR();
    computePID();
    calculateVelocities();
    updateMotors(); 
  //Serial.println(millis() - time);
}

void initMPU() {
  Wire.begin();
  TWBR = 24; 
  mpu.initialize();
  mpu.testConnection();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();    
  }
  else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void radioSetup() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  
  radio.enableDynamicPayloads();
  //radio.setRetries(15,15);
 // radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(1,pipe);
  //radio.openWritingPipe(pipe1);
  radio.startListening();
}

void computePID(){
  #ifdef DEBUGFUNCTIONS
  Serial.println("computePID");
  #endif
  if((remote[1] < ROLL_MIN) || (remote[1] > ROLL_MAX)) remote[1] = remoteLast[1];
  if((remote[2] < PITCH_MIN) || (remote[2] > PITCH_MAX)) remote[2] = remoteLast[2];
  if((remote[3] < YAW_MIN) || (remote[3] > YAW_MAX)) remote[3] = remoteLast[3];
  remoteLast[1] = remote[1];
  remoteLast[2] = remote[2];
  remoteLast[3] = remote[3];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();

}

void getYPR() {
   mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //ypr[0] = ypr[0] * 180/M_PI;
        ypr[0] = 0;
        ypr[1] = (ypr[1] * 180/M_PI);
        ypr[2] = (ypr[2] * 180/M_PI);
      /*if(ypr[1] > 90 || ypr[1] < -90) ypr[1] = yprLast[1];
        yprLast[1] = ypr[1];
        if(ypr[2] > 90 || ypr[2] < -90) ypr[2] = yprLast[2];
        yprLast[2] = ypr[2];*/
  
        #ifdef DEBUGYPR
        Serial.print("Yaw: ");
        Serial.print(ypr[0]);
        Serial.print("  Pitch: ");
        Serial.print(ypr[1]);
        Serial.print("  Roll: ");
        Serial.println(ypr[2]);
        #endif
    }
}

void calculateVelocities(){
  
  #ifdef DEBUGFUNCTIONS
  Serial.println("cakculateVelocities");
  #endif
  
  if((remote[0] < 0) || (remote[0] > 255)) remote[0] = remoteLast[0];
  remoteLast[0] = remote[0];
  velocity = map(remote[0], RC_THROTTLE_MIN, RC_THROTTLE_MAX, ESC_TAKEOFF_OFFSET, ESC_MAX);
  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  velocityLast = velocity; 
  va=velocity+bal_roll+bal_pitch+bal_axes;
  vb=velocity-bal_roll+bal_pitch-bal_axes;
  vc=velocity+bal_roll-bal_pitch-bal_axes;
  vd=velocity-bal_roll-bal_pitch+bal_axes;
  
  #ifdef DEBUGSPEED
  Serial.print("FR: ");
  Serial.print(va);
  Serial.print(" FL: ");
  Serial.print(vb);
  Serial.print(" RR: ");
  Serial.print(vc);
  Serial.print(" RL: ");
  Serial.println(vd);
  
  Serial.print("Bal_pitch: ");
  Serial.print(bal_pitch);
  Serial.print("  Bal_roll: ");
  Serial.println(bal_roll);
  Serial.print("  Bal_axes: ");
  Serial.println(bal_axes);
  #endif
  
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}

void updateMotors(){
  #ifdef DEBUGFUNCTIONS
  Serial.println("updateMotors");
  #endif
  if(remote[0] > 0) {
  fr.write(va);
  fl.write(vc);
  rr.write(vb);
  rl.write(vd);
  }
  else {
  fr.write(ESC_MIN);
  fl.write(ESC_MIN);
  rr.write(ESC_MIN);
  rl.write(ESC_MIN);
  }
}

void arm(){

  fr.write(ESC_MIN);
  fl.write(ESC_MIN);
  rr.write(ESC_MIN);
  rl.write(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

}


void initESCs(){

  fr.attach(ESC_FR);
  fl.attach(ESC_FL);
  rr.attach(ESC_RR);
  rl.attach(ESC_RL);
  delay(100); 
  arm();

}

void initBalancing(){

  bal_axes = 0;
  bal_roll = 0;
  bal_pitch = 0;

}

void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  pitchReg.SetSampleTime(100);
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  rollReg.SetSampleTime(100);
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);
  yawReg.SetSampleTime(100);

}

void rx() {
  #ifdef DEBUGFUNCTIONS
  Serial.println("rx");
  #endif
  if ( radio.available() )
  {
    // Read the data payload until we've received everything
    bool done = false;
    while (!done)
    {
      // Fetch the data payload
      done = radio.read( remote, sizeof(remote) );
    }
    //radio.stopListening();
    //radio.write(&ypr[1],sizeof(ypr[1]));
    //radio.startListening();
   updatePID();
   #ifdef DEBUGPID
   Serial.print("P: ");
   Serial.print(remote[4]);
   Serial.print(" I: ");
   Serial.print(remote[5]);
   Serial.print(" D: ");
   Serial.println(remote[6]);
   #endif
  }
  #ifdef DEBUGRADIO
  else Serial.println("No radio available");
  #endif

}

void updatePID() {
  remote[4] /= 100000;
  remote[5] /= 100000;
  remote[6] /= 100000;
  #ifdef DEBUGFUNCTIONS
  Serial.println("updatePID");
  #endif
  if(remoteLast[4] != remote[4] || remoteLast[5] != remote[5] || remoteLast[6] != remote[6]) {
  pitchReg.SetTunings(remote[4],remote[5],remote[6]);
  rollReg.SetTunings(remote[4],remote[5],remote[6]);
  yawReg.SetTunings(remote[4],remote[5],remote[6]);
  remoteLast[4] = remote[4];
  remoteLast[5] = remote[5];
  remoteLast[6] = remote[6];
  }
}
  
#endif


