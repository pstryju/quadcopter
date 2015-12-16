#ifndef QUADARDU
#define QUADARDU

#define DEBUGMODE

#define ESC_FL 6
#define ESC_RL 4
#define ESC_FR 5
#define ESC_RR 3

#define ESC_MIN 800
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 830

#define RC_THROTTLE_MAX 255
#define RC_THROTTLE_MIN 0

#define PITCH_P_VAL 0.6
#define PITCH_I_VAL 0.31
#define PITCH_D_VAL 0

#define ROLL_P_VAL 0.6
#define ROLL_I_VAL 0.31
#define ROLL_D_VAL 0

#define YAW_P_VAL 2.11
#define YAW_I_VAL 0.35
#define YAW_D_VAL 0

#define ANGLEX_P_VAL 0.22
#define ANGLEX_I_VAL 0
#define ANGLEX_D_VAL 0

#define ANGLEY_P_VAL 0.22
#define ANGLEY_I_VAL 0
#define ANGLEY_D_VAL 0

#define PITCH_MIN -90
#define PITCH_MAX 90
#define ROLL_MIN -90
#define ROLL_MAX 90
#define YAW_MIN -45
#define YAW_MAX 45

#define PID_PITCH_INFLUENCE 300
#define PID_ROLL_INFLUENCE 300
#define PID_YAW_INFLUENCE 200
#define PID_ANGLEX_INFLUENCE 100
#define PID_ANGLEY_INFLUENCE 100

#define GYROSCOPE_SENSITIVITY 65.536
#define GYRO_MAF_NR  3 
#define SPLIT  0.98 //Complementary filter
#define RadToDeg 180.0/PI 
#define  ACC_HPF_NR  98 

#include <Servo.h>
#include <PIDCont.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <Wire.h>
#include "MPU6050.h"

bool rateAngleSwitch = 1;
byte buf[7];
byte rx_throttle;
int rx_pitch, rx_roll, rx_yaw;

MPU6050 mpu;
double bal_roll = 0, bal_pitch = 0, bal_axes = 0;
float gx_aver=0, gy_aver=0, gz_aver=0;
float gx_temp[GYRO_MAF_NR]={0.0,0.0};
float gy_temp[GYRO_MAF_NR]={0.0,0.0};
float gz_temp[GYRO_MAF_NR]={0.0,0.0};
float angle_pitch, angle_roll;
int accx_temp = 0, accy_temp = 0, accz_temp = 0;

PIDCont PIDroll, PIDpitch, PIDyaw ,PIDangleX, PIDangleY;
int PIDroll_val, PIDpitch_val, PIDyaw_val;
float pGain, iGain, dGain;

Servo fr,fl,rr,rl;
int throttle; 

unsigned long ts=millis();
unsigned long tf=micros();
unsigned long timeAnglePrevious;
unsigned long radioWatchdogTimer;
unsigned int radioLostTimer;

void setup(){
  
 #ifdef DEBUGMODE
  Serial.begin(115200);
  Serial.flush();
  Serial.println("EOS!");
 #endif
 
  initRadio();
  initESCs();
  initMPU();
  initRegulators();
  timeAnglePrevious = millis();
}

void loop(){
    rx();
    getSensorsData();
    computePID();
    calculateVelocities();
}

void initMPU() {
  Wire.begin();
  mpu.initialize();
  mpu.setXGyroOffset(86);
  mpu.setYGyroOffset(24);
  mpu.setZGyroOffset(28);
  mpu.setXAccelOffset(-4895);
  mpu.setYAccelOffset(-1421);
  mpu.setZAccelOffset(900);
}

void initRadio() {
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"clie1");
  Mirf.payload = 7;
  Mirf.config();
}

void computePID(){
  if(rateAngleSwitch) {
    PIDangleX.resetI();
    PIDangleY.resetI();
  }
  else {
    rx_roll = (int)PIDangleX.Compute((float)rx_roll+angle_roll,gy_aver/65.536,(float)rx_roll);
    rx_pitch = (int)PIDangleY.Compute((float)rx_pitch-angle_pitch,gx_aver/65.536,(float)rx_pitch);
    #ifdef DEBUGMODE
    Serial.print( "  rx_roll: ");
    Serial.print(rx_roll);
    Serial.print( "  rx_pitch: ");
    Serial.print(rx_pitch); 
    #endif
  }

  PIDroll_val = (int)PIDroll.Compute((float)rx_roll-gx_aver/65.536);
  PIDpitch_val = (int)PIDpitch.Compute((float)rx_pitch-gy_aver/65.536);
  PIDyaw_val = (int)PIDyaw.Compute((float)rx_yaw-gz_aver/65.536);

}

void getSensorsData() {
   if((micros()-tf) > 700){
    getGyroData();
    tf = micros(); 
  }
  if((millis()-ts) > 20){  //Update only once per 20ms (50Hz update rate)
    getAccData();
    ts = millis();
  }
  unsigned long timeAngle = millis();
  float deltaAngleTime = (float)(timeAngle-timeAnglePrevious) / 1000.0;
  float accx = atan2(accx_temp,accz_temp)*RadToDeg;
  float accy = atan2(accy_temp,accz_temp)*RadToDeg; 
  angle_pitch = SPLIT*((-gy_aver/65.536)*deltaAngleTime+angle_pitch)+(1.0-SPLIT)*accx;
  angle_roll = SPLIT*((gx_aver/65.536)*deltaAngleTime+angle_roll)+(1.0-SPLIT)*accy;
  #ifdef DEBUGMODE
  Serial.print( "  angle_pitch: ");
  Serial.print(angle_pitch);
  Serial.print( "  angle_roll: ");
  Serial.print(angle_roll); 
  #endif
  timeAnglePrevious = timeAngle; 
   
}

void getGyroData() {
  int16_t buffer[3];
  mpu.getRotation(&buffer[0], &buffer[1], &buffer[2]);  //Update only per 1300us, (~800Hz update rate)
  for(byte i = 0; i < (GYRO_MAF_NR-1); i++){
    gx_temp[i]=gx_temp[i+1];
    gy_temp[i]=gy_temp[i+1];
    gz_temp[i]=gz_temp[i+1];
  }
  gx_temp[GYRO_MAF_NR-1] = (float)(buffer[0]);
  gy_temp[GYRO_MAF_NR-1] = (float)(buffer[1]);
  gz_temp[GYRO_MAF_NR-1] = (float)(buffer[2]);
  gyroMAF();
}

void getAccData() {
  int16_t buffer[3];
  mpu.getAcceleration(&buffer[0], &buffer[1], &buffer[2]);
  accx_temp = (ACC_HPF_NR*accx_temp + (100-ACC_HPF_NR)*buffer[0]) / 300;
  accy_temp = (ACC_HPF_NR*accy_temp + (100-ACC_HPF_NR)*buffer[1]) / 300;
  accz_temp = (ACC_HPF_NR*accz_temp + (100-ACC_HPF_NR)*buffer[2]) / 300;
}

void gyroMAF(){//Moving average filter
  gx_aver=0;
  gy_aver=0;
  gz_aver=0;
  for(byte i=0;i<GYRO_MAF_NR;i++){
    gx_aver=gx_aver+gx_temp[i];
    gy_aver=gy_aver+gy_temp[i];
    gz_aver=gz_aver+gz_temp[i];
  }  
  gx_aver=(float)gx_aver/GYRO_MAF_NR;
  gy_aver=(float)gy_aver/GYRO_MAF_NR;
  gz_aver=(float)gz_aver/GYRO_MAF_NR;

}

void calculateVelocities(){
  throttle = map(rx_throttle, RC_THROTTLE_MIN, RC_THROTTLE_MAX, ESC_MIN, ESC_MAX);
  if(rx_throttle==0) {
    PIDroll_val = 0;
    PIDpitch_val = 0;
    PIDyaw_val = 0;
    PIDroll.resetI();
    PIDpitch.resetI();
    PIDyaw.resetI();
    PIDangleX.resetI();
    PIDangleY.resetI();
  }
  int va=throttle-PIDroll_val-PIDpitch_val-PIDyaw_val;
  int vb=throttle+PIDroll_val-PIDpitch_val+PIDyaw_val;
  int vc=throttle-PIDroll_val+PIDpitch_val+PIDyaw_val;
  int vd=throttle+PIDroll_val+PIDpitch_val-PIDyaw_val;

  fr.writeMicroseconds(va);
  fl.writeMicroseconds(vb);
  rr.writeMicroseconds(vc);
  rl.writeMicroseconds(vd);
  
  #ifdef DEBUGMODE
  Serial.print("FR: ");
  Serial.print(va);
  Serial.print(" FL: ");
  Serial.print(vb);
  Serial.print(" RR: ");
  Serial.print(vc);
  Serial.print(" RL: ");
  Serial.print(vd);
  
  Serial.print(" Bal_pitch: ");
  Serial.print(PIDpitch_val);
  Serial.print("  Bal_roll: ");
  Serial.print(PIDroll_val);
  Serial.print("  Thr: ");
  Serial.print(throttle);
  Serial.print(" GX ");
  Serial.print(gx_aver/GYROSCOPE_SENSITIVITY);
  Serial.print(" GY: ");
  Serial.println(gy_aver/GYROSCOPE_SENSITIVITY);
  #endif
  
  if(throttle < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}

void arm(){
  delay(3000);
  fr.writeMicroseconds(ESC_TAKEOFF_OFFSET);
  fl.writeMicroseconds(ESC_TAKEOFF_OFFSET);
  rr.writeMicroseconds(ESC_TAKEOFF_OFFSET);
  rl.writeMicroseconds(ESC_TAKEOFF_OFFSET); 
  delay(1000);
  fr.writeMicroseconds(ESC_MIN);
  fl.writeMicroseconds(ESC_MIN);
  rr.writeMicroseconds(ESC_MIN);
  rl.writeMicroseconds(ESC_MIN);
  delay(2000);
}


void initESCs(){
  fr.attach(ESC_FR);
  fl.attach(ESC_FL);
  rr.attach(ESC_RR);
  rl.attach(ESC_RL);
  delay(100); 
  arm();

}

void initRegulators(){
  PIDroll.ChangeParameters(ROLL_P_VAL,ROLL_I_VAL,ROLL_D_VAL,-PID_ROLL_INFLUENCE,PID_ROLL_INFLUENCE);
  PIDpitch.ChangeParameters(PITCH_P_VAL,PITCH_I_VAL,PITCH_D_VAL,-PID_PITCH_INFLUENCE,PID_PITCH_INFLUENCE);
  PIDyaw.ChangeParameters(YAW_P_VAL,YAW_I_VAL,YAW_D_VAL,-PID_YAW_INFLUENCE,PID_YAW_INFLUENCE);
  PIDangleX.ChangeParameters(ANGLEX_P_VAL,ANGLEX_I_VAL,ANGLEX_D_VAL,-PID_ANGLEX_INFLUENCE,PID_ANGLEX_INFLUENCE);
  PIDangleY.ChangeParameters(ANGLEY_P_VAL,ANGLEY_I_VAL,ANGLEY_D_VAL,-PID_ANGLEY_INFLUENCE,PID_ANGLEY_INFLUENCE);
}

void updateRegulators(){
  //PIDroll.ChangeParameters(pGain,iGain,dGain,-PID_ROLL_INFLUENCE,PID_ROLL_INFLUENCE);
  //PIDpitch.ChangeParameters(pGain,iGain,dGain,-PID_PITCH_INFLUENCE,PID_PITCH_INFLUENCE);
  //PIDyaw.ChangeParameters(pGain,iGain,dGain,-PID_YAW_INFLUENCE,PID_YAW_INFLUENCE);
  PIDangleX.ChangeParameters(pGain,iGain,dGain,-PID_ANGLEX_INFLUENCE,PID_ANGLEX_INFLUENCE);
  PIDangleY.ChangeParameters(pGain,iGain,dGain,-PID_ANGLEY_INFLUENCE,PID_ANGLEY_INFLUENCE);
}

void rx() {
  Mirf.setTADDR((byte *)"serv1");
  if(Mirf.dataReady()) {
    Mirf.getData((byte *)&buf);
    radioWatchdogTimer = millis();
  }
  if(millis() - radioWatchdogTimer > 2000) {
    radioLost();
  }
  else {
    rx_throttle = buf[0];
    rx_roll = buf[1] - 30;
    rx_pitch = -1*(buf[2] - 30);
    rx_yaw = buf[3] - 30;
    pGain = buf[4]/100.0;
    iGain = buf[5]/100.0;
    dGain = buf[6]/10000.0;
    updateRegulators();
    if(false){
      rateAngleSwitch = false;
    }
    else{
      rateAngleSwitch = true;
    }
  }
}

void radioLost() {
  #ifdef DEBUGMODE
    Serial.print(" noRadio ");
      #endif
    if((millis() - radioLostTimer > 50) && (rx_throttle > 0)) {
      rx_throttle--;
      radioLostTimer = millis();
    }
}

#endif


