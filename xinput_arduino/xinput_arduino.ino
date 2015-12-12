#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#define CE_PIN   9
#define CSN_PIN 10

typedef struct package{
  short LX;
  short LY;
  short RX;
  short RY;
  short leftTrigger;
  short rightTrigger;
  long p;
  long i;
  short d;
  short crc;
  
};

struct package mb, buff;

byte remote[7];
float ypr[3];
short crc;

boolean tryToRead(struct package * mid, struct package * buff){
    char id;
    char *buf;
    buf=(char *) buff;
    char *m;
    m=(char *) mid;
    if(Serial.available()){
      if(Serial.read()==0xAB){
          for(id = 0; id < sizeof(struct package);){
            if(Serial.available()){
              buf[id]=Serial.read();
              id++;
            }
          }
          crc = buff->LX + buff->LY + buff->RY + buff->RX + buff->leftTrigger;
          if(buff->crc == crc) {
             for(id = 0; id < sizeof(struct package); id++) {
                m[id] = buf[id];
             }
          }
          return 1;   
      }
      Serial.flush();
      return 0;
    }
    return 0;
}


void setup(){
  Serial.begin(115200);  
  radioSetup();
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW);
}

void loop(){
  tryToRead(&mb, &buff);
  remote[0] = mb.leftTrigger;
  remote[1] = mb.LX;
  remote[2] = mb.LY;
  remote[3] = mb.RX;
  remote[4] = mb.p;
  remote[5] = mb.i;
  remote[6] = mb.d;
   Mirf.setTADDR((byte *)"clie1");
   Mirf.send((byte *) &remote);
   while(Mirf.isSending()){
    //Serial.println("Sending");
  }
  //Serial.println("Not sending");
    
}

void radioSetup() {
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = 7;
  Mirf.config();
}
