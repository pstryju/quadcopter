#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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
const uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(CE_PIN, CSN_PIN);

float remote[7];
float ypr[3];
bool ok;
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
  remote[1] = -1*((mb.LX) - 30);
  remote[2] = -1*((mb.LY) - 30);
  remote[3] = (mb.RX) - 30;
  remote[4] = mb.p;
  remote[5] = mb.i;
  remote[6] = mb.d;
  radio.stopListening();
  ok = radio.write(remote, sizeof(remote));
  if(ok) digitalWrite(6,HIGH);
  else digitalWrite(6,LOW);
    
}

void radioSetup() {
  radio.begin();
 // radio.setRetries(15,15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  radio.enableDynamicPayloads();
  //radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipe);
  radio.stopListening();
}
