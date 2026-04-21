#include <SPI.h>
#include <RF24.h>

RF24 radio(20, 17);

const byte adres[5] = {'1','0','1','0','1'};
uint32_t to_send=0;

int packet_loss=0;
int packets=0;

void setup() {
  delay(2000);
  Serial.begin(9600);
  pinMode(23, OUTPUT);
  
  init_radio();
  setup_radio();

  radio.stopListening();
}

void loop() {
  if(packets>=30){
    Serial.println((float)packet_loss/30.0);
    packet_loss=0;z
    packets=0;
  }

  if(Serial.available()>=4){

    byte buf[4];
    Serial.readBytes(buf, 4);
    to_send = ((uint32_t)buf[0] << 24) |
              ((uint32_t)buf[1] << 16) |
              ((uint32_t)buf[2] << 8) |
              ((uint32_t)buf[3]);
    if(radio.write(&to_send,sizeof(to_send))){
    }
    else{
      packet_loss++;
    }
    packets++;
  }
}

void init_radio(){
  if(!radio.begin()){
    Serial.println("ERROR: Radio hardware not responding");
    while(1){
      digitalWrite(23,HIGH);
      delay(500);
      digitalWrite(23,LOW);
      delay(500);
    }
  }
  Serial.println("Radio initialized correctly");
}

void setup_radio(){
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.openWritingPipe(adres);
  radio.openReadingPipe(1, adres);
  radio.setRetries(4, 3);
  
}