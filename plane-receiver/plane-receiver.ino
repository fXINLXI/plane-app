#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

RF24 radio(20, 17);

const byte adres[5] = {'1','0','1','0','1'};

uint32_t received=0;

uint8_t received_roll;
uint8_t received_pitch;
uint8_t received_yaw;
uint8_t received_motor;

Servo servos[7];
uint8_t servo_pins[7]={0,2,4,6,8,12,14};

int last_connection;

#define SDA_PIN 26
#define SCL_PIN 27

float c_gx;

float angle_x=0;

long last_read_gyro;

float gyro_x_speed=0;

bool sas=1;
bool flaps=1;
bool release=0;

void setup() {
  delay(2000);
  Serial.begin(115200);
  pinMode(23, OUTPUT);

  for(int i = 0; i<7; i++){
    servos[i].attach(servo_pins[i]);
  }
  
  init_radio();
  setup_radio();

  radio.startListening();

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(100000);

  mpu_begin();

  mpu_calibrate();
}

void loop() {
  mpu_update_angle_x();

  if(millis()-last_connection>1000){
    received_motor=0;
    sas=1;
  }

  if(radio.available()){
    radio.read(&received,sizeof(received));

    received_motor = (received   & 0xfe000000)>>24;
    received_yaw   = (received   & 0x00fe0000)>>16;
    received_pitch = (received   & 0x0000fe00)>>8;
    received_roll  = (received   & 0x000000fe);

    release        = (received   & 0x00010000)>>16;
    flaps          = (received   & 0x00000100)>>8;
    sas            = (received   & 0x00000001);

    print_received();
  }

  if(sas){

  }
  else{
    servos[0].write(MIN(MAX(received_roll+flaps*90,0),180));
    servos[1].write(MIN(MAX(received_roll-flaps*90,0),180));
    servos[2].write(received_pitch);
    servos[3].write(received_pitch);
    servos[4].write(received_yaw);
    servos[5].write(received_motor);
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

void mpu_begin(){
  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B);
  Wire1.write(0x00);
  Wire1.endTransmission(true);
}

void mpu_calibrate(){
  digitalWrite(23,HIGH);
  delay(300);
  digitalWrite(23,LOW);

  int no_measures=3000;

  long sum=0;

  for (int i = 0; i<no_measures;i++){
    Wire1.beginTransmission(0x68);
    Wire1.write(0x43);
    Wire1.endTransmission(false);

    Wire1.requestFrom(0x68, 2);
    while(Wire1.available()<2){delay(1);}
    int16_t gyro_x_raw = Wire1.read()<<8|Wire1.read();
    sum+=gyro_x_raw;
    while(Wire1.available()){Wire1.read();}
    last_read_gyro=millis();
  }
  c_gx=(float)sum/(float)no_measures;

  digitalWrite(23,HIGH);
  delay(300);
  digitalWrite(23,LOW);
}

void mpu_update_angle_x(){
  Wire1.beginTransmission(0x68);
  Wire1.write(0x43);
  Wire1.endTransmission(false);

  Wire1.requestFrom(0x68, 2);
  while(Wire1.available()<2){delay(1);}
  int16_t gyro_x_raw = Wire1.read()<<8|Wire1.read();
  gyro_x_speed = ((float)gyro_x_raw-c_gx)/131.0;
  long now = millis();

  angle_x+=gyro_x_speed*(now-last_read_gyro)/1000.0;

  last_read_gyro=now;
}

void print_received(){
  Serial.print(received,HEX);
  Serial.print(" ");
  servos[0].write(received_motor);
  Serial.print(received_motor);
  Serial.print(" ");
  servos[1].write(received_yaw);
  Serial.print(received_yaw);
  Serial.print(" ");
  servos[2].write(received_pitch);
  Serial.print(received_pitch);
  Serial.print(" ");
  servos[3].write(received_roll);
  Serial.print(received_roll);
  Serial.print(" ");
  Serial.print(release);
  Serial.print(" ");
  Serial.print(flaps);
  Serial.print(" ");
  Serial.print(sas);
  Serial.print(" ");
  Serial.println(angle_x);
}