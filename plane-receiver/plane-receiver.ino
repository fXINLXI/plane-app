#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

RF24 radio(20, 17);

const byte adres[5] = {'1','0','1','0','1'};

uint32_t received=0;

uint8_t received_roll=90;
uint8_t received_pitch=90;
uint8_t received_yaw=90;
uint8_t received_motor=90;

Servo servos[7];
uint8_t servo_pins[7]={0,2,4,6,14,12,10};

int last_connection;

#define SDA_PIN 26
#define SCL_PIN 27

bool sas=1;
bool flaps=1;
bool release=1;

float c_gx;
long last_read_gyro;

const float k_p=0.8;
const float k_i=1.7;
float gyro_x_int=0;
float gyro_x_prop=0;

float pid_roll=90;
float dt;

float left_roll=90;
float right_roll=90;

void setup() {
  delay(2000);
  Serial.begin(115200);
  pinMode(23, OUTPUT);
  
  setup_servo();

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
  mpu_update_pid();

  if(millis()-last_connection>1000){
    received_motor=0;
    sas=0;
    release=1;
    flaps=1;
    digitalWrite(23,HIGH);
  }
  else{
    digitalWrite(23,LOW);
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

    last_connection=millis();

    print_received();
  }

  servos[0].write(MIN(MAX(left_roll,0),180));
  servos[1].write(MIN(MAX(right_roll,0),180));
  servos[2].write(received_pitch);
  servos[3].write(received_yaw);
  servos[4].write(received_motor);
  servos[5].write(180-release*180);
  
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

void setup_servo(){
  servos[0].attach(servo_pins[0],800,2200);
  servos[1].attach(servo_pins[1],800,2200);
  servos[2].attach(servo_pins[2],1000,1900);
  servos[3].attach(servo_pins[3],1000,2000);
  servos[4].attach(servo_pins[4],1000,2000);
  servos[5].attach(servo_pins[5]);
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
  delay(100);

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

void mpu_update_pid(){
  Wire1.beginTransmission(0x68);
  Wire1.write(0x43);
  Wire1.endTransmission(false);

  Wire1.requestFrom(0x68, 2);
  while(Wire1.available()<2){delay(1);}
  int16_t gyro_x_raw = Wire1.read()<<8|Wire1.read();
  gyro_x_prop = (((float)gyro_x_raw-c_gx)/131.0-received_roll+90);//  deg/sec
  long now = millis();

  dt=(now-last_read_gyro)/1000.0;
  pid_roll = (gyro_x_prop*k_p+gyro_x_int*k_i);

  left_roll=90+pid_roll*(float)((float)flaps*0.5+1)-flaps*90;
  right_roll=90+pid_roll*(float)((float)flaps*0.5+1)+flaps*90;

  if((left_roll>0 && left_roll<180) || (right_roll>0 && left_roll<180)){
    gyro_x_int+=gyro_x_prop*dt;//                                     deg
  }

  last_read_gyro=now;
}

void print_received(){
  Serial.print(received,HEX);
  Serial.print(" ");
  Serial.print(received_motor);
  Serial.print(" ");
  Serial.print(received_yaw);
  Serial.print(" ");
  Serial.print(received_pitch);
  Serial.print(" ");
  Serial.print(received_roll);
  Serial.print(" ");
  Serial.print(release);
  Serial.print(" ");
  Serial.print(flaps);
  Serial.print(" ");
  Serial.println(sas);
}