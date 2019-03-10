#include "Config.h"

#include "RobotIO.h"
#include "Comm.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <Servo.h>

// Robot input and output structs
RobotIn in;
RobotOut out;
Comm comm(&in, &out);

// IO
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo elbowMotor;
Servo shoulderMotor;
Servo waistMotor;

// timing vars
unsigned long start;

void setup() {
  comm.begin(BAUD_RATE);
  Serial.begin(9600);
  
  // init IO
  pwm.begin();
  pwm.setPWMFreq(250);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1); // properties chip
  SPI.setBitOrder(MSBFIRST);  //properties chip

  pinMode(WAIST_PIN, OUTPUT);
  pinMode(SHOULDER_PIN, OUTPUT);
  pinMode(ELBOW_PIN, OUTPUT);
  Encoder_setup(WAIST_PIN, 0);
  Encoder_setup(SHOULDER_PIN, 14620);
  Encoder_setup(ELBOW_PIN, 1640);
  
  pinMode(VACUUM_MOTOR_PIN, OUTPUT);
  digitalWrite(VACUUM_MOTOR_PIN, LOW);

  waistMotor.attach(WAIST_MOTOR_PIN);
  shoulderMotor.attach(SHOULDER_MOTOR_PIN);
  elbowMotor.attach(ELBOW_MOTOR_PIN);

  start = millis();
}

void loop() {
  in.waist = (read_encoder(WAIST_PIN) + 1340) & 0x3FFF; // zero manually
  in.shoulder = read_encoder(SHOULDER_PIN);
  in.elbow = read_encoder(ELBOW_PIN);
  //Serial.print(in.waist); Serial.print(" "); Serial.print(in.shoulder); Serial.print(" "); Serial.println(in.elbow);
  
  // Write inputs to PC
  comm.write();
  int delayTime = 16 - (millis() - start);
  if(delayTime<0)
    delayTime = 0;
  delay(delayTime);
  //Serial.print("cycle time = "); Serial.println(millis()-start);
  start = millis();

  // Read output values to IO struct
  if(comm.read()){
    // Write RobotOut values to outputs
    pwm.setPin(DRIVE_FL_PIN, uint16_t(out.driveFL)*4+1024);
    pwm.setPin(DRIVE_BL_PIN, uint16_t(out.driveBL)*4+1024);
    pwm.setPin(DRIVE_FR_PIN, uint16_t(out.driveFR)*4+1024);
    pwm.setPin(DRIVE_BR_PIN, uint16_t(out.driveBR)*4+1024);
    //Serial.print(uint16_t(out.driveFL)*4+1024); Serial.print(" "); Serial.print(uint16_t(out.driveBL)*4+1024); Serial.print(" "); Serial.print(uint16_t(out.driveFR)*4+1024); Serial.print(" "); Serial.println(uint16_t(out.driveBR)*4+1024); 
    
    pwm.setPin(VACUUM2_MOTOR_PIN, uint16_t(out.wrist)*4+1024);
    //Serial.println(uint16_t(out.wrist)*4+1024);
    //pwm.setPin(WAIST_MOTOR_PIN, uint16_t(out.waist)*2+1030);
    //pwm.setPin(SHOULDER_MOTOR_PIN, uint16_t(out.shoulder)*2+1024);
    //pwm.setPin(ELBOW_MOTOR_PIN, uint16_t(out.elbow)*2+1024);

    double elbowout = out.elbow/255.0*180.0;
    waistMotor.write(out.waist/255.0*180.0);
    shoulderMotor.write(out.shoulder/255.0*180.0);
    elbowMotor.write(elbowout);
    Serial.println(elbowout);
    //Serial.print(uint16_t(out.waist)*2+1030); Serial.print(" "); Serial.print(uint16_t(out.shoulder)*2+1024); Serial.print(" "); Serial.println(uint16_t(out.elbow)*2+1024);

    if(out.vacuum == 127)
      digitalWrite(VACUUM_MOTOR_PIN, LOW);
    else
      digitalWrite(VACUUM_MOTOR_PIN, HIGH);
  }else if(comm.getFailures() > 6){
    pwm.setPin(DRIVE_FL_PIN, 1536);
    pwm.setPin(DRIVE_BL_PIN, 1536);
    pwm.setPin(DRIVE_FR_PIN, 1536);
    pwm.setPin(DRIVE_BR_PIN, 1536);
    
    pwm.setPin(VACUUM2_MOTOR_PIN, 1536);
    //pwm.setPin(WAIST_MOTOR_PIN, 1280);
    //pwm.setPin(SHOULDER_MOTOR_PIN, 1280);
    //pwm.setPin(ELBOW_MOTOR_PIN, 1280);
    
    waistMotor.write(90);
    shoulderMotor.write(90);
    elbowMotor.write(90);
    
    digitalWrite(VACUUM_MOTOR_PIN, LOW);
  }
}

void Encoder_setup(int encoderPin, unsigned int zero){
  AS5047D_Write(encoderPin , 0x0018, 0x0004); // setting 1
  AS5047D_Write(encoderPin , 0x0019, 0x0000); // setting 2
  AS5047D_Write(encoderPin , 0x0016, zero>>6); // most significant 8 bits of the zero position
  AS5047D_Write(encoderPin , 0x0017, zero&0x3F); // least significant 6 bits of the zero position
}

unsigned int read_encoder(int encoderPin){
  return AS5047D_Read(encoderPin, 0x3FFE) & 0x3FFF;
}

// ************************Write to AS5047D **************************
void AS5047D_Write( int SSPin, int address, int value)
{
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address via SPI:
  
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80);  // its  a write command and don't change the parity bit (0x80)
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //  send value via SPI:
  
  v_l = value & 0x00FF;
  v_h = (unsigned int)(value & 0x3F00) >> 8;
  
  if (parity(value & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80); // its a write command and don't change the parity bit (0x80)
  

  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SSPin, HIGH);
}

//*******************Read from AS5047D ********************************
unsigned int AS5047D_Read( int SSPin, unsigned int address)
{
  unsigned int result = 0;   // result to return
  
  byte res_h = 0;
  byte res_l = 0;
  
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address and value via SPI:
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address | (0x40 << 8)) == 1) v_h = v_h | 0x80; // set parity bit
  
  v_h = v_h | 0x40; // its  a read command
  
  // Serial.print( " parity:  ");Serial.println(parity(address | (RD <<8)));
  // Serial.print(v_h, HEX); Serial.print(" A ");  Serial.print(v_l, HEX);  Serial.print(" >> ");
  
  res_h = SPI.transfer(v_h);
  res_l = SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //if (parity(0x00 | (RD <<8))==1) res_h = res_h | 0x80;  // set parity bit
  //res_h = res_h | RD;
  
  res_h = (SPI.transfer(0x00));
  res_l = SPI.transfer(0x00);
  
  res_h = res_h & 0x3F;  // filter bits outside data
  
  //Serial.print(res_h, HEX);   Serial.print(" R  ");  Serial.print(res_l, HEX);   Serial.print("  ");
  
  digitalWrite(SSPin, HIGH);
  
  return (result = (res_h << 8) | res_l);
}

//*******************check parity ******************************************
int parity(unsigned int x) {
  int parity = 0;
  while (x > 0) {
    parity = (parity + (x & 1)) % 2;
    x >>= 1;
  }
  return (parity);
}
