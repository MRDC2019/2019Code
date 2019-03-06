#include "Config.h"

#include "RobotIO.h"
#include "Comm.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Robot input and output structs
RobotIn in;
RobotOut out;
Comm comm(&in, &out);

// IO
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// timing vars
unsigned long start;

void setup() {
  // init IO
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency, I don't know what this should be
  
  //shoulderMotor.attach(SHOULDER_MOTOR_PIN);
  //wristMotor.attach(WRIST_MOTOR_PIN);
  //pinMode(keyGrabberPin, OUTPUT);
  //digitalWrite(keyGrabberPin, HIGH);

  start = millis();
  
  comm.begin(BAUD_RATE);
  //Serial.begin(9600);
}

void loop() {
  //in.shoulder = analogRead(shoulderPotPin);
  //Serial.println(in.shoulder);
  in.waist = uint16_t(out.driveFL)*16;
  in.shoulder = 34;
  in.elbow = 56;

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
    pwm.setPin(DRIVE_FL_PIN, uint16_t(out.driveFL)*16);
    pwm.setPin(DRIVE_BL_PIN, uint16_t(out.driveBL)*16);
    pwm.setPin(DRIVE_FR_PIN, uint16_t(out.driveFR)*16);
    pwm.setPin(DRIVE_BR_PIN, uint16_t(out.driveBR)*16);
    //Serial.print(uint16_t(out.driveFL)*16); Serial.print(" "); Serial.print(uint16_t(out.driveBL)*16); Serial.print(" "); Serial.print(uint16_t(out.driveFR)*16); Serial.print(" "); Serial.println(uint16_t(out.driveBR)*16); 
    
    //shoulderMotor.write(out.shoulder);
  }else if(comm.getFailures() > 6){
    pwm.setPin(DRIVE_FL_PIN, 2048);
    pwm.setPin(DRIVE_BL_PIN, 2048);
    pwm.setPin(DRIVE_FR_PIN, 2048);
    pwm.setPin(DRIVE_BR_PIN, 2048);
  }
}

