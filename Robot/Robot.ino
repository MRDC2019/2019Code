#include "Config.h"

#include "RobotIO.h"
#include "Comm.h"

#include <Servo.h>
#include <SPI.h>

// Robot input and output structs
RobotIn in;
RobotOut out;
Comm comm(&in, &out);

// Drivetrain IO
Servo driveFL;
Servo driveBL;
Servo driveFR;
Servo driveBR;

// Arm IO
Servo waistMotor;
Servo shoulderMotor;
Servo elbowMotor;
Servo wristMotor;
Servo vacuumMotor;

int waistPotPin = WAIST_PIN;
int shoulderPotPin = SHOULDER_PIN;
int elbowPotPin = ELBOW_PIN;

// timing vars
unsigned long start;

void setup() {
  // init Drivetrain IO
  driveFL.attach(DRIVE_FL_PIN);
  driveBL.attach(DRIVE_BL_PIN);
  driveFR.attach(DRIVE_FR_PIN);
  driveBR.attach(DRIVE_BR_PIN);
  
  // init Arm IO
  //shoulderMotor.attach(SHOULDER_MOTOR_PIN);
  //wristMotor.attach(WRIST_MOTOR_PIN);
  //pinMode(keyGrabberPin, OUTPUT);
  //digitalWrite(keyGrabberPin, HIGH);

  start = millis();
  
  comm.begin(BAUD_RATE);
  Serial.begin(9600);
}

void loop() {
  //in.shoulder = analogRead(shoulderPotPin);
  //Serial.println(in.shoulder);
  in.waist = 12;
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
    driveFL.write(out.driveFL);
    driveBL.write(out.driveBL);
    driveFR.write(out.driveFR);
    driveBR.write(out.driveBR);
    //Serial.print(out.driveFL); Serial.print(" "); Serial.print(out.driveBL); Serial.print(" "); Serial.print(out.driveFR); Serial.print(" "); Serial.println(out.driveBR); 
    
    //shoulderMotor.write(out.shoulder);
  }else if(comm.getFailures() > 6){
    driveFL.write(90);
    driveBL.write(90);
    driveFR.write(90);
    driveBR.write(90);
    //shoulderMotor.write(90);
  }
}

