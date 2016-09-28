#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

const uint8_t sharpFL = A15;  // sharpFrontLeft
const uint8_t sharpFR = A14;  // sharpFrontRight
const uint8_t sharpRT = A12;  // sharpRightTop
const uint8_t sharpRB = A13;  // sharpRightBottom
const uint8_t sharpBL = A9;   // sharpBackLeft
const uint8_t sharpBR = A8;   // sharpBackRight
const uint8_t sharpLT = A11;  // sharpLeftTop
const uint8_t sharpLB = A10;  // sharpLeftBottom

SharpIR sharp1(sharpFL, 25, 93, 2016);
SharpIR sharp2(sharpFR, 25, 93, 2016);
SharpIR sharp3(sharpRT, 25, 93, 2016);
SharpIR sharp4(sharpRB, 25, 93, 2016);
SharpIR sharp5(sharpBL, 25, 93, 2016);
SharpIR sharp6(sharpBR, 25, 93, 2016);
SharpIR sharp7(sharpLT, 25, 93, 2016);
SharpIR sharp8(sharpLB, 25, 93, 2016);

void setup(){
  
  Serial.begin(9600);
  pinMode (A7, INPUT);
  pinMode (A8, INPUT);
  pinMode (A9, INPUT);
  pinMode (A10, INPUT);
  pinMode (A11, INPUT);
  pinMode (A12, INPUT);
  pinMode (A13, INPUT);
  pinMode (A14, INPUT); 
  
  AFMS.begin();
  M1->setSpeed(150);
  M2->setSpeed(150);
  M3->setSpeed(150);
  M4->setSpeed(150);
  M1->run(FORWARD);
  M2->run(FORWARD);
  M3->run(FORWARD);
  M4->run(FORWARD);
  M1->run(RELEASE);
  M2->run(RELEASE);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

void loop(){
  Serial.print("sharpFL: ");
  Serial.print(sharp1.distance());
  Serial.print("\tsharpFR: ");
  Serial.print(sharp2.distance());
  Serial.print("\tsharpRT: ");
  Serial.print(sharp3.distance());
  Serial.print("\tsharpRB: ");
  Serial.print(sharp4.distance());
  Serial.print("\tsharpBL: ");
  Serial.print(sharp5.distance());
  Serial.print("\tsharpBR: ");
  Serial.print(sharp6.distance());
  Serial.print("\tsharpLT: ");
  Serial.print(sharp7.distance());
  Serial.print("\tsharpLB: ");
  Serial.println(sharp8.distance());
  if(millis() > 8000)
  {
    M1->run(FORWARD);
    M2->run(FORWARD);
    M3->run(FORWARD);
    M4->run(FORWARD);
  }
}
  

