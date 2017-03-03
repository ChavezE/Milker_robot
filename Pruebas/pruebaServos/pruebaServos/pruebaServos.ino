// RoBorregos 2016
//
// Arduino Mega 2560
//
// I2C addresses
// IMU: 0x28
// Motor driver: 0x60
// LCD: 0x27
//
// Motor definitions
// MLF: Left Front
// MRF: Right Front
// MRB: Right Back
// MLB: Left Back

//--------------------- Libraries --------------------------//
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SharpIR.h>
#include <NewPing.h>
#include <Servo.h>

//--------------------- Pins -------------------------------//
const uint8_t sharpFL = A15;  // sharpFrontLeft
const uint8_t sharpFR = A14;  // sharpFrontRight
const uint8_t sharpRT = A12;  // sharpRightTop
const uint8_t sharpRB = A13;  // sharpRightBottom
const uint8_t sharpBL = A9;   // sharpBackLeft
const uint8_t sharpBR = A8;   // sharpBackRight
const uint8_t sharpLT = A11;  // sharpLeftTop
const uint8_t sharpLB = A10;  // sharpLeftBottom
const uint8_t sharpG = A7;    // sharpGripper

//const uint8_t ultrasonicGT = 7; // ultrasonicGripperTrigger
//const uint8_t ultrasonicGE = 8; // ultrasonicGripperEcho

const uint8_t encoderF = 18;  // encoderFront
//const uint8_t encoderB = 19;  // encoderBack

const uint8_t servoGA = 35; // servoGripperActuator
const uint8_t servoGT = 41; // servoGripperTurn
const uint8_t servoGM = 37; // servoGripperMovement
const uint8_t servoHM = 51; // servoHorizontalMovement
const uint8_t servoL = 33;  // servoLevas
const uint8_t servoAM = 39; // servoASOMovement

const uint8_t limitSwitchHMI = 16; // limitSwitchHorizontalMovementIn
const uint8_t limitSwitchHMO = 17; // limitSwitchHorizontalMovementOut
const uint8_t limitSwitchGMI = 43;  // limitSwitchGripperMovementIn
const uint8_t limitSwitchGMO = 45;  // limitSwitchGripperMovementOut
const uint8_t limitSwitchAMI = 47;  // limitSwitchASOMovementIn
const uint8_t limitSwitchAMO = 49;  // limitSwitchASOMovementOut

//--------------------- Motors -----------------------------//
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *MLB = AFMS.getMotor(1);
Adafruit_DCMotor *MRB = AFMS.getMotor(2);
Adafruit_DCMotor *MRF = AFMS.getMotor(3);
Adafruit_DCMotor *MLF = AFMS.getMotor(4);

//--------------------- Servos -----------------------------//
Servo gripperA;
Servo gripperT;
Servo gripperM;
Servo horizontalM;
Servo levas;
Servo asoM;

//--------------------- LCD --------------------------------//
//LiquidCrystal_I2C lcd(0x27, 16, 2);

//--------------------- IMU --------------------------------//
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//--------------------- Ultrasonic -------------------------//
//NewPing distGripper(ultrasonicGT, ultrasonicGE, 200);

//--------------------- Sharps -----------------------------//
SharpIR distFL(sharpFL, 25, 93, 2016);
SharpIR distFR(sharpFR, 25, 93, 2016);
SharpIR distRT(sharpRT, 25, 93, 2016);
SharpIR distRB(sharpRB, 25, 93, 2016);
SharpIR distBL(sharpBL, 25, 93, 2016);
SharpIR distBR(sharpBR, 25, 93, 2016);
SharpIR distLT(sharpLT, 25, 93, 2016);
SharpIR distLB(sharpLB, 25, 93, 2016);
SharpIR distG(sharpG, 25, 93, 2016);

//--------------------- Constants -------------------------//
const float precisionIMU = 2.0;
const unsigned int wallDistance = 8;
const int stepsPerCm = 42;
const float kp = 2.5;
const unsigned int terrineDistance = 5;

//--------------------- Global variables ------------------//
volatile unsigned int stepsF = 0; // stepsFront
volatile unsigned int stepsB = 0; // stepsBack

byte rightMotorSpeed = 40; //145
byte leftMotorSpeed = 40;  //145

unsigned long clk;  //used to measure time

//--------------------- Control movements -----------------//
void moveForwardStraight(float ePos, float iLim, float oLim)
{
  if(millis() - clk >= 200)
  {
    float oPos = getOrientation();
    int angleNeeded = ePos - oPos;
    byte operation;
    if(angleNeeded > 180)
    {
      operation = (360-ePos+oPos) * kp;
      rightMotorSpeed = rightMotorSpeed + operation >= 160 ? 160 : rightMotorSpeed + operation;
      MRB->setSpeed(rightMotorSpeed);
      MRF->setSpeed(rightMotorSpeed);
    }
    else if(angleNeeded > 0)
    {
      operation = (angleNeeded) * kp;
      rightMotorSpeed = rightMotorSpeed - operation <= 40 ? 40 : rightMotorSpeed - operation;
      MRB->setSpeed(rightMotorSpeed);
      MRF->setSpeed(rightMotorSpeed);
    }
    else if(angleNeeded > -180)
    {
      operation = (-angleNeeded) * kp;
      rightMotorSpeed = rightMotorSpeed + operation >= 160 ? 160 : rightMotorSpeed + operation;
      MRB->setSpeed(rightMotorSpeed);
      MRF->setSpeed(rightMotorSpeed);
    }
    else
    {
      operation = (360-oPos+ePos) * kp;
      rightMotorSpeed = rightMotorSpeed - operation <= 40 ? 40 : rightMotorSpeed - operation;
      MRB->setSpeed(rightMotorSpeed);
      MRF->setSpeed(rightMotorSpeed);
    }

    clk = millis();
  }

  MLB->run(FORWARD);
  MRB->run(FORWARD);
  MRF->run(FORWARD);
  MLF->run(FORWARD);
}

String moveForward(int distance)
{
  stepsF = 0;
  float ePos = getOrientation();
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  clk = millis();

  float distanceFR = distFR.distance();
  float distanceFL = distFL.distance();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("FR:");
  // lcd.print(distanceFR);
  // lcd.setCursor(0,1);
  // lcd.print("FL:");
  // lcd.print(distanceFL);
  while(stepsF < distance*stepsPerCm && distanceFR > wallDistance && distanceFL > wallDistance)
  {
    moveForwardStraight(ePos, iLim, oLim);

    distanceFR = distFR.distance();
    distanceFL = distFL.distance();
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("FR:");
    // lcd.print(distanceFR);
    // lcd.setCursor(0,1);
    // lcd.print("FL:");
    // lcd.print(distanceFL);
  }
  stopMotors();
  if(stepsF >= distance*stepsPerCm)
    return "0";
  if(distFR.distance() <= wallDistance + 3 && distFL.distance() <= wallDistance + 3)
    return "1";
  return "-1";
}

String moveBackward(int distance)
{
  stepsF = 0;
  float distanceBR = distBR.distance();
  float distanceBL = distBL.distance();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("BR:");
  // lcd.print(distanceBR);
  // lcd.setCursor(0,1);
  // lcd.print("BL:");
  // lcd.print(distanceBL);
  while(stepsF < distance*stepsPerCm && distanceBR > wallDistance && distanceBL > wallDistance)
  {
    MLB->run(BACKWARD);
    MRB->run(BACKWARD);
    MRF->run(BACKWARD);
    MLF->run(BACKWARD);
    distanceBR = distBR.distance();
    distanceBL = distBL.distance();
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("BR:");
    // lcd.print(distanceBR);
    // lcd.setCursor(0,1);
    // lcd.print("BL:");
    // lcd.print(distanceBL);
  }
  stopMotors();
  if(stepsF >= distance*stepsPerCm)
    return "0";
  if(distBR.distance() <= wallDistance + 3 && distBL.distance() <= wallDistance + 3)
    return "1";
  return "-1";
}

String moveBackward(int distance, int backDist)
{
  stepsF = 0;
  float distanceBR = distBR.distance();
  float distanceBL = distBL.distance();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("BR:");
  // lcd.print(distanceBR);
  // lcd.setCursor(0,1);
  // lcd.print("BL:");
  // lcd.print(distanceBL);
  while(stepsF < distance*stepsPerCm && distanceBR > wallDistance - backDist && distanceBL > wallDistance - backDist)
  {
    MLB->run(BACKWARD);
    MRB->run(BACKWARD);
    MRF->run(BACKWARD);
    MLF->run(BACKWARD);
    distanceBR = distBR.distance();
    distanceBL = distBL.distance();
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("BR:");
    // lcd.print(distanceBR);
    // lcd.setCursor(0,1);
    // lcd.print("BL:");
    // lcd.print(distanceBL);
  }
  stopMotors();
  if(stepsF >= distance*stepsPerCm)
    return "0";
  if(distBR.distance() <= wallDistance - backDist + 3 && distBL.distance() <= wallDistance - backDist + 3)
    return "1";
  return "-1";
}

void stopMotors()
{
  MLB->run(BRAKE);
  MRB->run(BRAKE);
  MRF->run(BRAKE);
  MLF->run(BRAKE);
  delay(20);
}

void turnRight(int angle)
{
  // lcd.setCursor(0,0);
  // lcd.print("IMU:");
  float oPos = getOrientation();
  float ePos = oPos + angle > 360 ? oPos - 360 + angle : oPos + angle;
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  MLB->setSpeed(140);
  MRB->setSpeed(140);
  MRF->setSpeed(140);
  MLF->setSpeed(140);
  MLB->run(FORWARD);
  MRB->run(BACKWARD);
  MRF->run(BACKWARD);
  MLF->run(FORWARD);
  if(oLim > iLim)
    while(!(oPos >= iLim && oPos <= oLim))
    {
      // lcd.setCursor(4,0);
      // lcd.print("      ");
      oPos = getOrientation();
      // lcd.setCursor(4,0);
      // lcd.print(oPos);
    }
  else
    while(!(oPos >= iLim || oPos <= oLim))
    {
      // lcd.setCursor(4,0);
      // lcd.print("      ");
      oPos = getOrientation();
      // lcd.setCursor(4,0);
      // lcd.print(oPos);
    }
  stopMotors();
  MLB->setSpeed(leftMotorSpeed);
  MRB->setSpeed(rightMotorSpeed);
  MRF->setSpeed(rightMotorSpeed);
  MLF->setSpeed(leftMotorSpeed);
}

void turnLeft(int angle)
{
  // lcd.setCursor(0,0);
  // lcd.print("IMU:");
  float oPos = getOrientation();
  float ePos = oPos - angle < 0 ? oPos + 360 - angle : oPos - angle;
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  MLB->setSpeed(140);
  MRB->setSpeed(140);
  MRF->setSpeed(140);
  MLF->setSpeed(140);
  MLB->run(BACKWARD);
  MRB->run(FORWARD);
  MRF->run(FORWARD);
  MLF->run(BACKWARD);
  if(oLim > iLim)
    while(!(oPos >= iLim && oPos <= oLim))
    {
      // lcd.setCursor(4,0);
      // lcd.print("      ");
      oPos = getOrientation();
      // lcd.setCursor(4,0);
      // lcd.print(oPos);
    }
  else
    while(!(oPos >= iLim || oPos <= oLim))
    {
      // lcd.setCursor(4,0);
      // lcd.print("      ");
      oPos = getOrientation();
      // lcd.setCursor(4,0);
      // lcd.print(oPos);
    }
  stopMotors();
  MLB->setSpeed(leftMotorSpeed);
  MRB->setSpeed(rightMotorSpeed);
  MRF->setSpeed(rightMotorSpeed);
  MLF->setSpeed(leftMotorSpeed);
}

void turnTo(int angle)
{
    int angleNeeded = angle - getOrientation();
    if(angleNeeded > 180)
      turnLeft(360-angle+getOrientation());
    else if(angleNeeded > 0)
      turnRight(angleNeeded);
    else if(angleNeeded > -180)
      turnLeft(-angleNeeded);
    else
      turnRight(360-getOrientation()+angle);
}

//--------------------- Servos ----------------------------//
void openGripper()
{
  gripperA.attach(servoGA);
  int openPos = 40;
  for (int pos = gripperA.read(); pos > openPos; pos -= 1)
  {
    gripperA.write(pos);
    delay(15);  
  } 
  gripperA.detach();
}

void closeGripper()
{
  gripperA.attach(servoGA);
  int closePos = 70;
  for (int pos = gripperA.read(); pos <= closePos; pos += 1)
  {
    gripperA.write(pos);
    delay(15);  
  }
  gripperA.detach();
}

void horizontalMovementIn(byte velocity)
{
  horizontalM.attach(servoHM);
  horizontalM.write(velocity); //85
}

void horizontalMovementOut(byte velocity)
{
  horizontalM.attach(servoHM);
  horizontalM.write(velocity); //95
}

void stopHorizontalMovement()
{
  horizontalM.write(90);
  horizontalM.detach();
}

void gripperMovementIn(byte velocity)
{
  gripperM.attach(servoGM);
  gripperM.write(velocity); //95
}

void gripperMovementOut(byte velocity)
{
  gripperM.attach(servoGM);
  gripperM.write(velocity); //80
}

void stopGripperMovement()
{
  gripperM.write(90);
  gripperM.detach();
}

void asoMovementIn(byte velocity)
{
  asoM.attach(servoAM);
  asoM.write(100);  // 100
}

void asoMovementOut(byte velocity)
{
  asoM.attach(servoAM);
  asoM.write(80); // 80
}

void stopAsoMovement()
{
  asoM.write(90);
  asoM.detach();
}

void milk(byte velocity)
{
  levas.attach(servoL);
  levas.write(velocity); // 180
}

void stopMilking()
{
  levas.write(90);
  levas.detach();
}

//--------------------- IMU utilities ---------------------//
byte getIMUCalStatus()
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return gyro;
}

float getOrientation()
{
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

//--------------------- Special Functions ----------------//
void locateTerrine()
{
  float ePos = getOrientation();
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  clk = millis();

  float distanceFR = distFR.distance();
  float distanceFL = distFL.distance();
  float distanceGripper = distG.distance();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("FR:");
  // lcd.print(distanceFR);
  // lcd.setCursor(0,1);
  // lcd.print("FL:");
  // lcd.print(distanceFL);
  // lcd.setCursor(8,0);
  // lcd.print("Gr:");
  // lcd.print(distanceGripper);
  //Serial.println(distanceGripper);
  //if(distanceGripper <= 14)
  //  moveForward(3);
  distanceGripper = distG.distance();
  clk = millis();
  int modeData = 0;
  int numData = 0;
  int lastState = 99999;
  while(distanceFR > wallDistance && distanceFL > wallDistance && millis() - clk < 1000)
  {
    //Serial.println(distanceGripper);
    moveForwardStraight(ePos, iLim, oLim);

    distanceFR = distFR.distance();
    distanceFL = distFL.distance();
    distanceGripper = distG.distance();

    if(distanceGripper == modeData)
    {
      numData++;
      if(numData >= 5)
      {
        if(modeData < lastState)
        {
          lastState = modeData;
        }
        else
        {
          break;
        }
      }
    }
    else    
    {
      modeData = distanceGripper;
      numData = 1;
    }

    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("FR:");
    // lcd.print(distanceFR);
    // lcd.setCursor(0,1);
    // lcd.print("FL:");
    // lcd.print(distanceFL);
    // lcd.setCursor(8,0);
    // lcd.print("Gr:");
    // lcd.print(distanceGripper);
  }
  /*
  if(distanceGripper <= 14)
  {
    MLB->run(FORWARD);
    MRB->run(FORWARD);
    MRF->run(FORWARD);
    MLF->run(FORWARD);
    delay(240);
  } */
  stopMotors();
  //Serial.println(distanceGripper);
}

int enterCow()
{
  bool legLTDetected = false;
  bool legLBDetected = false;
  bool legRTDetected = false;
  bool legRBDetected = false;
  bool legFRDetected = false;
  bool legFLDetected = false;

  bool timeout = false;
  clk = millis();
  while(!(legLTDetected && legRTDetected) && distFR.distance() > wallDistance + 5 && distFL.distance() > wallDistance + 5 && !timeout)
  {
    timeout = millis() - clk > 12500;
    if(!legLTDetected && !legRTDetected)
    {
      MLB->run(FORWARD);
      MRB->run(FORWARD);
      MRF->run(FORWARD);
      MLF->run(FORWARD);
      int distanceLT = distLT.distance();
      int distanceRT = distRT.distance();
      //Serial.print("LT: ");
      //Serial.print(distanceLT);
      //Serial.print("\tRT: ");
      //Serial.println(distanceRT);
      legLTDetected = distanceLT > wallDistance + 5 ? legLTDetected : true;
      legRTDetected = distanceRT > wallDistance + 5 ? legRTDetected : true;
    }
    else if(!legLTDetected && legRTDetected)
    {
      MRB->run(BRAKE);
      MRF->run(BRAKE);
      MLB->setSpeed(140);
      MLF->setSpeed(140);
      int distanceLT = distLT.distance();
      int distanceRT = distRT.distance();
      //Serial.print("LT: ");
      //Serial.print(distanceLT);
      //Serial.print("\tRT: ");
      //Serial.println(distanceRT);
      legLTDetected = distanceLT > wallDistance + 5 ? legLTDetected : true;
      legRTDetected = distanceRT > wallDistance + 5 ? legRTDetected : true;
    }
    else if(legLTDetected && !legRTDetected)
    {
      MLB->run(BRAKE);
      MLF->run(BRAKE);
      MRB->setSpeed(140);
      MRF->setSpeed(140);
      int distanceLT = distLT.distance();
      int distanceRT = distRT.distance();
      //Serial.print("LT: ");
      //Serial.print(distanceLT);
      //Serial.print("\tRT: ");
      //Serial.println(distanceRT);
      legLTDetected = distanceLT > wallDistance + 5 ? legLTDetected : true;
      legRTDetected = distanceRT > wallDistance + 5 ? legRTDetected : true;
    }
    //Serial.print("TIME: ");
    //Serial.println(millis() - clk);
  }
  stopMotors();
  MLB->setSpeed(leftMotorSpeed);
  MRB->setSpeed(rightMotorSpeed);
  MRF->setSpeed(rightMotorSpeed);
  MLF->setSpeed(leftMotorSpeed);
  if(distFL.distance() <= wallDistance + 5)
    return 1;
  else if(distFR.distance() <= wallDistance + 5)
    return 2;
  else if(timeout)
    return -2;
  else 
  {
    timeout = false;
    clk = millis();
    while(!(legRTDetected && legRBDetected) && !timeout)
    {
      timeout = millis() - clk > 2000;
      MLB->setSpeed(leftMotorSpeed);
      MRB->setSpeed(rightMotorSpeed);
      MRF->setSpeed(rightMotorSpeed);
      MLF->setSpeed(leftMotorSpeed);
      MLB->run(FORWARD);
      MRB->run(FORWARD);
      MRF->run(FORWARD);
      MLF->run(FORWARD);
      int distanceLB = distLB.distance();
      int distanceRB = distRB.distance();
      //Serial.print("LB: ");
      //Serial.print(distanceLB);
      //Serial.print("\tRB: ");
      //Serial.println(distanceRB);
      legLBDetected = distanceLB > wallDistance + 5 ? legLBDetected : true;
      legRBDetected = distanceRB > wallDistance + 5 ? legRBDetected : true;
      //Serial.print("TIME: ");
      //Serial.println(millis() - clk);
    }
    stopMotors();
    MLB->setSpeed(leftMotorSpeed);
    MRB->setSpeed(rightMotorSpeed);
    MRF->setSpeed(rightMotorSpeed);
    MLF->setSpeed(leftMotorSpeed);
    if(timeout)
      return 3;
    return 0;
  }
}

//--------------------- Initial Setup ---------------------//
void setup() {
  // init serial
  Serial.begin(9600);

  // init encoders
  pinMode(encoderF, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderF), addFrontStep, CHANGE);
  
  // init motors
  AFMS.begin();
  MLB->setSpeed(leftMotorSpeed);
  MRB->setSpeed(rightMotorSpeed);
  MRF->setSpeed(rightMotorSpeed);
  MLF->setSpeed(leftMotorSpeed);
  MLB->run(FORWARD);
  MRB->run(FORWARD);
  MRF->run(FORWARD);
  MLF->run(FORWARD);
  MLB->run(RELEASE);
  MRB->run(RELEASE);
  MRF->run(RELEASE);
  MLF->run(RELEASE);

  //init Limit Switches
  pinMode (limitSwitchHMI, INPUT);
  pinMode (limitSwitchHMO, INPUT);
  pinMode (limitSwitchGMI, INPUT);
  pinMode (limitSwitchGMO, INPUT);
  pinMode (limitSwitchAMI, INPUT);
  pinMode (limitSwitchAMO, INPUT);

  // init Sharps
  pinMode (sharpFL, INPUT);
  pinMode (sharpFR, INPUT);
  pinMode (sharpRT, INPUT);
  pinMode (sharpRB, INPUT);
  pinMode (sharpBL, INPUT);
  pinMode (sharpBR, INPUT);
  pinMode (sharpLT, INPUT);
  pinMode (sharpLB, INPUT); 
  pinMode (sharpG, INPUT);
  
  // waiting for Raspberry to boot
  while(Serial.available() <= 0);
  Serial.print(Serial.readString());
}

//--------------------- Main program ----------------------//
void loop() {    
  // TODO: Being able to start in different corners
  if(Serial.available() > 0)
  {
    switch(Serial.read() - 48)
    {
      case 0:
      {
        Serial.println("Case 0: Opening Gripper");
        openGripper();
      }
      break;
      case 1:
      {
        Serial.println("Case 1: Closing Gripper");
        closeGripper();
      }
      break;
      case 2:
      {
        Serial.println("Case 2: Horizontal Movement Out"); 
        horizontalMovementOut(95);
        delay(1000);
        stopHorizontalMovement();
      }
      break;
      case 3:
      {
        Serial.println("Case 3: Horizontal Movement In"); 
        horizontalMovementIn(85);
        delay(1000);
        stopHorizontalMovement();
      }
      break;
      case 4:
      {
        Serial.println("Case 4: Gripper Movement Out"); 
        gripperMovementOut(80);
        delay(1000);
        stopGripperMovement();
      }
      break;
      case 5:
      {
        Serial.println("Case 5: Gripper Movement In"); 
        gripperMovementIn(95);
        delay(1000);
        stopGripperMovement();
      }
      break;
      case 6:
      {
        Serial.println("Case 6: ASO Movement Out"); 
        asoMovementOut(40);
        delay(5000);
        stopAsoMovement();
      }
      break;
      case 7:
      {
        Serial.println("Case 7: ASO Movement In"); 
        asoMovementIn(100);
        delay(1000);
        stopAsoMovement();
      }
      break;
      case 8:
      {
        Serial.println("Case 8: Milking"); 
        milk(180);
        delay(5000);
        stopMilking();
      }
      break;
    }
  }
}

void addFrontStep()
{
  stepsF++;
}
