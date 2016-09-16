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

//--------------------- Libraries -------------------------//
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SharpIR.h>

//--------------------- Pins ------------------------------//
const uint8_t sharpFL = A10;	// sharpFrontLeft
const uint8_t sharpFR = A9;		// sharpFrontRight
const uint8_t sharpRT = A7;		// sharpRightTop
const uint8_t sharpRB = A8;		// sharpRightBottom
const uint8_t sharpBL = A11;	// sharpBackLeft
const uint8_t sharpBR = A12;	// sharpBackRight
const uint8_t sharpLT = A14;	// sharpLeftTop
const uint8_t sharpLB = A13;	// sharpLeftBottom

const uint8_t encoderF = 18;	// encoderFront
const uint8_t encoderB = 19;	// encoderBack

//--------------------- Motors ----------------------------//
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *MLB = AFMS.getMotor(1);
Adafruit_DCMotor *MRB = AFMS.getMotor(2);
Adafruit_DCMotor *MRF = AFMS.getMotor(3);
Adafruit_DCMotor *MLF = AFMS.getMotor(4);

//--------------------- LCD -------------------------------//
LiquidCrystal_I2C lcd(0x27, 16, 2);

//--------------------- IMU -------------------------------//
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//--------------------- Sharps -----------------------------//
SharpIR distFL(sharpFL, 25, 93, 2016);
SharpIR distFR(sharpFR, 25, 93, 2016);
SharpIR distRT(sharpRT, 25, 93, 2016);
SharpIR distRB(sharpRB, 25, 93, 2016);
SharpIR distBL(sharpBL, 25, 93, 2016);
SharpIR distBR(sharpBR, 25, 93, 2016);
SharpIR distLT(sharpLT, 25, 93, 2016);
SharpIR distLB(sharpLB, 25, 93, 2016);

//--------------------- Constants -------------------------//
const float precisionIMU = 2.0;
const unsigned int wallDistance = 15;
const int stepsPerCm = 200;

//--------------------- Global variables ------------------//
volatile unsigned int stepsF = 0;	// stepsFront
volatile unsigned int stepsB = 0;	// stepsBack

unsigned long clk;	//used to measure time

//--------------------- Control movements -----------------//
String moveForward(int distance)
{
  stepsF = 0;
  while(stepsF < distance*stepsPerCm && distFR.distance() > wallDistance && distFL.distance() > wallDistance)
  {
  	MLB->run(FORWARD);
	MRB->run(FORWARD);
	MRF->run(FORWARD);
	MLF->run(FORWARD);
  }
  stopMotors();
  if(stepsF >= distance*stepsPerCm)
  	return "0";
  return "-1";
}

String moveBackward(int distance)
{
  stepsF = 0;
  while(stepsF < distance*stepsPerCm && distBR.distance() > wallDistance && distBL.distance() > wallDistance)
  {
  	  MLB->run(BACKWARD);
	  MRB->run(BACKWARD);
	  MRF->run(BACKWARD);
	  MLF->run(BACKWARD);
  }
  stopMotors();
  if(stepsF >= distance*stepsPerCm)
  	return "0";
  return "-1";
}

void stopMotors()
{
  MLB->run(RELEASE);
  MRB->run(RELEASE);
  MRF->run(RELEASE);
  MLF->run(RELEASE);
  delay(20);
}

void turnRight(int angle)
{
  lcd.setCursor(0,0);
  lcd.print("IMU:");
  float oPos = getOrientation();
  float ePos = oPos + angle > 360 ? oPos - 360 + angle : oPos + angle;
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  MLB->run(FORWARD);
  MRB->run(BACKWARD);
  MRF->run(BACKWARD);
  MLF->run(FORWARD);
  while(!(oPos >= iLim && oPos <= oLim))
  {
    lcd.setCursor(4,0);
    lcd.print("      ");
    oPos = getOrientation();
    lcd.setCursor(4,0);
    lcd.print(oPos);
  }
  stopMotors();
}

void turnLeft(int angle)
{
  lcd.setCursor(0,0);
  lcd.print("IMU:");
  float oPos = getOrientation();
  float ePos = oPos - angle < 0 ? oPos + 360 - angle : oPos - angle;
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  MLB->run(BACKWARD);
  MRB->run(FORWARD);
  MRF->run(FORWARD);
  MLF->run(BACKWARD);
  while(!(oPos >= iLim && oPos <= oLim))
  {
    lcd.setCursor(4,0);
    lcd.print("      ");
    oPos = getOrientation();
    lcd.setCursor(4,0);
    lcd.print(oPos);
  }
  stopMotors();
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

//--------------------- Initial Setup ---------------------//
void setup() {
  // init serial
  Serial.begin(9600);

  // init encoders
  pinMode(encoderF, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderF), addFrontStep, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), addBackStep, CHANGE);

  // init lcd
  lcd.begin();
  lcd.backlight();
  
  // init motors
  AFMS.begin();
  MLB->setSpeed(100);
  MRB->setSpeed(100);
  MRF->setSpeed(100);
  MLF->setSpeed(100);
  MLB->run(FORWARD);
  MRB->run(FORWARD);
  MRF->run(FORWARD);
  MLF->run(FORWARD);
  MLB->run(RELEASE);
  MRB->run(RELEASE);
  MRF->run(RELEASE);
  MLF->run(RELEASE);

  // init Sharps
  pinMode (sharpFL, INPUT);
  pinMode (sharpFR, INPUT);
  pinMode (sharpRT, INPUT);
  pinMode (sharpRB, INPUT);
  pinMode (sharpBL, INPUT);
  pinMode (sharpBR, INPUT);
  pinMode (sharpLT, INPUT);
  pinMode (sharpLB, INPUT); 

  // init IMU
  if(!bno.begin())
  {
    lcd.setCursor(0,0);
    lcd.print("IMU not working");
    delay(1000);
    lcd.clear();
  }
  bno.setExtCrystalUse(true);
  lcd.setCursor(0,0);
  lcd.print("Calibrating IMU");
  while(getIMUCalStatus() <= 0);
  lcd.setCursor(0,1);
  lcd.print("Done");
  delay(500);
  lcd.clear();

  // waiting for Raspberry to boot
  /*
  while(Serial.available() <= 0)
  {
    lcd.setCursor(0,0);
    lcd.print("Booting Rasp...");
  }
  Serial.read();
  lcd.clear();
  lcd.print("Rasp: OK");
  delay(500);
  lcd.clear();
  */
}

//--------------------- Main program ----------------------//
void loop() {
  bool legLTDetected = false;
  bool legLBDetected = false;
  bool legRTDetected = false;
  bool legRBDetected = false;
  while(!(legLTDetected && legLBDetected && legRTDetected && legRBDetected) && distFL.distance() > wallDistance && distFR.distance() > wallDistance)
      {
        legLTDetected = distLT.distance() > wallDistance ? legLTDetected : true;
        legLBDetected = distLB.distance() > wallDistance ? legLBDetected : true;
        legRTDetected = distRT.distance() > wallDistance ? legRTDetected : true;
        legRBDetected = distRB.distance() > wallDistance ? legRBDetected : true;
        MLB->run(FORWARD);
        MRB->run(FORWARD);
        MRF->run(FORWARD);
        MLF->run(FORWARD);
        if(leg && distRT.distance() <= wallDistance)
        {
          MRB->setSpeed(0);
          MRF->setSpeed(0);
        }
        else if(distLT.distance() <= wallDistance && distRT.distance() > wallDistance)
        {
          MLB->setSpeed(0);
          MLF->setSpeed(0);
        }
        else if(distLT.distance() <= wallDistance && distRT.distance() <= wallDistance)
        {
          MLB->setSpeed(100);
          MRB->setSpeed(100);
          MRF->setSpeed(100);
          MLF->setSpeed(100);
        }
      }
      stopMotors();
      delay(10000);
  /*
  if(Serial.available() > 0)
  {
  	switch(Serial.readString().toInt())
  	{
  		// 1.- Getting ready in the empty terrines zone
  		case 1:
  			moveBackward(500);
  			if(distLT.distance() > wallDistance)
  				Serial.print("-1");
  			else
  				Serial.print("0");
  		break;

  		// 2.- Find terrine
  		case 2:
  		break;

  		// 3.- Grab terrine
  		case 3:
  		break;

  		// 4.- Move in grid searching the cow
  		case 4:
  			// Serial order: 
  			// #1 - move(0) or turn(1)
  			// #2 - cm or degrees
  			// Status:
  			// 0: OK
  			// -1: Cannot move that distance in that direction
  			while(Serial.available() < 0);
			if(Serial.readString().toInt())
			{
				while(Serial.available() < 0);
				int angle = Serial.readString().toInt();
			    lcd.clear();
			    lcd.setCursor(12,0);
			    lcd.print("0:");
			    lcd.print(angle);
			    if(angle > 0)		// + to the right and - to the left
			      turnRight(angle);
			    else
			      turnLeft(-angle);
			    Serial.print("0");
			}
			else
			{
				while(Serial.available() < 0);
				int distance = Serial.readString().toInt();
				lcd.clear();
			    lcd.setCursor(0,0);
			    lcd.print("Moving ");
			    lcd.print(distance);
			    lcd.print(" cm");
			    String status;
			    if(distance > 0)	// + forward and - backwards
			    	status = moveForward(distance);
			   	else 
			   		status = moveBackward(-distance);
			   	Serial.print(status);
			}
  		break;

  		// 5.- Detect cow with vision (null, raspberry work)
  		case 5:
  		break;

  		// 6.- Position in the cow lateral (null, use case 4)
  		case 6:
  		break;

  		// 7.- Enter below cow
  		case 7:
			while((distLT.distance() > wallDistance || distLB.distance() > wallDistance || distRT.distance() > wallDistance || distRB.distance() > wallDistance) && distFL.distance() > wallDistance && distFR.distance() > wallDistance)
			{
				MLB->run(FORWARD);
				MRB->run(FORWARD);
				MRF->run(FORWARD);
				MLF->run(FORWARD);
				if(distLT.distance() > wallDistance && distRT.distance() <= wallDistance)
				{
					MRB->setSpeed(0);
					MRF->setSpeed(0);
				}
				else if(distLT.distance() <= wallDistance && distRT.distance() > wallDistance)
				{
					MLB->setSpeed(0);
					MLF->setSpeed(0);
				}
				else if(distLT.distance() <= wallDistance && distRT.distance() <= wallDistance)
				{
					MLB->setSpeed(100);
					MRB->setSpeed(100);
					MRF->setSpeed(100);
					MLF->setSpeed(100);
				}
			}
			stopMotors();
			if(distFL.distance() <= wallDistance || distFR.distance() <= wallDistance)
				Serial.print("-1");
			Serial.print("0");
  		break;

  		// 8.- Milk cow
  		case 8:
  		break;

  		// 9.- Exit cow
  		case 9:

  		break;

  		// 10.- Return to empty terrines zone (null, use case 4)
  		case 10:
  		break;

  		// 11.- Search door
  		case 11:
  		
  		break;

  		// 12.- Localize milk tank
  		case 12:
  		break;

  		// 13.- Move towards milk tank
  		case 13:
  		break;

  		// 14.- Getting ready to deposit milk
  		case 14:
  		break;

  		// 15.- Poor milk in tank
  		case 15:
  		break;

  		// 16.- Move towards exchange zone
  		case 16:
  		break;

  		// 17.- Drop terrine
  		case 17:
  		break;

  		// 18.- Search door (null, see case 11)
  		case 18:
  		break;

  		// 19.- Go to empty terrines zone
  		case 19:
  		break;
  		default:
  			// return 't' meaning no correct value received, resend.
  			Serial.print('t');
  	}
  }
  /*
  clk = millis();
  Serial.print('R');
  while(Serial.available() <= 0 && millis() - clk <= 5000)
  {
    lcd.setCursor(0,0);
    lcd.print("Waiting...");
  }
  if(Serial.available() > 0)
  {
    int angle = Serial.readString().toInt();
    lcd.clear();
    lcd.setCursor(12,0);
    lcd.print("0:");
    lcd.print(angle);
    if(angle > 0)
      turnRight(angle);
    else
      turnLeft(angle);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Timeout!");
  }
  */
}

void addFrontStep()
{
	stepsF++;
}

void addBackStep()
{
	stepsB++;
}
