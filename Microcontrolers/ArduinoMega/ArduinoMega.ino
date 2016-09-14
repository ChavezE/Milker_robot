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
// M4: Left Front
// M3: Right Front
// M2: Right Back
// M1: Left Back

//--------------------- Libraries -------------------------//
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//--------------------- Pins ------------------------------//
const uint8_t sharpFL = 0;		// sharpFrontLeft
const uint8_t sharpFR = 0;		// sharpFrontRight
const uint8_t sharpRT = A7;		// sharpRightTop
const uint8_t sharpRB = A8;		// sharpRightBottom
const uint8_t sharpBL = A11;	// sharpBackLeft
const uint8_t sharpBR = A12;	// sharpBackRight
const uint8_t sharpLT = A14;	// sharpLeftTop
const uint8_t sharpLB = A13;	// sharpLeftBottom

//--------------------- Motors ----------------------------//
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

//--------------------- LCD -------------------------------//
LiquidCrystal_I2C lcd(0x27, 16, 2);

//--------------------- IMU -------------------------------//
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//--------------------- Constants -------------------------//
const float precisionIMU = 2.0;
const unsigned int analogWallDistance = 400;

//--------------------- Global variables ------------------//
unsigned long clk;

//--------------------- Control movements -----------------//
void moveBackward()
{
  M1->run(BACKWARD);
  M2->run(BACKWARD);
  M3->run(BACKWARD);
  M4->run(BACKWARD);
}

void stopMotors()
{
  M1->run(RELEASE);
  M2->run(RELEASE);
  M3->run(RELEASE);
  M4->run(RELEASE);
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
  M1->run(FORWARD);
  M2->run(BACKWARD);
  M3->run(BACKWARD);
  M4->run(FORWARD);
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
  M1->run(BACKWARD);
  M2->run(FORWARD);
  M3->run(FORWARD);
  M4->run(BACKWARD);
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

  // init lcd
  lcd.begin();
  lcd.backlight();
  
  // init motors
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
}

//--------------------- Main program ----------------------//
void loop() {
  if(Serial.available() > 0)
  {
  	switch(Serial.readString().toInt())
  	{
  		// 1.- Getting ready in the empty terrines zone
  		case : 1
  			while(analogRead(sharpBR) < analogWallDistance)
  				moveBackward();
  			stopMotors();
  			if(analogRead(sharpLT) < analogWallDistance)
  				Serial.print("-1");
  			else
  				Serial.print("0");
  		break;

  		// 2.- Find terrine
  		case : 2
  		break;

  		// 3.- Grab terrine
  		case : 3
  		break;

  		// 4.- Move in grid searching the cow
  		case : 4
  			// Serial order: 
  			// #1 - move(0) or turn(1)
  			// #2 - cm or degrees
  			while(Serial.available() < 0);
			if(Serial.readString().toInt())
			{
				while(Serial.available() < 0);
				int angle = Serial.readString().toInt();
			    lcd.clear();
			    lcd.setCursor(12,0);
			    lcd.print("0:");
			    lcd.print(angle);
			    if(angle > 0)
			      turnRight(angle);
			    else
			      turnLeft(angle);
			    Serial.print("0");
			}
			else
			{
				while(Serial.available() < 0);
				// TODO: Implement here
			}
  		break;

  		// 5.- Detect cow with vision (null, raspberry work)
  		case : 5
  		break;

  		// 6.- Position in the cow lateral
  		case : 6
  		break;

  		// 7.- Enter below cow
  		case : 7
  		break;

  		// 8.- Milk cow
  		case : 8
  		break;

  		// 9.- Exit cow
  		case : 9
  		break;

  		// 10.- Return to empty terrines zone
  		case : 10
  		break;

  		// 11.- Search door
  		case : 11
  		break;

  		// 12.- Localize milk tank
  		case : 12
  		break;

  		// 13.- Move towards milk tank
  		case : 13
  		break;

  		// 14.- Getting ready to deposit milk
  		case : 14
  		break;

  		// 15.- Poor milk in tank
  		case : 15
  		break;

  		// 16.- Move towards exchange zone
  		case : 16
  		break;

  		// 17.- Drop terrine
  		case : 17
  		break;

  		// 18.- Search door (null, see case 11)
  		case : 18
  		break;

  		// 19.- Go to empty terrines zone
  		case : 19
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
