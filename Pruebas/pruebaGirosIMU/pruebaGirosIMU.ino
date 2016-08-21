// Prueba giros con IMU

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

LiquidCrystal lcd(52, 50, 48, 49, 46, 47);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const float precisionIMU = 2.0;

void stopMotors()
{
  M1->run(RELEASE);
  M2->run(RELEASE);
  M3->run(RELEASE);
  M4->run(RELEASE);
  delay(20);
}

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

void turnRight(float angle)
{
  float oPos = getOrientation();
  float ePos = oPos + angle > 360 ? oPos - 360 + angle : oPos + angle;
  float iLim = ePos - precisionIMU <= 0 ? ePos + 360 - precisionIMU : ePos - precisionIMU;
  float oLim = ePos + precisionIMU > 360 ? ePos - 360 + precisionIMU : ePos + precisionIMU;
  M1->run(FORWARD);
  M2->run(BACKWARD);
  M3->run(FORWARD);
  M4->run(BACKWARD);
  while(!(oPos >= iLim && oPos <= oLim))
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("IMU: ");
    oPos = getOrientation();
    lcd.print(oPos);
  }
  stopMotors();
}

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  
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

  if(!bno.begin())
  {
    lcd.setCursor(0,0);
    lcd.print("IMU not working");
    delay(3000);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  turnRight(90.0);
  delay(2000);
}
