// Prueba giros con IMU
// I2C addresses
// IMU: 0x28
// Motor driver: 0x60
// LCD: 0x27
// M4: Left Front
// M3: Right Front
// M2: Right Back
// M1: Left Back

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

LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const float precisionIMU = 2.0;

unsigned long clk;

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

void setup() {
  // put your setup code here, to run once:
  lcd.begin();
  lcd.backlight();

  Serial.begin(9600);
  
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
}

void loop() {
  // put your main code here, to run repeatedly:
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
  turnRight(90);
  delay(3000);
}
