#include <SharpIR.h>

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
}
  

