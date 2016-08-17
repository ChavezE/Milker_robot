//<------------------Libraries----------------------------->
//Lcd display
#include <LiquidCrystal.h>
//Ultrasonic sensors
#include <NewPing.h>

//<----------------------------Pins------------------------>

//declaring encoder pin and volatile variable for counting
const byte encoderI = 2;

volatile int count;

//declaring ultrasonic pins
const byte trigPin1 = 13;
const byte echoPin1 = 12;

const byte trigPin2 = A2;
const byte echoPin2 = A3;

const byte trigPin3 = A0;
const byte echoPin3 = A1;

const byte trigPin4 = A11;
const byte echoPin4 = A10;

const byte trigPin5 = 51;
const byte echoPin5 = 53;

const byte trigPin6 = 31;
const byte echoPin6 = 33;

const byte maxDistance = 200;

//declaring motors pins motorRightFrontForward
// TODO: Usaremos 2 o 4 motores?
const byte BackLeftFor = 11;
const byte BackLeftBack = 10;

const byte BackRightFor = 5;
const byte BackRightBack = 4;

const byte FrontLeftBack = 8;
const byte FrontLeftFor = 9;

const byte FrontRightBack = 7;
const byte FrontRightFor = 6;

//declaring led pins
const byte led1 = 23;
const byte led2 = 25;

//sharp reading distance values
short voltage[]={90,102,113,123,136,148,169,190,215,255,315,410,525};
short distance[]={140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20};

//variables for moving correction
float diffLeft = 0, diffRight = 0;

//<----------------------------Global Variables------------------------>

//storing distance of every ultrasonic sensor
double arrUltra1[]= {0, 0, 0};
double arrUltra2[]= {0, 0, 0};
double arrUltra3[]= {0, 0, 0};
double arrUltra4[]= {0, 0, 0};
double arrUltra5[]= {0, 0, 0};
double arrUltra6[]= {0, 0, 0};
short arrIrFront[]= {0, 0, 0, 0, 0, 0, 0, 0};
short arrIrBack[]= {0, 0, 0, 0, 0, 0, 0, 0};

// EL ARREGLO arrDist[] guarda de la siguiente manera las distancias {U1, U2, U3, U4, U5, U6}
double arrDist[] = {0,0,0,0,0,0};

// El arreglo guarda las distancias de los IR
short arrIR[] = {0,0};

// lcd initializing
LiquidCrystal lcd(52, 50, 48, 49, 46, 47);

//<----------------------------Global Variables------------------------>

//declaring control global variables

// TODO: Cambiar resolución del mapa y funcionalidades
//map multidimentional array code:
// 0: Hasn't been there
// 1: Has been there
// 2: Clear hall
// 3: Wall
// 4: Victim detected
// 5: Black tile
// 6: Ramp Bottom
// 7: Ramp Top
// 8: Initial / Final place
// 9: Clear and no go hall
byte iMap[33][33];

//declaring maping variables
byte iYMap=15, iXMap=15;

//<----------------------------Motors------------------------>
void forward(int iRotations)
{
  count = 0;
  while(count < iRotations)
  { 
    analogWrite(FrontLeftFor, 250);
    analogWrite(BackLeftFor, 250);
    analogWrite(BackRightFor, 235);
    analogWrite(FrontRightFor, 235); 
  }
  stopMotors();
}

void back(int iRotations)
{
  count = 0; 
  while(count < iRotations)
  {
     analogWrite(FrontLeftBack, 210);
     analogWrite(BackLeftBack, 220);
     analogWrite(FrontRightBack, 200);
     analogWrite(BackRightBack, 200);
  }
  stopMotors();
}


void turnLeft(int iRotations)
{
  count=0;
  while(count < iRotations)
    {
    analogWrite(FrontRightFor, 220);
    analogWrite(BackRightFor, 220);
    analogWrite(BackLeftBack, 220);
    analogWrite(FrontLeftBack, 220);
    }
  stopMotors();
}

void turnRight(int iRotations)
{
  count=0;
  while(count < iRotations)
  { 
    analogWrite(FrontRightBack, 220);
    analogWrite(BackRightBack, 220);
    analogWrite(BackLeftFor, 220);
    analogWrite(FrontLeftFor, 220);
  }
  stopMotors();
}

void stopMotors()
{
  analogWrite(FrontLeftFor, 0);
  analogWrite(BackLeftFor, 0);
  analogWrite(FrontRightFor, 0);
  analogWrite(BackRightFor, 0);

  analogWrite(FrontLeftBack, 0);
  analogWrite(BackLeftBack, 0);
  analogWrite(FrontRightBack, 0);
  analogWrite(BackRightBack, 0);
  delay(20);
}

//<----------------------------Distance------------------------>

//initializing distance variables
NewPing ultra1(trigPin1, echoPin1, maxDistance);

NewPing ultra2(trigPin2, echoPin2, maxDistance);

NewPing ultra3(trigPin3, echoPin3, maxDistance);

NewPing ultra4(trigPin4, echoPin4, maxDistance);

NewPing ultra5(trigPin5, echoPin5, maxDistance);

NewPing ultra6(trigPin6, echoPin6, maxDistance);

void calculateDiffLeft()
{
  diffLeft = abs(arrDist[0] - arrDist[1]);
}

void calculateDiffRight()
{
  diffRight = abs(arrDist[4] - arrDist[5]);
}

void readDist1()
{
  arrDist[0] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra1[iA] = ultra1.ping() / US_ROUNDTRIP_CM;
    arrDist[0] += arrUltra1[iA];
    delay(10);
  }
  arrDist[0] /= 3;
}

void readDist2()
{
  arrDist[1] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra2[iA] = ultra2.ping() / US_ROUNDTRIP_CM;
    arrDist[1] += arrUltra2[iA];
    delay(10);
  }
  arrDist[1] /= 3;
}

void readDist3()
{
  arrDist[2] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra3[iA] = ultra3.ping() / US_ROUNDTRIP_CM;
    arrDist[2] += arrUltra3[iA];
    delay(10);
  }
  arrDist[2] /= 3;
}

void readDist4()
{
  arrDist[3] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra4[iA] = ultra4.ping() / US_ROUNDTRIP_CM;
    arrDist[3] += arrUltra4[iA];
    delay(10);
  }
  arrDist[3] /= 3;
}

void readDist5()
{
  arrDist[4] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra5[iA] = ultra5.ping() / US_ROUNDTRIP_CM;
    arrDist[4] += arrUltra5[iA];
    delay(5);
  }
  arrDist[4] /= 3;
}

void readDist6()
{
  arrDist[5] = 0;
  for(byte iA = 0; iA < 3; iA++)
  {
    arrUltra6[iA] = (ultra6.ping() / US_ROUNDTRIP_CM) - 1;
    if(arrUltra6[iA] < 0)
      arrUltra6[iA] = 0;
    arrDist[5] += arrUltra6[iA];
    delay(10);
  }
  arrDist[5] /= 3;
}

void readDistIR1()
{
  double b = 0, m = 0;  
  arrIR[0] = 0;
  
  for(int iA = 0; iA < 8; iA++){
    arrIrFront[iA] = analogRead(A7);
    arrIR[0] += arrIrFront[iA];
  }  
    
  arrIR[0] = arrIR[0] >> 3; 
  
  /*lcd.setCursor(0,1);
  lcd.print("irF:");
  lcd.print(arrIR[0]);
  delay(100);
  lcd.clear();
  */
 if(arrIR[0] <125){
      arrIR[0]=0;   
    }
    else if(arrIR[0] >400){
      arrIR[0]=0;
      }
    else
      for(byte iA = 0; iA < 13; iA++){
   
          if(arrIR[0] >= voltage[iA] && arrIR[0] <= voltage[iA+1])
            {
                m = (voltage[iA + 1] - voltage[iA]) / (distance[iA + 1] - distance[iA]);
                b = voltage[iA + 1] - (m * distance[iA + 1]);
                //distance = (voltage - b) / m
                arrIR[0] = (arrIR[0] - b) / m; //-----------------> DISTANCIA AQUI
            }
      }
  /*lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("irF:");
  lcd.print(arrIR[0]);
  delay(100);**/
}


void readDistIR2()
{
   double b = 0, m = 0;  
  arrIR[1] = 0;

  for(int iA = 0; iA < 8; iA++){
    arrIrBack[iA] = analogRead(A5);
    arrIR[1] += arrIrBack[iA];
  }  
    
  arrIR[1] = arrIR[1] >> 3; 
  
  for(byte iA = 0; iA < 13; iA++)
    if(arrIR[1] >= voltage[iA] && arrIR[1] <= voltage[iA+1])
    {
      m = (voltage[iA + 1] - voltage[iA]) / (distance[iA + 1] - distance[iA]);
      b = voltage[iA + 1] - (m * distance[iA + 1]);
      //distance = (voltage - b) / m
      arrIR[1] = (arrIR[1] - b) / m; //-----------------> DISTANCIA AQUI
    }
    /*
  lcd.clear();
  lcd.setCursor(7,1);
  lcd.print("irB:");
  lcd.print(arrIR[1]);
  delay(100);
  **/
}

void readIR()
{
  readDistIR1();
  readDistIR2();
}

void readDistLeft()
{
  readDist1();
  readDist2();
}

void readDistFront()
{
  readDist3();
  readDist4();
}

void readDistRight()
{
  readDist5();
  readDist6();
}

void movementCorrectionLeft()
{ // Utiliza U1 y U2  
  //Esta corrección es para tratar de centrarse en el cuadro
    readDistLeft();
    
      while((arrDist[0] <= 4 || arrDist[1] <= 4) && (arrDist[0] != 0 && arrDist[1] != 0)){
                    slideRight();  
                    readDistLeft();
      }
                  stopMotors();
      while((arrDist[0] >= 10 || arrDist[1] >= 10) && (arrDist[0] != 0 && arrDist[1] != 0)){   
        slideLeft();
                    readDistLeft();
      }  
                  stopMotors();
  readDistLeft();
  calculateDiffLeft();    
  if(diffLeft >= maxInclinationDistance && arrDist[0] != 0 && arrDist[1] != 0){              
    double U1 = arrDist[0];
    double U2 = arrDist[1];
    if(U1 > U2){
      while(U1 - U2 >= maxInclinationDistance){
        turnRight(8);
  
        readDistLeft();
        calculateDiffLeft();
        U1 = arrDist[0];
        U2 = arrDist[1];
        }      
        }
        else{
    while(U2 - U1 >= maxInclinationDistance){
      
            turnLeft(8);
            readDistLeft();
      calculateDiffLeft();
      U1 = arrDist[0];
      U2 = arrDist[1];
     }
  }
  }
}

void movementCorrectionRight(){
 //Esta corrección es para tratar de centrarse en el cuadro
        readDistRight();
        while((arrDist[4] <= 4 || arrDist[5] <= 4) && (arrDist[4] != 0 && arrDist[5] != 0)){
             slideLeft();
             readDistRight();
        }
        
        stopMotors();
        while((arrDist[4] > 8 || arrDist[5] > 8) && (arrDist[4] != 0 && arrDist[5] != 0)){
             slideRight();
             readDistRight();
        }
        stopMotors();
  // Esta corrección endereza al Robot
        readDistRight();
   calculateDiffRight();
  if(diffRight >= maxInclinationDistance && (arrDist[4] != 0 && arrDist[5] != 0)){
    double U5 = arrDist[4];
    double U6 = arrDist[5];
    if(U6 > U5){
      while(U6 - U5 >= maxInclinationDistance){
        turnLeft(8);
                  
        readDistRight();
        calculateDiffRight();
        U5 = arrDist[4];
        U6 = arrDist[5];
      }
    }
    else{
      while(U5 - U6 >= maxInclinationDistance){
        turnRight(8);
        readDistRight();
        calculateDiffRight();
        U5 = arrDist[4];
        U6 = arrDist[5];
      }
    }
  }  
}

void movementCorrectionFront(){
  readDistFront();
  readDistIR1();
  while((arrDist[2] != 0 && arrDist[2] <= 3.5))
  {
    back(20);
    readDistFront();
  }
  stopMotors();
  while((arrDist[2] != 0 && arrDist[2] >= 9))
  {
    forward(20);
    readDistFront();
  }
  stopMotors();
  while(arrDist[2] == 0 && arrIR[0] != 0 && (arrIR[0] % 30) <= 5)
  {
    //lcd.setCursor(0,0);
    //lcd.print(arrIR[0] % 30);
    back(10);
    readDistIR1();
    readDistFront();
  }
  stopMotors();
  while(arrDist[2] == 0 && arrIR[0] != 0 && (arrIR[0] % 30) > 12)
  {
    //lcd.setCursor(0,0);
    //lcd.print(arrIR[0] % 30);
    forward(10);
    readDistIR1();
    readDistFront();
  }
  stopMotors();
}

void movementCorrection()
{
  
  readDistLeft();
  readDistRight();
  if(arrDist[0] != 0 && arrDist[1] != 0)
    movementCorrectionLeft();
  else if(arrDist[4] != 0 && arrDist[5] != 0) 
    movementCorrectionRight(); 
    
  movementCorrectionFront();
}


//<-----------------------------Extras------------------------>
void led1ON()
{
  digitalWrite(led1,HIGH);
}

void led1OFF()
{
  digitalWrite(led1,LOW);
}

void led2ON()
{
  digitalWrite(led2,HIGH);
}

void led2OFF()
{
  digitalWrite(led2,LOW);
}

void ultraLcdPrint()
{
  lcd.setCursor(0,1);
  lcd.print(arrDist[0]);
  lcd.print(" ");
  lcd.print(arrDist[1]);
  lcd.print(" ");
  lcd.print(arrDist[2]);  
  lcd.print(" ");
  lcd.print(arrDist[3]);
  lcd.print(" ");
  lcd.print(arrDist[4]);
  lcd.print(" ");
  lcd.print(arrDist[5]);
}

//<--------------------Main Functions------------------------>



//<-----------------------Main Core------------------------>


//<----------------------------Set Up------------------------>
void setup()
{
  Serial.begin(9600);

  //initializing lcd with message
  lcd.begin(16, 2);

  //initializing arrays with 0 and assign initial position value
  memset(iMap,0,sizeof(byte)*33*33);
  iMap[iYMap][iXMap] = 6; // TODO: ver valor inicial en matriz

  //<---------------------------Motors------------------------>
  pinMode(FrontLeftFor, OUTPUT);
  pinMode(BackLeftFor, OUTPUT);
  pinMode(BackRightFor, OUTPUT);
  pinMode(FrontRightFor, OUTPUT);

  pinMode(FrontLeftBack, OUTPUT);
  pinMode(BackLeftBack, OUTPUT);
  pinMode(BackRightBack, OUTPUT);
  pinMode(FrontRightBack, OUTPUT);

  //Encoders
 
  pinMode(encoderI, INPUT);
  attachInterrupt(0, handleEncoder, RISING);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  
  led1ON();
  delay(2000);
  led1OFF();
}

//<-----------------------------Loop------------------------>
void loop()
{
  if (!exchangeZoneIsAvailable)
    locateExchangeZone();
  locateTerrine();
  pickTerrine();
  if (!cowIsAvailable)
    locateClosestCow();
  goNerbyCow();
  turnOnVision();
  while (!cowIsInFront || !cowIsCentered)
  {
    if (!cowIsInFront)
      moveToSide();
    if (!cowIsCentered)
      moveCowCenter();
  }
  turnOffVision();
  getUnderCow();
  milkCow();
  getOutofCow();
  if (!exitIsAvailable)
    locateExit();
  goToExit();
  if (!containerIsAvailable)
    locateContainer();
  goToContainer();
  positionBesideContainer();
  pourMilk();
  if (!emptyTerrineZoneIsAvailable)
    locateEmptyTerrineZone();
  goToEmptyTerrineZone();
  locateFreeSpace();
  dropTerrine();
  goToExit();
}

void handleEncoder()
{
if(digitalRead(encoderI) == HIGH)
 count++;
else
 count--;
}