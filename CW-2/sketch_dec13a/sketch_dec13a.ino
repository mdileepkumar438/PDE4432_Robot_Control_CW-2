/*------------------------------------------------------------------
7 sensors Smart Robot - Maze Solver and Line Follower with programable PID controller via BT
==> Basic movement based on Nano Mouse Robot, developed by Michael Backus (http://www.akrobotnerd.com/ )
==> Line follow based on http://samvrit.tk/tutorials/pid-control-arduino-line-follower-robot/?ckattempt=1
==> CREDIT to Patrick McCabe for the path Solving Code, visit patrickmccabemakes.com!!
Marcelo Jose Rovai - 23 April, 2016 - Visit: http://mjrobot.org
-------------------------------------------------------------------*/

#include <Servo.h>
#include "robotDefines.h"

String command;
String device;

// BT Module
#include <SoftwareSerial.h>
SoftwareSerial BT1(10, 11);  // El pin 10 es Rx y el pin 11 es Tx

void ledBlink(void) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

//-----------------------------------------------------------------------------

void checkBTcmd() {
  while (BT1.available())  //Check if there is an available byte to read
  {
    delay(10);            //Delay added to make thing stable
    char c = BT1.read();  //Conduct a serial read
    device += c;          //build the string.
  }
  if (device.length() > 0) {
    Serial.print("Command received from BT ==> ");
    Serial.println(device);
    command = device;
    device = "";  //Reset the variable
    BT1.flush();
  }
}

//------------------------------------------------------------------------
void manualCmd() {
  switch (command[0]) {
    case 'g':
      mode = FOLLOWING_LINE;
      break;

    case 's':
      motorStop();  //turn off both motors
      break;

    case 'f':
      motorForward();
      break;

    case 'r':
      motorTurn(RIGHT, 30);
      motorStop();

      break;

    case 'l':
      motorTurn(LEFT, 30);
      motorStop();
      break;

    case 'b':
      motorBackward();
      break;

    case 'p':
      Kp = command[2];
      break;

    case 'i':
      Ki = command[2];
      break;

    case 'd':
      Kd = command[2];
      break;
  }
}

//------------------------------------------------------------------------
void sendBTdata(int data)  // send data to BT

{
  digitalWrite(ledPin, HIGH);
  BT1.print("Data from Arduino");
  BT1.print(data);
  BT1.print(" xx");
  BT1.println('\n');
  digitalWrite(ledPin, LOW);
}

//--------------------------------------------------------
void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}

//--------------------------------------------------------
void checkPIDvalues() {

  BT1.print("PID: ");
  BT1.print(Kp);
  BT1.print(" - ");
  BT1.print(Ki);
  BT1.print(" - ");
  BT1.println(Kd);

  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);
}

//-----------------------------------------------
void testLineFollowSensors() {
  int LFS0 = digitalRead(lineFollowSensor0);
  int LFS1 = digitalRead(lineFollowSensor1);
  int LFS2 = digitalRead(lineFollowSensor2);
  int LFS3 = digitalRead(lineFollowSensor3);
  int LFS4 = digitalRead(lineFollowSensor4);

  Serial.print("LFS: L  0 1 2 3 4  R ==> ");
  Serial.print(LFS0);
  Serial.print(" ");
  Serial.print(LFS1);
  Serial.print(" ");
  Serial.print(LFS2);
  Serial.print(" ");
  Serial.print(LFS3);
  Serial.print(" ");
  Serial.print(LFS4);
  Serial.print("  ==> ");

  Serial.print(" P: ");
  Serial.print(P);
  Serial.print(" I: ");
  Serial.print(I);
  Serial.print(" D: ");
  Serial.print(D);
  Serial.print(" PID: ");
  Serial.println(PIDvalue);
}

void mazeEnd(void) {
  motorStop();
  BT1.print("The End");
  mode = STOPPED;
  while (digitalRead(buttonPin)) {
    ledBlink();
  }
}

//----------------------------------------------
void followingLine(void) {
  readLFSsensors();
  calculatePID();
  motorPIDcontrol();
}


void motorStop() {
  leftServo.writeMicroseconds(1500);
  rightServo.writeMicroseconds(1500);
  delay(200);
}

//---------------------------------------------
void motorForward() {
  leftServo.writeMicroseconds(1500 - (power + adj));
  rightServo.writeMicroseconds(1500 + power);
}

//---------------------------------------------
void motorBackward() {
  leftServo.writeMicroseconds(1500 + power);
  rightServo.writeMicroseconds(1500 - power);
}

//---------------------------------------------
void motorFwTime(unsigned int time) {
  motorForward();
  delay(time);
  motorStop();
}

//---------------------------------------------
void motorBwTime(unsigned int time) {
  motorBackward();
  delay(time);
  motorStop();
}

//------------------------------------------------
void motorTurn(int direction, int degrees) {
  leftServo.writeMicroseconds(1500 - (iniMotorPower + adj) * direction);
  rightServo.writeMicroseconds(1500 - iniMotorPower * direction);
  delay(round(adjTurn * degrees + 1));
  motorStop();
}

//---------------------------------------------------
void motorPIDcontrol() {

  int leftMotorSpeed = 1500 - (iniMotorPower + adj) - PIDvalue;
  int rightMotorSpeed = 1500 + iniMotorPower - PIDvalue;

  // The motor speed should not exceed the max PWM value
  constrain(leftMotorSpeed, 1000, 2000);
  constrain(rightMotorSpeed, 1000, 2000);

  leftServo.writeMicroseconds(leftMotorSpeed);
  rightServo.writeMicroseconds(rightMotorSpeed);

  //Serial.print (PIDvalue);
  //Serial.print (" ==> Left, Right:  ");
  //Serial.print (leftMotorSpeed);
  //Serial.print ("   ");
  //Serial.println (rightMotorSpeed);
}

//---------------------------------------------------
void runExtraInch(void) {
  motorPIDcontrol();
  delay(extraInch);
  motorStop();
}

//---------------------------------------------------
void goAndTurn(int direction, int degrees) {
  motorPIDcontrol();
  delay(adjGoAndTurn);
  motorTurn(direction, degrees);
}
//-----------------------------
void drivePolygon(unsigned int time, int sides)  // for motor test only
{
  for (int i = 0; i < sides; i++) {
    motorFwTime(time);
    motorTurn(RIGHT, 360 / sides);
  }
}

int mode = 0;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6

const int power = 250;
const int iniMotorPower = 250;
const int adj = 100;
float adjTurn = 9;
int extraInch = 200;
int adjGoAndTurn = 950;

const int ledPin = 13;
const int buttonPin = 9;

// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 12;
const int lineFollowSensor1 = 18;
const int lineFollowSensor2 = 17;
const int lineFollowSensor3 = 16;
const int lineFollowSensor4 = 19;

const int farRightSensorPin = 0;  //Analog Pin A0
const int farLeftSensorPin = 1;   //Analog Pin A0
const int THRESHOLD = 200;
int farRightSensor = 0;
int farLeftSensor = 0;


int LFSensor[5] = { 0, 0, 0, 0, 0 };

// PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

#define RIGHT 1
#define LEFT -1

Servo leftServo;
Servo rightServo;


//-------------------------------------------------------------
/* read line sensors values 
Sensor Array 	Error Value
0 0 0 0 1	 4              
0 0 0 1 1	 3              
0 0 0 1 0	 2              
0 0 1 1 0	 1              
0 0 1 0 0	 0              
0 1 1 0 0	-1              
0 1 0 0 0	-2              
1 1 0 0 0	-3              
1 0 0 0 0	-4    

0 0 1 1 1        

1 1 1 1 1        0 Robot found continuous line - test if an intersection or end of maze
0 0 0 0 0        0 Robot found no line: turn 180o
*/
void readLFSsensors() {
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);
  LFSensor[3] = digitalRead(lineFollowSensor3);
  LFSensor[4] = digitalRead(lineFollowSensor4);

  farRightSensor = analogRead(farRightSensorPin);
  farLeftSensor = analogRead(farLeftSensorPin);

  if ((farLeftSensor < THRESHOLD) && (farRightSensor < THRESHOLD)) {
    mode = CONT_LINE;
    error = 0;
  } else if (farRightSensor < THRESHOLD) {
    mode = RIGHT_TURN;
    error = 0;
  } else if (farLeftSensor < THRESHOLD) {
    mode = LEFT_TURN;
    error = 0;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = NO_LINE;
    error = 0;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 4;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 3;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = 2;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = 1;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = 0;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -1;
  } else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -2;
  } else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -3;
  } else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -4;
  }

  Serial.print(farLeftSensor);
  Serial.print(" <== LEFT  RIGH==> ");
  Serial.print(farRightSensor);
  Serial.print("  mode: ");
  Serial.print(mode);
  Serial.print("  error:");
  Serial.println(error);
}


//---------------------------------------------
void setup() {

  Serial.begin(9600);
  BT1.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // line follow sensors
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);

  // servos
  leftServo.attach(5);
  rightServo.attach(3);

  BT1.print("check the PID constants to be sent to Robot");
  BT1.println('\n');




  while (digitalRead(buttonPin) && !mode) {
    checkBTcmd();  // verify if a comand is received from BT remote control
    manualCmd();
    command = "";
  }

  motorFwTime(3000);

  checkPIDvalues();
  mode = STOPPED;
}

void loop() {
  readLFSsensors();
  switch (mode) {
    case NO_LINE:
      motorStop();
      goAndTurn(LEFT, 180);
      break;

    case CONT_LINE:
      runExtraInch();
      readLFSsensors();
      if (mode == CONT_LINE) mazeEnd();
      else goAndTurn(LEFT, 90);  // or it is a "T" or "Cross"). In both cases, goes to LEFT
      break;

    case RIGHT_TURN:
      runExtraInch();
      readLFSsensors();
      if (mode == NO_LINE) goAndTurn(RIGHT, 90);
      break;

    case LEFT_TURN:
      goAndTurn(LEFT, 90);
      break;

    case FOLLOWING_LINE:
      followingLine();
      break;
  }
}
