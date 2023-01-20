#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

//IR revciever
int RECV_PIN = 3;
IRrecv irrecv(RECV_PIN);
decode_results results;

//16*2 LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


int mode = 0;
const int pingPin = 9;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 8;  // Echo Pin of Ultrasonic Sensor
int inches;
int cm;
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6

int pos;

//speeds
int rspeed;
int lspeed;
const int base_speed = 255;

//to be pressed to find set point

// PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

//--------------------------------------------------------------
/* 
Remote button testing / configuring (2 meters range)
  0xFF6897 = 1
  0xFF9867 = 2
  0xFFB04F = 3
  0xFF30CF = 4
  0xFF18E7 = 5
  0xFF7A85 = 6
  0xFF10EF = 7
  0xFF38C7 = 8
  0xFF5AA5 = 9
  0xFF4AB5 = 0
  0xFF42BD = *
  0xFF52AD = #
  0xFF02FD = 'ok'
  0xFF22DD = <
  0xFFC23D = >
  0xFF629D = up arrow
  0xFFA857 = down arrow
*/
// DC motor
int motor1pin1 = 5;
int motor1pin2 = 6;

int motor2pin1 = 10;
int motor2pin2 = 11;

long sensor[] = { 0, 1, 2, 3, 4 };  //leftmost - 0, rightmost - 4
long sensor_average;
int sensor_sum;

//float Kp = 5;   // dummy
//float Ki = 0;   //dummy
//float Kd = 40;  //(Kp-1)*10
float t = millis() + 1500;
void pid_calc();
void calc_turn();
void motor_drive(int, int);
void motor_drive_2(int, int);
int station = 0;
void setup() {
  lcd.init();
  lcd.clear();
  lcd.backlight();

  // put your setup code here, to run once:

  //5 channel IR sensors
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(12, INPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  Serial.begin(9600);
  irrecv.enableIRIn();
  Serial.begin(9600);
  irrecv.enableIRIn();

  lcd.begin(16, 2);
  lcd.print("Starting System");
  delay(2000);
  lcd.clear();
  lcd.print("System on");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Button");
  lcd.setCursor(0, 1);
  lcd.print("To Get Value");
}
void check_object() {


  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  if (cm < 5) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("object is placed");

    Serial.println("object_Detected");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No object");
    Serial.println("No object");
  }
}
long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
long range_time;
void loop() {
  if (millis() >= range_time) {
    range_time = millis() + 500;
    check_object();
  }
  manualcmd();

}

int i =0;
void station_1() {
  readdigitalSensors();
  pid_calc();
  calc_turn();
  motor_drive(rspeed, lspeed);
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    stop();
    lcd.clear();
    lcd.setCursor(15, 0);
        lcd.print("Reached Station-1");
        delay(500);
        for (i = 0; i < 20; i++) {
          lcd.scrollDisplayLeft();
          delay(150);
        }
        lcd.clear();
    
    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Place Item for ");
    lcd.setCursor(0, 1);
    lcd.print("Delivery");
  }
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    Right();
  }

  //  if (station==1 && (sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
  //    Right();
  //    stop();
  //}
}

void station_2() {
  while ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    readdigitalSensors();
    pid_calc();
    calc_turn();

    motor_drive_2(rspeed, lspeed);
  }
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    Right();
    if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
      stop();
    }
  }

  //  if (station==1 && (sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
  //    Right();
  //    stop();
  //}
}

void forward() {

  analogWrite(motor2pin1, 0);
  analogWrite(motor2pin2, 255);
  analogWrite(motor1pin1, 0);
  analogWrite(motor1pin2, 255);
  //
  //  digitalWrite(motor1pin1, HIGH);
  //  digitalWrite(motor1pin2, LOW);
  //
  //  digitalWrite(motor2pin1, HIGH);
  //  digitalWrite(motor2pin2, LOW);
  //
}

void stop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void Backward() {

  analogWrite(motor2pin1, 255);
  analogWrite(motor2pin2, 0);
  analogWrite(motor1pin1, 255);
  analogWrite(motor1pin2, 0);
  //
  //  digitalWrite(motor1pin1, LOW);
  //  digitalWrite(motor1pin2, HIGH);
  //
  //  digitalWrite(motor2pin1, LOW);
  //  digitalWrite(motor2pin2, HIGH);
}

void Left() {

  analogWrite(motor1pin1, 255);
  analogWrite(motor1pin2, 0);

  analogWrite(motor2pin1, 0);
  analogWrite(motor2pin2, 255);
  
  //  digitalWrite(motor1pin1, HIGH);
  //  digitalWrite(motor1pin2, LOW);
  //
  //  digitalWrite(motor2pin1, LOW);
  //  digitalWrite(motor2pin2, HIGH);
}
void Right() {

  analogWrite(motor1pin1, 255);
  analogWrite(motor1pin2, 0);

  analogWrite(motor2pin1, 0);
  analogWrite(motor2pin2, 255);
  //
  //  digitalWrite(motor1pin1, 0);
  //  digitalWrite(motor1pin2, 1);
  //
  //  digitalWrite(motor2pin1, 1);
  //  digitalWrite(motor2pin2, 0);
}


void manualcmd() {
  if (irrecv.decode(&results)) {
    if (results.value == 0xFF6897) {
      //1 Button
      //Serial.print("Button Pressed 1 Passing Text to LCD");
      //count = count + 1;
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Going to Station 1");
      Serial.println("station 1");
     station_1();
    }
    if (results.value == 0xFF9867) {


      while ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
        readdigitalSensors();
        pid_calc();
        calc_turn();
        motor_drive_2(rspeed, lspeed);
      }
      Right();
      readdigitalSensors();
      pid_calc();
      calc_turn();
      motor_drive(rspeed, lspeed);
      if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
        stop();
      }
    }
    //irrecv.resume();
  }
}

void readanalogSensors() {
  sensor[0] = analogRead(A0);
  sensor[1] = analogRead(A1);
  sensor[2] = analogRead(A2);
  sensor[3] = analogRead(A3);
  sensor[4] = analogRead(A4);
  Serial.print(sensor[0]);
  Serial.print(",");
  Serial.print(sensor[1]);
  Serial.print(",");
  Serial.print(sensor[2]);
  Serial.print(",");
  Serial.print(sensor[3]);
  Serial.print(",");
  Serial.print(sensor[4]);
  Serial.println(" ");
}
void readdigitalSensors() {
  sensor[0] = digitalRead(A0);  //sensor data read from A0 arduino pin
  sensor[1] = digitalRead(A1);  //sensor data read from A1 arduino pin
  sensor[2] = digitalRead(A2);  //sensor data read from A2 arduino pin
  sensor[3] = digitalRead(A3);  //sensor data read from A3 arduino pin
  sensor[4] = digitalRead(12);  //sensor data read from A4 arduino pin
  Serial.print(sensor[0]);
  Serial.print(",");
  Serial.print(sensor[1]);
  Serial.print(",");
  Serial.print(sensor[2]);
  Serial.print(",");
  Serial.print(sensor[3]);
  Serial.print(",");
  Serial.print(sensor[4]);
  Serial.println(" ");

  //Serial.print(" P: ");
  //Serial.print(P);
  //Serial.print(" I: ");
  //Serial.print(I);
  //Serial.print(" D: ");
  //Serial.print(D);
  //Serial.print(" PID: ");
  //Serial.println(PIDvalue);

  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -4;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    error = -3;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = -2;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = -1;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 0;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 1;
  }

  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 2;
  }

  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 3;
  }

  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    mode = FOLLOWING_LINE;
    error = 4;
  }

  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    mode = FOLLOWING_LINE;
    stop();
    error = 0;
  }
}

void pid_calc() {

  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}

void calc_turn() {
  rspeed = base_speed + PIDvalue;
  lspeed = base_speed - PIDvalue;

  //restricting speeds of motors between 255 and -255

  //constrain(lspeed, -255, 255);
  //constrain(rspeed, -255, 255);


  if (rspeed > 255)
    rspeed = 255;

  if (lspeed > 255)
    lspeed = 255;

  if (rspeed < -255)
    rspeed = -255;

  if (lspeed < -255)
    lspeed = -255;


  //motor_drive(rspeed, lspeed);
}

void motor_drive(int rspeed, int lspeed) {


  //analogWrite(9,initial_motor_speed-PID_value);   //Left Motor Speed
  //analogWrite(10,initial_motor_speed+PID_value);
  analogWrite(motor2pin1, rspeed);
  analogWrite(motor2pin2, 0);
  analogWrite(motor1pin1, lspeed);
  analogWrite(motor1pin2, 0);
}

void motor_drive_2(int rspeed, int lspeed) {


  //analogWrite(9,initial_motor_speed-PID_value);   //Left Motor Speed
  //analogWrite(10,initial_motor_speed+PID_value);
  analogWrite(motor2pin1, rspeed);
  analogWrite(motor2pin2, 0);
  analogWrite(motor1pin1, lspeed);
  analogWrite(motor1pin2, 0);
}
