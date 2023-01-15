#include <IRremote.h>

const int line_pin[5] = { 8, 9, 10, 11, 12 };
int line_val[5] = { 0 };
int reading[5] = { 0 };
//String IR_val;
int i;
// DC motor
int motor1pin1 = 4;
int motor1pin2 = 5;
int motor2pin1 = 6;
int motor2pin2 = 7;

void setup() {

  Serial.begin(9600);
  for (i = 0; i < 5; i++) {
    pinMode(line_pin[i], INPUT);
  }
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void read_sensor() {

  for (i = 0; i < 5; i++) {
    line_val[i] = digitalRead(line_pin[i]);
  }
}
void forward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, LOW);
}

void back() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void right() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void left() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void stop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void loop() {
  read_sensor();
int r = (line_val[0], line_val[1], line_val[2] ,line_val[3] ,line_val[4]);
  Serial.println(r);
}