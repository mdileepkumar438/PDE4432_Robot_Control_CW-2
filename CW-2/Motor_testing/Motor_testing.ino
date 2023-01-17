
// https://create.arduino.cc/projecthub/ryanchan/how-to-use-the-l298n-motor-driver-b124c5


int motor1pin1 = 5;
int motor1pin2 = 6;

int motor2pin1 = 10;
int motor2pin2 = 11;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  


  
}

void loop() {
   analogWrite(motor1pin1, 255);
  analogWrite(motor1pin2, 0);

  analogWrite(motor2pin1, 255);
  analogWrite(motor2pin2, 0); // put your main code here, to run repeatedly:   
  
}