
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
int mode = 0;

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2
# define CONT_LINE 3
# define POS_LINE 4
# define RIGHT_TURN 5
# define LEFT_TURN 6
// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 12; 
const int lineFollowSensor1 = 18; 
const int lineFollowSensor2 = 17; 
const int lineFollowSensor3 = 16;
const int lineFollowSensor4 = 19;


int LFSensor[5]={0, 0, 0, 0, 0};


// PID controller
float Kp=50;
float Ki=0;
float Kd=0;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

//IR revciever 
int RECV_PIN = 5;
IRrecv irrecv(RECV_PIN);
decode_results results;

//16*2 LCD Display 
LiquidCrystal_I2C lcd(0x27, 2, 16);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int i = 0;

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
  1 1 1 1 1        0 Robot found continuous line - test if an intersection or end of maze
  0 0 0 0 0        0 Robot found no line: turn 180o
*/

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
int motor1pin1 = 6;
int motor1pin2 = 7;

int motor2pin1 = 8;
int motor2pin2 = 9;

void setup() {
  lcd.init();
  lcd.clear();
  lcd.backlight();  // Make sure backlight is on
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // lcd.createChar(1, Heart);
  // lcd.createChar(2, Z1);
  // lcd.createChar(3, Z2);
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

//--------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}


//-----------------------------------------------
void testLineFollowSensors()
{
     int LFS0 = digitalRead(lineFollowSensor0);
     int LFS1 = digitalRead(lineFollowSensor1);
     int LFS2 = digitalRead(lineFollowSensor2);
     int LFS3 = digitalRead(lineFollowSensor3);
     int LFS4 = digitalRead(lineFollowSensor4);
     
     Serial.print ("LFS: L  0 1 2 3 4  R ==> "); 
     Serial.print (LFS0); 
     Serial.print (" ");
     Serial.print (LFS1); 
     Serial.print (" ");
     Serial.print (LFS2); 
     Serial.print (" ");
     Serial.print (LFS3); 
     Serial.print (" ");
     Serial.print (LFS4); 
     Serial.print ("  ==> ");
    
     Serial.print (" P: ");
     Serial.print (P);
     Serial.print (" I: ");
     Serial.print (I);
     Serial.print (" D: ");
     Serial.print (D);
     Serial.print (" PID: ");
     Serial.println (PIDvalue);
}


void readLFSsensors()
{
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);
  LFSensor[3] = digitalRead(lineFollowSensor3);
  LFSensor[4] = digitalRead(lineFollowSensor4);
  
  farRightSensor = analogRead(farRightSensorPin);
  farLeftSensor = analogRead(farLeftSensorPin);
  
  if     ((farLeftSensor <  THRESHOLD) &&                                        (farRightSensor < THRESHOLD)) {mode = CONT_LINE; error = 0;}
  else if                                                                        (farRightSensor < THRESHOLD)  {mode = RIGHT_TURN; error = 0;}
  else if (farLeftSensor <  THRESHOLD)                                                                         {mode = LEFT_TURN; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = NO_LINE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error =- 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 . ))  {mode = FOLLOWING_LINE; error = -3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -4;}
  
  Serial.print (farLeftSensor);
  Serial.print (" <== LEFT  RIGH==> ");
  Serial.print (farRightSensor);
  Serial.print ("  mode: ");
  Serial.print (mode);
  Serial.print ("  error:");
  Serial.println (error);
    
}



void manualcmd(){

  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF629D:  //1 Button
        Serial.print("Button Pressed 1 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print(" forward");  // Button 2

        forward();
        delay(3000);
        stop();
        //lcd.setCursor(0, 1);
        //lcd.print("How Are You");
        break;
      case 0xFFA857:  //2 Button
        Serial.print("Button Pressed 2 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Backward");  // Button 2
        Backward();
        delay(3000);
        stop();
        break;
      case 0xFF22DD:  //3 Button
        Serial.print("Button Pressed 3 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Left");  // Button 2
        Left();
        delay(1500);
        stop();

        break;

      case 0xFFC23D:  //4 Button
        Serial.print("Button Pressed 4 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Right");  // Button 2
        Right();
        delay(1500);
        stop();
        break;

      case 0xFF02FD:  //5 Button
        Serial.print("Button Pressed 5 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Button Press 5");  // Button 2
        

        //lcd.setCursor(0, 1);
        //lcd.print("Custom Icon");
        break;


      case 0xFF7A85:  //6 Button
        Serial.print("Button Pressed 6 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("  I ");
        lcd.write(1);
        lcd.print(" ARDUINO");
        lcd.setCursor(0, 1);
        lcd.print("  I sleep ");
        lcd.write(2);
        lcd.write(3);
        break;


      case 0xFF10EF:  //7 Button
        Serial.print("Button Pressed 6 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Button Press 6");  // Button 2

        break;


      case 0xFF38C7:  //8 Button
        Serial.print("Button Pressed 7 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Button Press 7");  // Button 2
        lcd.setCursor(0, 1);
        lcd.print("Animate Text");
        break;

      case 0xFF5AA5:  //9 Button
        Serial.print("BLinking");

        lcd.begin(16, 2);

        lcd.print("BLinking");  // Button 2
        lcd.setCursor(0, 1);
        lcd.print("    Text    ");
        lcd.noBlink();
        delay(2000);
        lcd.blink();
        delay(2000);

      case 0xFF4AB5:  //0 Button
        Serial.print("Scroll");

        lcd.setCursor(15, 0);
        lcd.print("I LOVE");
        delay(500);
        for (i = 0; i < 20; i++) {
          lcd.scrollDisplayLeft();
          delay(150);
        }
        lcd.clear();
        lcd.setCursor(15, 0);
        lcd.print("ARDUINO");
        delay(500);
        for (i = 0; i < 22; i++) {
          lcd.scrollDisplayLeft();
          delay(150);
        }
        lcd.clear();


        break;
      default:
        Serial.print("Undefined code received: 0x");
        Serial.println(results.value, HEX);
        lcd.begin(16, 2);
        lcd.print("Undefined Button");
        lcd.setCursor(0, 1);
        lcd.print(results.value, HEX);
        break;
    }
    irrecv.resume();
  }
}

void motorforward() {

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void motorstop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void motorBackward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void motorLeft() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void motorRight() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
void loop() {
  
}