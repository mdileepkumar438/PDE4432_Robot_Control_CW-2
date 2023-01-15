#include <IRremote.h>


//speeds
int rspeed;
int lspeed;
const int base_speed = 255;

int pos;
long sensor_average;
int sensor_sum;

//int button = 3; //to be pressed to find set point

float p;
float i;
float d;
float lp;
float error;
float correction;
float sp;

float Kp = 5;   // dummy
float Ki = 0;   //dummy
float Kd = 40;  //(Kp-1)*10

void pid_calc();
void calc_turn();
void motor_drive(int, int);

int RECV_PIN = 5;
IRrecv irrecv(RECV_PIN);
decode_results results;
int count_stations = 0;


void setup() {

  while (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF02FD:  //ok Button
        sensor_average = 0;
        sensor_sum = 0;

        for (int i = -2; i <= 2; i++) {
          sensor[i] = analogRead(i);
          sensor_average += sensor[i] * i * 1000;  //weighted mean
          sensor_sum += int(sensor[i]);
        }
        pos = int(sensor_average / sensor_sum);

        break;

      default:
        Serial.print("Undefined button pressed");
        lcd.begin(16, 2);
        lcd.print("Undefined Button");
        break;
    }
  }
  sp = pos;
}
// put your setup code here, to run once:


void loop() {
  //while (irrecv.decode(&results))
  //{
  //  switch (results.value) {
  //    case 0xFF6897:  //ok Button
  //  }
  //}
  //// put your main code here, to run repeatedly:

  
  pid_calc();
  calc_turn();

}

void pid_calc() {
  sensor_average = 0;
  sensor_sum = 0;
  i = 0;

  for (int i = -2; i <= 2; i++) {
    sensor[i] = analogRead(i);
    sensor_average = sensor[i] * i * 1000;  //weighted mean
    sensor_sum += sensor[i];
  }

  pos = int(sensor_average / sensor_sum);

  error = pos - sp;

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  correction = int(Kp * p + Ki * i + Kd * d);
}

void calc_turn()
{
  rspeed = base_speed + correction;
  lspeed = base_speed - correction;

  //restricting speeds of motors between 255 and -255
  
  if (rspeed > 255) 
    rspeed = 255;
    
  if (lspeed > 255) 
    lspeed = 255;
    
  if (rspeed < -255) 
    rspeed = -255;
    
  if (lspeed < -255) 
    lspeed = -255;
 
 motor_drive(rspeed,lspeed);  
}

void motor_drive(int right, int left){
  
  if(right>0)
  {
    analogWrite(rmf, right);   
    analogWrite(rmb, 0);
  }
  else 
  {
    analogWrite(rmf, 0); 
    analogWrite(rmb, abs(right));
  }
  
 
  if(left>0)
  {
    analogWrite(lmf, left);
    analogWrite(lmb, 0);
  }
  else 
  {
    analogWrite(lmf, 0);
    analogWrite(lmb, abs(left));
  }
  
}