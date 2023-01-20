#include <ros.h>
#include <LiquidCrystal_I2C.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#define USE_USBCON

ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::String str;
std_msgs::Int32 data;
ros::Publisher pub_data( "/ultrasound_data", &data);

LiquidCrystal_I2C lcd(0x27, 16, 2);

int trigger_pin = 9;
int echo_pin = 8;


long range_time;

void messageCb( const std_msgs::String& incoming_msg )
{
  String  str = incoming_msg.data; 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(str);
  Serial.println(str);
}
ros::Subscriber<std_msgs::String> sub_data("Ultrasonic_data_sub", &messageCb );

void setup() {
  lcd.init();
  lcd.clear();
  lcd.backlight();

  nh.initNode();
  
  nh.advertise(pub_data);
  nh.subscribe(sub_data);
  Serial.begin(57600);

  lcd.begin(16, 2);
  lcd.print("Starting System");
  delay(2000);
  lcd.clear();
  lcd.print("Detecting object");
  delay(1000);
  
  
}

void loop()
{
  if ( millis() >= range_time ){ 
      data.data = getRange();
      
      pub_data.publish(&data);
      range_time =  millis() + 500;
    }    
    nh.spinOnce();
    
}

long microsecondsToCentimeters(long microseconds)
{

return microseconds / 29 / 2;
}

int getRange()
{
  long duration, inches, cm;
  
  pinMode(trigger_pin, OUTPUT);
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  
  pinMode(echo_pin, INPUT);
  duration = pulseIn(echo_pin, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  //Serial.println(cm);
  // convert the time into a distance
  return cm;
}
