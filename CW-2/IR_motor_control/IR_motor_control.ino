
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

int RECV_PIN = 5;
IRrecv irrecv(RECV_PIN);
decode_results results;

LiquidCrystal_I2C lcd(0x27, 2, 16);  // set the LCD address to 0x3F for a 16 chars and 2 line display
int i = 0;


// Remote button testing / configuring (2 meters range)
// 0xFF6897    = 1
// 0xFF9867    = 2
// 0xFFB04F    = 3
// 0xFF30CF    = 4
// 0xFF18E7    = 5
// 0xFF7A85    = 6
// 0xFF10EF    = 7
// 0xFF38C7    = 8
// 0xFF5AA5    = 9
// 0xFF4AB5    = 0
// 0xFF42BD = *
// 0xFF52AD = #
// 0xFF02FD = 'ok'
// 0xFF22DD = <
// 0xFFC23D = >
// 0xFF629D = up arrow
// 0xFFA857 = down arrow

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

void loop() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF629D:  //1 Button
        Serial.print("Button Pressed 1 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print(" forward");  // Button 2
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);

        delay(3000);
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, LOW);
        //lcd.setCursor(0, 1);
        //lcd.print("How Are You");
        break;
      case 0xFFA857:  //2 Button
        Serial.print("Button Pressed 2 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Backward");  // Button 2
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, HIGH);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, HIGH);
        delay(3000);
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, LOW);
        break;
      case 0xFF22DD:  //3 Button
        Serial.print("Button Pressed 3 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Left");  // Button 2
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, HIGH);
        delay(1500);
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, LOW);


        break;

      case 0xFFC23D:  //4 Button
        Serial.print("Button Pressed 4 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Right");  // Button 2
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, HIGH);

        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);
        delay(1500);
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, LOW);
        break;

      case 0xFF02FD:  //5 Button
        Serial.print("Button Pressed 5 Passing Text to LCD");
        lcd.begin(16, 2);
        lcd.print("Button Press 5");  // Button 2
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);

        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, LOW);

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