long sensor[] = {0, 1, 2, 3, 4}; //leftmost - 0, rightmost - 4
long sensor_average;
int sensor_sum;
void setup() {
  // put your setup code here, to run once:
//sensors
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
readSensors();

}

void readSensors()	//accepts input from sensors
{
	sensor[0]  = analogRead(A0);
sensor[1]  = analogRead(A1);
sensor[2]  = analogRead(A2);
sensor[3]  = analogRead(A3);
sensor[4]  = analogRead(A4);
Serial.print(sensor[0]);
Serial.print (",");
Serial.print(sensor[1]);
Serial.print (",");
Serial.print(sensor[2]);
Serial.print (",");
Serial.print(sensor[3]);
Serial.print (",");
Serial.print(sensor[4]);
Serial.println(" ");


}