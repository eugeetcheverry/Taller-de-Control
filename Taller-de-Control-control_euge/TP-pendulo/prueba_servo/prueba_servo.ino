#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------
const int servoPin = 3;
Servo servo;

void setup() {

  Serial.begin(115200);
  delay(1000);
  servo.attach(servoPin);
  delay(1000);

}

void loop(){

  servo.write(45);
  Serial.print("45 grad \n");
  delay(5000);
    Serial.print("0 grad \n");
  servo.write(0);
  delay(5000);
}