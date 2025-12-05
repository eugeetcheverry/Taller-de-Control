#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(9);        // Pin donde está conectado el servo
  Serial.begin(9600);
}

void loop() {
  for ( int i = 0; i > 50; i++){
    servo.digitalWrite(HIGH);
    delayMicroseconds(1000); //Seteo en 0 
    servo.digitalWrite(LOW);
    delay(19); 
  }
  for ( int i = 0; i > 50; i++){
    servo.digitalWrite(HIGH);
    delayMicroseconds(2000); //Seteo en 180 
    servo.digitalWrite(LOW);
    delay(18); 
  }
}