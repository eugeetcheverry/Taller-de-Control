#include <Servo.h>

Servo servo;  
int potPin = A0;
int valor;
int angulo;

unsigned long lastSample = 0;
unsigned long sampleInterval = 10; // 100 Hz = cada 10 ms

void setup() {
  servo.attach(9);        // Pin donde está conectado el servo
  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample >= sampleInterval) {
    lastSample = now;

    valor = analogRead(potPin);              // 0-1023
    angulo = map(valor, 0, 1023, 0, 180);    // Servo: 0° a 180°
   // angulo = ;
    servo.write(angulo);  // Enviar ángulo al servo

    Serial.print("Pot: ");
    Serial.print(valor);
    Serial.print("  -> Servo: ");
    Serial.println(angulo);
  }
  //delay(1000);
}
