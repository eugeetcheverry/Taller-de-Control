#include <Arduino.h>

int potPin = A0;    // Pin analógico donde conectamos el potenciómetro
int valor = 0;      // Variable para guardar la lectura

void setup() {
  Serial.begin(9600);  // Inicia comunicación serie
}

void loop() {
  valor = analogRead(potPin);   // Lee el valor (0 a 1023)
  Serial.println(valor);        // Muestra el valor por el monitor serie
  delay(100);                   // Pequeña pausa
}
