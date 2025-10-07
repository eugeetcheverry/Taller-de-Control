#include <Servo.h>

Servo servo;
const int pinServo = 9;    // Pin PWM donde está conectado el servo

// --- Límites físicos del servo (ajustar según montaje real) ---
const int ANGULO_MIN = 60;   // Ángulo mínimo permitido
const int ANGULO_MAX = 120;  // Ángulo máximo permitido

// --- Función que mueve el servo a un ángulo de referencia ---
void moverServo(int angRef) {
  // Saturación: limita el valor a los rangos físicos del brazo
  if (angRef < ANGULO_MIN) angRef = ANGULO_MIN;
  if (angRef > ANGULO_MAX) angRef = ANGULO_MAX;

  servo.write(angRef);   // Mueve el servo
  delay(15);             // Pequeño retardo para estabilizar el movimiento

  // Mostrar por monitor serie
  Serial.print("Ángulo solicitado: ");
  Serial.print(angRef);
  Serial.println("°");
}

void setup() {
  servo.attach(pinServo);
  Serial.begin(9600);
  Serial.println("Listo. Ingrese ángulo deseado por Monitor Serie (en grados)");
}

void loop() {
  // Si el usuario ingresa un número por el monitor serie, se mueve el servo
  if (Serial.available()) {
    int ref = Serial.parseInt();  // Leer valor ingresado
    moverServo(ref);              // Llamar a la función
  }
}
