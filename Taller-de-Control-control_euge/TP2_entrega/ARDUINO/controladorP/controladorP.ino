//-------------------------Librerías---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------

const int servoPin = 9;
const int potPin = A0;
const int periodo = 20; // 20 ms de periodo del servo
unsigned long tiempoMuestra = 0;

Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

const int periodoMuestreo = 20; // cada cuanto tiempo se toman muestras (ms)

Servo servo;
int PMIN = 500;   // Pulso mínimo (ajustar según el servo)
int PMAX = 2500;  // Pulso máximo (ajustar según el servo)
const float angServoEq = 90;
const float angRef = 13.93;

const float theta0 = -2.6; // theta del péndulo en equilibrio (corrección de la IMU)
float theta = 0;   // Ángulo actual
float u = 90;      // Acción de control actual
float u_print = 0; // Para enviar a imprimir si se desea

float theta_ref = 0; // Entrada de referencia

float GIRO;
float phi;
float phi0 = -5;

const float T = 0.02;
const float Ts = 0.02;

// VARIABLES CONTROLADOR P
float kp = 0.4;

//--------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  //------------------------Inicialización de la IMU------------------------
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(1000);

  //------------------------Inicialización del Servo------------------------
  servo.attach(servoPin, PMIN, PMAX);
}

void loop() {
  sensors_event_t a, g, temp;
  unsigned long tiempoActual = millis();

  if ((tiempoActual - tiempoMuestra >= periodoMuestreo)) {
    //---------------Calcular Theta-------------------------
    mpu.getEvent(&a, &g, &temp);
    float gyrox = g.gyro.x;
    float anguloRotG = anguloRotAnt + gyrox * periodoMuestreo / 1000.0;
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z);
    float anguloRot = (1 - alpha) * anguloRotG + alpha * anguloRotA;

    theta = (anguloRot * 180 / PI - theta0);
    anguloRotAnt = anguloRot;

    //---------------Calcular Phi----------------------------
    GIRO = analogRead(potPin);
    phi = map(GIRO, 0, 1023, -90, 270) - phi0;

    //---------------Control---------------------------------
    u = controlP(theta_ref, theta);
    servo.write(u);
    matlab_send(theta, phi, u);

    //---------------Enviar a MATLAB (opcional)--------------
    // matlab_send(theta, phi, u);

    tiempoMuestra = tiempoActual;
    delay(15); // Pequeño delay adicional
  }
}

//-----------------------Controlador Proporcional--------------------------
float controlP(float referencia, float salida) {
  float error = -(referencia - salida);
  float u = kp * error;

  // Saturación
  if (u < -45) u = -45;
  if (u > 45)  u = 45;

  u = constrain(u + 90, 0, 180); // Centrado en 90°
  return u;
}

//-----------------------Envío de datos a MATLAB (opcional)----------------
void matlab_send(float dato1, float dato2, float dato3) {
  Serial.write(97);  // 'a'
  Serial.write(98);  // 'b'
  Serial.write(99);  // 'c'
  Serial.write(100); // 'd'

  byte* b0 = (byte*)&dato1;
  Serial.write(b0, 4);

  byte* b1 = (byte*)&dato2;
  Serial.write(b1, 4);

  byte* b2 = (byte*)&dato3;
  Serial.write(b2, 4);
}
