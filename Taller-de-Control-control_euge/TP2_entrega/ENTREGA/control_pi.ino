//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------
const int servoPin = 9;
const int potPin = A0;
const int periodoMuestreo = 20;

unsigned long tiempoMuestra = 0;

Adafruit_MPU6050 mpu;

// Complementario
float anguloRotAnt = 0;
float alpha = 0.02;

// Servo
Servo servo;
int PMIN = 500;
int PMAX = 2500;
const float angServoEq = 90;

// Referencia
const float angRef = 0;      // ahora referencia en 0° (theta = 0)
const float theta0 = 0.543104171752930;   // offset IMU
const float phi0   = -5;

// Control PD
float kp = 0.9;
//float kd = 0.0;
float ki = 0.3;
float integral = 0;
float T  = 0.02;

float error_ant = 0;

// --------- Variables de calibración ---------
float gyroBiasX = 0;
float accelAngleOffset = 0;

// --------- Rutina de calibración ---------
void calibrarIMU(int N = 500) {

  Serial.println("Calibrando IMU... Mantener el pendulo quieto.");

  float sumaGyro = 0;
  float sumaAng = 0;

  for (int i = 0; i < N; i++) {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Ángulo estimado solo con acelerómetro
    float angA = atan2(a.acceleration.y, a.acceleration.z);

    sumaAng  += angA;
    sumaGyro += g.gyro.x;   // rad/s → bias

    delay(5);
  }

  gyroBiasX = sumaGyro / N;
  accelAngleOffset = sumaAng / N;

  Serial.println("Calibración lista.");
  Serial.print("Offset acelerometro (theta0) = ");
  Serial.println(accelAngleOffset, 6);
  Serial.print("Bias giroscopio (rad/s): ");
  Serial.println(gyroBiasX, 6);
}


//--------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("IMU error"); while(1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(500);

  servo.attach(servoPin, PMIN, PMAX);
  servo.write(angServoEq);

  delay(3000);
  calibrarIMU(1000);   // <<--- AGREGAR ESTO

  servo.attach(servoPin, PMIN, PMAX);
  servo.write(angServoEq);
  delay(1000);
}

//--------------------------------------------------------------------
void loop() {

  unsigned long tiempoActual = millis();

  if ((tiempoActual - tiempoMuestra) >= periodoMuestreo)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //----------------------------Theta IMU----------------------------
    float gyrox = g.gyro.x - gyroBiasX;
    float anguloRotG = anguloRotAnt + gyrox * (periodoMuestreo / 1000.0);
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z) - accelAngleOffset;
    float anguloRot = (1 - alpha) * anguloRotG + alpha * anguloRotA;

    float theta = -(anguloRot * 180.0 / PI - theta0) - 0.52;  
    anguloRotAnt = anguloRot;

    //----------------------------Phi Pot------------------------------
    int GIRO = analogRead(potPin);
    float phi = map(GIRO, 0, 1023, -90, 270) - phi0;

    //----------------------------Control PD---------------------------
    float u = controlPI(angRef, theta);
    servo.write(u);

    //Serial.println(theta);
    //Serial.println(u);
    //----------------------------Enviar a MATLAB----------------------
    matlab_send(theta, phi, u);

    tiempoMuestra = tiempoActual;
  }
}

//-----------------------Envio de datos a MATLAB---------------------
void matlab_send(float dato1, float dato2, float dato3){
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);

  byte* b0 = (byte*) &dato1; Serial.write(b0,4);
  byte* b1 = (byte*) &dato2; Serial.write(b1,4);
  byte* b2 = (byte*) &dato3; Serial.write(b2,4);
}

// --------------------- Control PI ----------------------------
float controlPI(float referencia, float salida) {

  float error = referencia - salida;

  // ---- Ganancias ----
  float P = kp * error;
  float I = ki * (error_ant + error) * (T / 2.0);   // integración trapezoidal

  integral += I;                                    // acumular integral

  // Anti-windup (opcional pero recomendable)
  integral = constrain(integral, -50, 50);

  float u = angServoEq + P + integral;

  error_ant = error;

  return constrain(u, 0, 180);
}