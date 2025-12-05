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
const float angRef = 6.24;      // referencia en 0°
const float theta0 = 0.543104171752930;
const float phi0   = -5;

// Control PD
float kp = 0.9;
float kd = -0.01;          // <--- AJUSTAR ESTE (nuevo)
float T  = 0.02;

float error_ant = 0;
bool firstSample = true;

// Filtro derivada (nuevo)
float D_prev = 0;
float alphaD = 0.25;

float controlPD(float referencia, float salida, float theta_dot) {

  float error = referencia - salida;

  float P = kp * error;
 // float D = kd * (error - error_ant) / T;
  float D = kd * theta_dot;

  float u = (P + D + angServoEq);

  error_ant = error;

  return constrain(u, 0, 180);
}
//--------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(1000);

  //------------------------Inicializacion IMU-------------------------
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(1000);

  //------------------------Servo-------------------------------------
  servo.attach(servoPin, PMIN, PMAX);
  servo.write(angServoEq);
  delay(1000);
}

float theta_ant=0;
//--------------------------------------------------------------------
void loop() {

  unsigned long tiempoActual = millis();

  if ((tiempoActual - tiempoMuestra) >= periodoMuestreo)
  {
    float dt = (tiempoActual - tiempoMuestra) / 1000.0;
    if (dt <= 0) dt = 0.02;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //----------------------------Theta IMU----------------------------
    float gyrox_rad_s = g.gyro.x;  // rad/s

    float anguloRotG = anguloRotAnt + gyrox_rad_s * dt;
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z);
    float anguloRot  = (1 - alpha) * anguloRotG + alpha * anguloRotA;

    float theta = -(anguloRot * 180.0 / PI - theta0);
    float theta_dot = (theta - theta_ant)/T;
    anguloRotAnt = anguloRot;

    //----------------------------Phi Pot------------------------------
    int GIRO = analogRead(potPin);
    float phi = map(GIRO, 0, 1023, -90, 270) - phi0;

    //----------------------------Control PD---------------------------

    float u = controlPD(0,theta, theta_dot);

    // *** Nueva derivada usando giroscopio ***



    u = constrain(u, 0, 180);
    servo.write((int)u);
    theta_ant = theta;

    Serial.println(theta);
    Serial.println(u);
    //----------------------------Enviar a MATLAB----------------------
    //matlab_send(theta, phi, u);

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

// --------------------- Control PD real ----------------------------
// *** La dejo EXACTAMENTE como estaba, sin tocar ***

