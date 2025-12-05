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
float kp = 0.6;
float kd = 0;
float T  = 0.02;

float error_ant = 0;

// ---- Matriz A discreta (4x4) ----
float Ad[4][4] = {
    {0.9606, 0.1429, -0.0000, 0.0014},
    {-0.1429, 0.9606, -0.0011, 0.0192},
    {0, 0, 0.9315, 0.1744},
    {0, 0, -0.1744, 0.9315}
};

// ---- Matriz B discreta (4x1) ----
float Bd[4][1] = {
    {0.0000},
    {0.0002},
    {0.0018},
    {0.0194}
};

// ---- Matriz C (2x4) ----  y = [phi; theta]
float Cd[1][4] = {
    //{1, 0, 0, 0},   // phi
    {-0.1261, 0.6350, 0, 0}    // theta
};

// ---- Ganancia de observador (Luenberger) ----
float Ld[4][1] = {
   {-0.2341*1.0e3},
   {-0.0409*1.0e3},
   {-0.5517*1.0e3},
   {-1.4750*1.0e3}
};

// ---- Ganancia del control por estados ----
// u = -K * xhat
float K[4] = {
    910.0900,
    -403.3933,  
    134.4443,   
    25.2939
};

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

//--------------------------------------------------------------------
void loop() {

  unsigned long tiempoActual = millis();

  if ((tiempoActual - tiempoMuestra) >= periodoMuestreo)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //----------------------------Theta IMU----------------------------
    float gyrox = g.gyro.x;
    float anguloRotG = anguloRotAnt + gyrox * (periodoMuestreo / 1000.0);
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z);
    float anguloRot = (1 - alpha) * anguloRotG + alpha * anguloRotA;

    float theta = -(anguloRot * 180.0 / PI - theta0);  
    anguloRotAnt = anguloRot;

    //----------------------------Phi Pot------------------------------
    int GIRO = analogRead(potPin);
    float phi = map(GIRO, 0, 1023, -90, 270) - phi0;

    //----------------------------Control PD---------------------------
    float u = controlPD(angRef, theta);
    servo.write(u);

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

// --------------------- Control PD real ----------------------------
float controlPD(float referencia, float salida) {

  float error = referencia - salida;

  float P = kp * error;
  float D = kd * (error - error_ant) / T;

  float u = (P + D + angServoEq);

  error_ant = error;

  return constrain(u, 0, 180);
}

//
