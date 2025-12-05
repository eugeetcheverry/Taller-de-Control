//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------
const int servoPin = 3;
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
const float theta_meas = 0.543104171752930;   // offset IMU
const float phi_meas   = -5;

// Control PD
float kp = 0.6;
float kd = 0;
float T  = 0.02;

float error_ant = 0;

// ---- Matriz A discreta (4x4) ----
float Ad_1[4] = {-6.0871,    8.5942,    0.1944,   -9.7902};
float Ad_2[4] = {-1.1695,    1.4915,    0.0430,   -1.9488};
float Ad_3[4] = {2.0851,   -3.9519,    0.8983,    2.5447};
float Ad_4[4] = {2.4739,   -1.4570,   -0.2675,    4.9021};

// ---- Matriz B discreta (4x1) ----
float Bd_1[4] = {-0.2045,   -4.3036}; //matriz del observador Bodd
float Bd_2[4] = {-0.0424,   -0.3776};
float Bd_3[4] = {0.0505,    1.7899};
float Bd_4[4] = {0.1575,    0.9732};

// ---- Matriz C (2x4) ----  y = [phi; theta]
float Cd[4] = {-0.5043,    2.5401,         0,         0};

// ---- Ganancia de observador (Luenberger) ----
// Ld[row_estado][col_salida], columnas: 0->phi, 1->theta
float Ld[4] = {-17.8565, -2.5295, 5.1922, 6.2587};

// ---- Ganancia del control por estados ----
// u = -K * xhat
float K[4] = {-18.3382, 7.3372, 37.7365, 6.5830};

// INIT theta_ant
float theta_ant = 0;
//Init estados estimados
float x_est[4] = {0, 0, 0, 0};
float x_hat[4] = {0, 0, 0, 0};
//Init u_ant y u
float u_ant = 0;
float u = 0; 

float Lterm[4] = {0, 0, 0, 0};
float x_next[4] = {0, 0, 0, 0};
//float x_hat[4] = {0, 0, 0, 0};
float x_hat_ant[4] = {0, 0, 0, 0};
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
  servo.write(90);
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
    //----------------------------Phi Pot------------------------------
    int GIRO = analogRead(potPin);
    float phi = map(GIRO, 0, 1023, -90, 270) - phi0;
    //----------------------------Observador---------------------------
    x_est[0] = theta;                 // y1
    x_est[1] = (theta - theta_ant) / T;;               // y2
    x_est[2] = u;
    x_est[3] = (u - u_ant)/T;

    actualizarObservador(u, x_hat, x_est); //Actualizo x_hat
    //---------------------------Calculo de accion de control----------
    u = calcularControl(x_hat, K);
    servo.write(u);
    //----------------------------Enviar a MATLAB----------------------
    matlab_send(theta, phi, u, x_est[0], x_est[1], x_est[2], x_est[3], x_hat[0], x_hat[1], x_hat[2], x_hat[3]);
    theta_ant = theta;
    u_ant = u;
    //for(int i=0;i<4;i++){
    //    x_hat_ant[i] = x_hat[i];
    //}
    tiempoMuestra = tiempoActual;
  }
}

//-----------------------Envio de datos a MATLAB---------------------
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8, float dato9, float dato10, float dato11){
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);
  //Serial.write(101);

  Serial.write((byte*) &dato1, 4);
  Serial.write((byte*) &dato2, 4);
  Serial.write((byte*) &dato3, 4);
  Serial.write((byte*) &dato4, 4);
  Serial.write((byte*) &dato5, 4);
  Serial.write((byte*) &dato6, 4);
  Serial.write((byte*) &dato7, 4);
  Serial.write((byte*) &dato8, 4);
  Serial.write((byte*) &dato9, 4);
  Serial.write((byte*) &dato10, 4);
  Serial.write((byte*) &dato11, 4);
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

// -------------------Control Observador---------------------------
void actualizarObservador(float u, float* x_hat, float* x_est)
{
    // Error de salida: y - C*xhat
    // Error de salida: y - C*xhat
    float y_hat_phi = Cd[0]*x_hat[0] + Cd[1]*x_hat[1] + Cd[2]*x_hat[2] + Cd[3]*x_hat[3];

    float y_hat_theta = Cd[0]*x_hat[0] + Cd[1]*x_hat[1] + Cd[2]*x_hat[2] + Cd[3]*x_hat[3];

    float x_next[4];

    // A*x_hat + B*u + Lterm
    x_next[0] = Ad_1[0]*x_hat[0] + Ad_1[1]*x_hat[1] + Ad_1[2]*x_hat[2] + Ad_1[3]*x_hat[3] + Bd_1[0]*u;
    x_next[1] = Ad_2[0]*x_hat[0] + Ad_2[1]*x_hat[1] + Ad_2[2]*x_hat[2] + Ad_2[3]*x_hat[3] + Bd_1[0]*u;
    x_next[2] = Ad_3[0]*x_hat[0] + Ad_3[1]*x_hat[1] + Ad_3[2]*x_hat[2] + Ad_3[3]*x_hat[3] + Bd_2[0]*u; 
    x_next[3] = Ad_4[0]*x_hat[0] + Ad_4[1]*x_hat[1] + Ad_4[2]*x_hat[2] + Ad_4[3]*x_hat[3] + Bd_2[0]*u;

    //Sumo L
    float phi_error   = x_est[2]   - y_hat_phi;
    float theta_error = x_est[0] - y_hat_theta;
    Lterm[0] = Ld[0]*phi_error + Ld[0]*theta_error;
    Lterm[1] = Ld[1]*phi_error + Ld[1]*theta_error;
    Lterm[2] = Ld[2]*phi_error + Ld[2]*theta_error;
    Lterm[3] = Ld[3]*phi_error + Ld[3]*theta_error;
    
    for(int i=0;i<4;i++){
        x_hat[i] = x_next[i] + Lterm[i];
    }
}
float calcularControl(float x_hat[4], float K[4])
{
    // Control por realimentación de estados
    // u_action = -K * xhat
    float action = (K[0]*x_hat[0] + 
                     K[1]*x_hat[1] +
                     K[2]*x_hat[2] +
                     K[3]*x_hat[3]);

    // limitar acción
    if (action > 30) action = 30;
    if (action < -30) action = -30;

    //devolver ángulo absoluto para servo: equilibrio + acción
    return action;
}


