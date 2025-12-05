// ==========================================================
//  CONTROL POR ESTADOS + OBSERVADOR DE LUENBERGER (4 estados)
//  PÉNDULO rotativo FIUBA - Arduino UNO
// ==========================================================

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Servo servoMotor;

// Variables de medición
float phi_meas = 0;      // Ángulo brazo (potenciómetro)
float theta_meas = 0;    // Ángulo péndulo (IMU)

// Estados estimados: x_hat = [phi, dphi, theta, dtheta]
float xhat[4] = {0, 0, 0, 0};

// Periodo de muestreo fijo
const float Ts = 0.01;   // 100 Hz

// ==========================================================
//     COMPLETAR ESTAS MATRICES CON TUS VALORES
// ==========================================================

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

// ==========================================================
// LECTURA DE SENSORES
// ==========================================================

// ----------------------------
//  Potenciómetro → φ (grados)
// ----------------------------
float leerPhi() {
    int val = analogRead(A0); // O el pin que uses
    // AJUSTAR este map según tu calibración:
    float phi = map(val, 0, 1023, -45, 45); 
    return phi;
}

// ----------------------------
//  IMU (MPU6050) → θ (grados)
// ----------------------------
float leerTheta() {
    mpu.getEvent(&a, &g, &temp);
    // Elegir el eje correcto según montaje
    float theta = mpu.getAngleX(); 
    return theta;
}

// ==========================================================
//  OBSERVADOR DE LUENBERGER
// ==========================================================

void actualizarObservador(float u)
{
    // Error de salida: y - C*xhat
    float y1 = phi_meas  - (Cd[0][0]*xhat[0] + Cd[0][1]*xhat[1] + Cd[0][2]*xhat[2] + Cd[0][3]*xhat[3]);
    float y2 = theta_meas - (Cd[1][0]*xhat[0] + Cd[1][1]*xhat[1] + Cd[1][2]*xhat[2] + Cd[1][3]*xhat[3]);

    float xnext[4];

    for(int i=0;i<4;i++){
        // Ad * xhat
        float Ax = 0;
        for(int j=0;j<4;j++){
            Ax += Ad[i][j] * xhat[j];
        }
        // Bd * u
        float Bu = Bd[i][0] * u;
        
        // Ld * (y - Cx)
        float Ly = Ld[i][0]*y1 + Ld[i][1]*y2;

        xnext[i] = Ax + Bu + Ly;
    }

    for(int i=0;i<4;i++){
        xhat[i] = xnext[i];
    }
}

// ==========================================================
//  CONTROLADOR: u = -K * xhat
// ==========================================================
float calcularControl(){
    float u = -(K[0]*xhat[0] + K[1]*xhat[1] + K[2]*xhat[2] + K[3]*xhat[3]);
    
    // Saturación para el servo
    if(u > 30) u = 30;
    if(u < -30) u = -30;

    return u;
}

// ==========================================================
//  SETUP
// ==========================================================

void setup() {
    Serial.begin(115200);

    Wire.begin();
    mpu.begin();
    mpu.calcOffsets(); // Calibración básica IMU

    servoMotor.attach(9);  // Pin PWM
}

// ==========================================================
//  LOOP PRINCIPAL (100 Hz)
// ==========================================================

void loop() {

    // -----------------------------
    // Paso 1: leer sensores reales
    // -----------------------------
    phi_meas = leerPhi(); 
    theta_meas = leerTheta();

    // -----------------------------
    // Paso 2: calcular u
    // -----------------------------
    float u = calcularControl();

    // -----------------------------
    // Paso 3: actualizar observador
    // -----------------------------
    actualizarObservador(u);

    // -----------------------------
    // Paso 4: mover servo
    // -----------------------------
    servoMotor.write(90 + u);   // 90 = medio, u hace desviación

    // -----------------------------
    // Paso 5: enviar datos a MATLAB
    // -----------------------------
    Serial.print(phi_meas); Serial.print(",");
    Serial.print(theta_meas); Serial.print(",");
    Serial.print(xhat[0]);   Serial.print(",");
    Serial.print(xhat[1]);   Serial.print(",");
    Serial.print(xhat[2]);   Serial.print(",");
    Serial.println(xhat[3]);

    delay(10); // 100 Hz
}
