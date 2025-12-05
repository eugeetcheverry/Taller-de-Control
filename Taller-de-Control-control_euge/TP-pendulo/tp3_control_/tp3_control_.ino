// control observador

//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------
const int pinServo = 9;
const int potPin = A0;

int potMin = 136;
int potMax = 843;

const int rangos[] = {0,180};
const int periodo = 20;             //20ms de periodo del servo
const int minAnchoPulso = 1210;     //minimo ancho de pulso (0 grados)
const int maxAnchoPulso = 5200;     //maximo ancho de pulso (180 grados)

Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

Servo servo;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos
unsigned long tiempoMuestra = 0;

int i = 0;

const float angServoEq = 90;
const float angRef = 0;

float ang_error_ant2 = 0;
float ang_error_ant = 0;
float ang_error_act = 0;


const float theta0 = 0.543104171752930;   // offset IMU
float theta   = 0;//theta actual
float theta_1 = 0;//theta -1
float u = 0;//u actual
float u_1=0; //u-1 anterior
float u_print=0; //para enviar a imprimir
float theta_e  = 0;//theta estimada
float theta_ref = 15;//entrada de referencia del sistema es un regulador

float xe[4]={0  , 0 , 0  , 0};// vector de estados estimados Xe
float xe_1[4]={0  , 0 , 0  , 0};// vector de estados estimados Xe-1 anterior
float K[4]={-18.3382,    7.3372,   37.7365,    6.5830};// vector K del controller -5+i*10 ;p2c=-5-i*10; p3c=-10+i*30 ; p4c=-10-i*30;
float L[4]={-17.8565, -2.5295, 5.1922, 6.2587};   // vector L del observer

float A_1[4]={-6.0871,    8.5942,    0.1944,   -9.7902};// matriz A del observador Aodd
float A_2[4]={-1.1695,    1.4915,    0.0430,   -1.9488};
float A_3[4]={2.0851,   -3.9519,    0.8983,    2.5447};
float A_4[4]={2.4739,   -1.4570,   -0.2675,    4.9021};

float B_1[2]= {-0.2045,   -4.3036}; //matriz del observador Bodd
float B_2[2]= {-0.0424,   -0.3776};
float B_3[2]= {0.0505,    1.7899};
float B_4[2]= {0.1575,    0.9732};

float B_d[4]={-0.0001, -0.0050, 0.0063, 0.0690};
 
float Cd[4]={-0.5043,    2.5401,         0,         0};

float ang_error_acum = 0;

float accion_actual = 0;
float accion_ant   = 0;

const float k_p = 1.1;
const float k_i = 0.35;
const float k_d = 0.005;

float Xi = 0;
const float Ts = 0.02;

float theta_aux = 0;
int contador = 0;
int estado = 0;

//--------------------------------------------------------------------

void setup() {

  //--------------------Configuracion del Servo-------------------------------------
  pinMode(pinServo, OUTPUT);
  //Configuro el timer1 en modo Fast PWM con frec = 50Hz y Duty Cycle = 10%
  TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Fast PWM, clear OC1A on compare match, set OC1A at BOTTOM
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // Fast PWM, prescaler 8
  ICR1 = 39999;  // Frecuencia de PWM de 50 Hz (20 ms)
  delay(5000);
  Serial.begin(115200);
  delay(1000);
  //------------------------Inicializacion de la IMU--------------------------------
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(1000);
 
//  //-------------------------Calibracion---------------------------------
  servo.attach(pinServo, 500, 2500);

  delay(1000);
  servo.write(90);
  delay(3000);
  servo.write(120);
  delay(500);
}

void loop() {
    sensors_event_t a, g, temp;

    unsigned long tiempoActual = millis();
    
    if((tiempoActual - tiempoMuestra >= periodoMuestreo))
    {

      int potValor = analogRead(potPin);
      float angulo = map(potValor, potMin, potMax, 0, 180);
      float phi = map(potValor, potMin, potMax, 0, 180);

      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x;
      float anguloRotG = anguloRotAnt + gyrox*periodoMuestreo/1000;
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;


      float anguloMedido = anguloRot*180/PI;
      float theta = (anguloRot*180/PI-theta0);// theta0 el error sistematico de la IMU
      
      anguloRotAnt = anguloRot;
   
      float theta_dot = gyrox * 180 / PI; // en grados por segundo
      float error = theta_ref - theta;
      Xi += Ts * error;
      
      u = control_observador(theta, &theta_e, &u_1 ,&u_print, Xi);
      servo.write(u);
      \
      ang_error_acum = ang_error_ant - anguloMedido + angRef;
      tiempoMuestra = tiempoActual;
      matlab_send(u, theta, u, )
    }
}

//-------------------Funciones Control-------------------------------

float control_observador(float theta,  float* theta_e, float* u_1, float* u_print,float Xi)
{
  // ang_error_act = 1 * (theta_ref - theta);
 // theta = -ang_error_act;
  xe[0]= (A_1[0]*xe_1[0]+ A_1[1]*xe_1[1]+A_1[2]*xe_1[2]+A_1[3]*xe_1[3]) + B_1[0]* (*u_1) + B_1[1]*theta ;
  xe[1]= (A_2[0]*xe_1[0]+ A_2[1]*xe_1[1]+A_2[2]*xe_1[2]+A_2[3]*xe_1[3]) + B_2[0]* (*u_1) + B_2[1]*theta ;
  xe[2]= (A_3[0]*xe_1[0]+ A_3[1]*xe_1[1]+A_3[2]*xe_1[2]+A_3[3]*xe_1[3]) + B_3[0]* (*u_1) + B_3[1]*theta ;
  xe[3]= (A_4[0]*xe_1[0]+ A_4[1]*xe_1[1]+A_4[2]*xe_1[2]+A_4[3]*xe_1[3]) + B_4[0]* (*u_1) + B_4[1]*theta ;
  
  float u = -1*( K[0]*xe[0]+K[1]*xe[1]+K[2]*xe[2]+K[3]*xe[3]+0.6*Xi);
  float theta_est = Cd[0]*xe[0]+Cd[1]*xe[1]+Cd[2]*xe[2]+Cd[3]*xe[3];
    
    xe_1[0]= xe[0] ;
    xe_1[1]= xe[1] ;
    xe_1[2]= xe[2] ;
    xe_1[3]= xe[3] ;
      
    *u_1=u ;
    *theta_e=theta_est++;
    *u_print=u ;
    u = -u;
    Serial.print("Xi: "); Serial.println(Xi);
    u = constrain(u + 90, 0, 180);
    return u;
}

//-----------------------Envio de datos a MATLAB---------------------
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7){
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
}

