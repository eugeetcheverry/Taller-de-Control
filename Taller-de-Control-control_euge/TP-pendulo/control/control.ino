// control clasico

//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//-------------------------Variables---------------------------------
const int servoPin = 9;
const int potPin = A0;
//
//int potMin = 1023;
//int potMax = 0;

//int potMin = 137;
//int potMax = 900;
int potMin = 136;
int potMax = 843;

const int rangos[] = {0,180};
const int periodo = 20;             //20ms de periodo del servo
const int minAnchoPulso = 1210;     //minimo ancho de pulso (0 grados)
const int maxAnchoPulso = 5200;     //maximo ancho de pulso (180 grados)

Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos
unsigned long tiempoMuestra = 0;

//int angulos[] = {120,110,90,70,100};
int i = 0;

const float angServoEq = 90;
const float angRef = 13.93;

float ang_error_act = 0;
float ang_error_ant = 0;
float ang_error_ant2 = 0;

float ang_error_acum = 0;

float accion_actual = 0;
float accion_ant   = 0;

//float angMedido_ant = 0;

const float k_p = 1.1;
const float k_i = 0.35;
const float k_d = 0.005;

const float T = 0.02;


//--------------------------------------------------------------------

void setup() {

  //--------------------Configuracion del Servo-------------------------------------
  pinMode(servoPin, OUTPUT);
  //Configuro el timer1 en modo Fast PWM con frec = 50Hz y Duty Cycle = 10%
  TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Fast PWM, clear OC1A on compare match, set OC1A at BOTTOM
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // Fast PWM, prescaler 8
  ICR1 = 39999;  // Frecuencia de PWM de 50 Hz (20 ms)
  delay(10000);
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
  //calibrar();
  //delay(500);
  //Serial.println("Valores despues de Calibrar");
  //Serial.println(potMin);
  //Serial.println(potMax);
//delay(10000);
 
  //---------------Seteo en 90° la posicion del brazo--------------------
  //Serial.println("muevoelservo");
  //moverServo(30);
    //delay(10000);
    moverServo(90);
    //delay(10000);
    //moverServo(150);
    delay(5000);

}

void loop() {
    sensors_event_t a, g, temp;

    unsigned long tiempoActual = millis();

    if(tiempoActual - tiempoMuestra >= periodoMuestreo)
    {

      int potValor = analogRead(potPin);
      float angulo = map(potValor, potMin, potMax, 0, 180);
//    moverServo(60);
    
//    matlab_send(angulo);

      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x;
      float anguloRotG = anguloRotAnt + gyrox*periodoMuestreo/1000;
      //float anguloRotA = atan2(a.acceleration.y,sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)));
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;
      
      float anguloMedido = anguloRot*180/PI;

      anguloRotAnt = anguloRot;

//      controlP(anguloMedido);
//      controlPD_T(anguloMedido, &ang_error_ant, &ang_error_acum);
//      controlPD_BD(anguloMedido, &ang_error_ant);
//controlPI_T(anguloMedido, &ang_error_ant, &ang_error_acum);
        //controlPI_tustin(anguloMedido, &ang_error_ant, &accion_ant);
        controlPID_euler(anguloMedido, &ang_error_ant, &accion_ant, &ang_error_ant2);
//ang_error_acum = ang_error_ant - anguloMedido + angRef;
//Serial.print(ang_error_acum);
//Serial.print(" ,  ");
Serial.print((anguloMedido - angRef));
Serial.print(" ,  ");
Serial.print(ang_error_ant);
Serial.print(" ,  ");
Serial.print(ang_error_ant2);
Serial.println(" ,  ");
//Serial.println(accion_ant);

//   
      //matlab_send(angulo, (anguloMedido-angRef),accion_ant);
  
    
      tiempoMuestra = tiempoActual;
    }

}

//-------------------Funciones Control-------------------------------
void controlP(float anguloMedido)
{
  ang_error_act = angRef - anguloMedido;
  float accionP = ang_error_act * k_p;
  moverServo(accionP + angServoEq);
}

void controlPI_BD(float anguloMedido, float* ang_error_acum)
{
  ang_error_act = angRef - anguloMedido;
  float accionPI = (k_p + k_i*T)*ang_error_act + k_i*(*ang_error_acum);
  *ang_error_acum += ang_error_act;
  moverServo(accionPI + angServoEq);
}

void controlPD_BD(float anguloMedido, float* ang_error_ant)
{
  ang_error_act = angRef - anguloMedido;
  float accionPD = k_p*ang_error_act + k_d*(ang_error_act - *ang_error_ant)/T;
  *ang_error_ant = ang_error_act;
  moverServo(accionPD + angServoEq);
}


void controlPI_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum)
{
  ang_error_act = angRef - anguloMedido;
  float I = (*ang_error_acum) + T/2 * ang_error_act + T/2 * (*ang_error_ant);
  float accionPI = k_p*ang_error_act + k_i*I;
  *ang_error_ant = ang_error_act;
  *ang_error_acum = I;
  moverServo(accionPI + angServoEq);
}

void controlPI_tustin(float anguloMedido, float* ang_error_ant, float* accion_ant)
{
  ang_error_act = -1 *(angRef - anguloMedido);
  float accion_actual = (*accion_ant) +  ang_error_act *(k_p+k_i*T/2)+ (*ang_error_ant)*(k_i*T/2-k_p);
  //float accionPI = k_p*ang_error_act + k_i*I;
  *ang_error_ant = ang_error_act;
  *accion_ant = accion_actual;
  moverServo(accion_actual + angServoEq);
  
}

void controlPID_euler(float anguloMedido, float* ang_error_ant, float* accion_ant, float* ang_error_ant2)
{
  ang_error_act = -1 *(angRef - anguloMedido);
  
  float accion_actual = (*accion_ant) +  ang_error_act *(k_p+k_i*T+k_d/T) - (*ang_error_ant)*(1+2*k_d/T) + (*ang_error_ant2)*k_d/T ;
  *ang_error_ant2 = *ang_error_ant;
  *ang_error_ant = ang_error_act;
  *accion_ant = accion_actual;
  moverServo(accion_actual + angServoEq);
  
}



void controlPD_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum)
{
  ang_error_act = angRef - anguloMedido;
  float D = 2*(ang_error_act - *ang_error_ant)/T - *ang_error_acum;
  float accionPD = k_p*ang_error_act + k_d*D;
  *ang_error_ant = ang_error_act;
  *ang_error_acum = D;
  moverServo(accionPD + angServoEq);
}

//-------------------Funcion Calibracion-----------------------------
void calibrar()
{    
    Serial.println("Valores antes de Calibrar");
    Serial.println(potMin);
    Serial.println(potMax);
//    for(int pos = 0; pos < 2; pos++)
//    {
//      unsigned long tiempoActual = millis();
//
//      if(tiempoActual - tiempoMuestra >= periodoMuestreo)
//      {
//        int potValor = analogRead(potPin);
//        moverServo(rangos[pos]);
//      
//        if(potValor > potMax)
//        {
//          potMax = potValor;
//        }
//
//        if(potValor < potMin)
//        {
//          potMin = potValor;
//        }
//        tiempoMuestra = tiempoActual;
//      }
      moverServo(0);
      delay(1000);
      int potValor = analogRead(potPin);
      if(potValor > potMax)
      {
        potMax = potValor;
      }

      if(potValor < potMin)
      {
        potMin = potValor;
      }

      delay(500);
      moverServo(180);
      delay(1000);
      potValor = analogRead(potPin);
      
      if(potValor > potMax)
      {
        potMax = potValor;
      }

      if(potValor < potMin)
      {
        potMin = potValor;
      }
      delay(1000);
//    }
}

//----------------Funciones para movimiento del Servo----------------
void moverServo(int angulo)
{
  int anchoPulso = map(angulo, 0, 180, minAnchoPulso, maxAnchoPulso);
  OCR1A = anchoPulso;
}

void moverServoPWM(float tiempo)
{
  OCR1A = ((tiempo*1000)/(periodo*1000))*(40000) - 1;
}

//-----------------------Envio de datos a MATLAB-------------------------
void matlab_send(float dato1, float dato2, float dato3){
  //Serial.write("abcd");
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);
  
  byte * b0 = (byte *) &dato1;
  Serial.write(b0,4);
  
  byte * b1 = (byte *) &dato2;
  Serial.write(b1,4);
//  
byte * b2 = (byte *) &dato3;
Serial.write(b2,4);
}
