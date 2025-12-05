// ejemplo configuracion de la IMU
//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//-------------------------Variables---------------------------------
const int servoPin = 9;
const int potPin = A0;
const int periodo = 20;             //20ms de periodo del servo
unsigned long tiempoMuestra=0;

Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos


const float angServoEq = 90;
const float angRef = 13.93;



const float theta0 = 13.92;// theta del pendulo en equilibrio lo que muesta la IMU
float theta   = 0;//theta actual
float theta_1 = 0;//theta -1
float u = 0;//u actual
float u_1=0; //u-1 anterior
float u_print=0; //para enviar a imprimir

float theta_ref = 0;//entrada de referencia del sistema es un regulador







const float T = 0.02;
const float Ts = 0.02;



//--------------------------------------------------------------------

void setup() {

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

  // set accelerometer range to +-8G  probar con diversas alternativas
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s  probar con diversas alternativas
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 5-10-21-44-94-184-260 Hz   probar con diversas alternativas
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(1000);


}

void loop() {
    sensors_event_t a, g, temp;

    unsigned long tiempoActual = millis();

    if((tiempoActual - tiempoMuestra >= periodoMuestreo))
    {

      
      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x;
      float anguloRotG = anguloRotAnt + gyrox*periodoMuestreo/1000;
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;
      
      float anguloMedido = anguloRot*180/PI;
      float theta = (anguloRot*180/PI-theta0);// theta0 el error sistematico de la IMU

      anguloRotAnt = anguloRot;
      delay(15);




matlab_send(theta, anguloRotG,anguloRotA);
  
    
      tiempoMuestra = tiempoActual;
    }

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
