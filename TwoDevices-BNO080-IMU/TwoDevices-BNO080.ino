#include "soc/rtc_wdt.h"
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

#define sizedata 4
#define num_sensors 2
#define ms_send 10

//portas SDA e SCL
#define I2C_SDA 4
#define I2C_SCL 5

TwoWire I2CIMUs = TwoWire(0); //cria objeto do tipo TwoWire

#include "Biblioteca_IMU_quat.h"

// Declaracion de las IMUs
IMU  IMUs[num_sensors];

long _t[2] = {0}; //variavel para calculo de tempo
double *euler_ang; //variavel que recebe o retorno da funcao de transformacao de angulos
double Cw, Cnorm, Cxyz[3]; //variaveis auxiliares para calculo dos angulos

double **IMU_quatA, **IMU_quatB; //variaveis que recebem os valores dos quaternions retornados da funcao .get_data

double *calc_angle(double *quatA, double *quatB); //header da funcao de transformacao de angulos

void setup() {
  Serial.begin(1000000);
  Serial.println();

  I2CIMUs.begin(I2C_SDA, I2C_SCL); //inicializa o objeto com as portas definidas de SDA e SCL
  I2CIMUs.setClock(400000);
  
  //define endereco das IMUs
  IMUs[0].begin_loop(0x4B, &I2CIMUs);
  IMUs[1].begin_loop(0x4A, &I2CIMUs);

  //calculo de tempo de execucao
  _t[0] = micros();
  _t[1] = _t[0] + ms_send * 1000;
}

void loop() {

  //faz a leitura dos quaternions das IMUs
  for (int i = 0 ; i < num_sensors ; i++) IMUs[i].get_data();
  IMU_quatA = IMUs[0].reciev_data();
  IMU_quatB = IMUs[1].reciev_data();

  //chama a funcao de calculo dos angulos de euler
  euler_ang = calc_angle(*IMU_quatA, *IMU_quatB);

  //escreve no serial os angulos de euler a uma velocidade definida
  if (_t[1] <= micros() ) {
    _t[1] += ms_send * 1000;

    Serial.print((euler_ang[0] * 180) / 3.141592);
    Serial.print('\t');
    Serial.print((euler_ang[1] * 180) / 3.141592);
    Serial.print('\t');
    Serial.print((euler_ang[2] * 180) / 3.141592);
    Serial.print('\n');
  }
  return;
}

double * calc_angle(double *p, double *q) {
  Cw = ((q[3] * p[3]) + (q[0] * p[0]) + (q[1] * p[1]) + (q[2] * p[2])); //parte real quaternion
  //if (Cw < 0 ) Cw = Cw * (-1);
  Cxyz[0] = ((-q[3] * p[0]) + (q[0] * p[3]) + (-q[1] * p[2]) + (q[2] * p[1]));
  Cxyz[1] = ((-q[3] * p[1]) + (q[1] * p[3]) + (q[0] * p[2]) + (-q[2] * p[0]));
  Cxyz[2] = ((-q[3] * p[2]) + (q[2] * p[3]) + (-q[0] * p[1]) + (q[1] * p[0]));

  Cnorm = sqrt((Cxyz[0] * Cxyz[0]) + (Cxyz[1] * Cxyz[1]) + (Cxyz[2] * Cxyz[2])); //calculo da norma do vetor

  //for (int i = 0; i < 3; i++) Cxyz[i] = (Cxyz[i] / Cnorm); //normaliza vetor

  //theta = 2 * atan2(Cnorm, Cw); //calculo do angulo quaterninons

  //euler angles
  static double euler_angles[3];
  euler_angles[0] = atan2(2 * (Cw * Cxyz[0] + Cxyz[1] * Cxyz[2]), 1 - (2 * ((Cxyz[0] * Cxyz[0]) + (Cxyz[1] * Cxyz[1])))); //theta
  euler_angles[1] = asin(2 * ((Cw * Cxyz[1]) - (Cxyz[2] * Cxyz[0]))); //phi
  euler_angles[2] = atan2(2 * (Cw * Cxyz[2] + Cxyz[0] * Cxyz[1]), 1 - (2 * ((Cxyz[1] * Cxyz[1]) + (Cxyz[2] * Cxyz[2])))); //psi
  
  return euler_angles;

}
