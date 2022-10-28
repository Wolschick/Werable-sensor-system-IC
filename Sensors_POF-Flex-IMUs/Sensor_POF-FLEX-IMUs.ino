#include "soc/rtc_wdt.h"
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#include <Adafruit_SleepyDog.h>
#include <esp_now.h>
#include <WiFi.h>
//#include <BasicLinearAlgebra.h>

//IMUs****************************

#define sizedata 4
#define num_sensors 2
#define ms_send 15

//portas SDA e SCL
#define I2C_SDA_IMU 4
#define I2C_SCL_IMU 5
#define I2C_SDA_FLEX 21
#define I2C_SCL_FLEX 22

//endereco das IMUs
#define IMU1_ADD 0x4B
#define IMU2_ADD 0x4A

//define o tipo de valores que sera feito o TARE
const uint8_t quat_report = 0x05;

TwoWire I2CIMUs = TwoWire(0); //cria objeto do tipo TwoWire
TwoWire I2CFLEX = TwoWire(1); //cria objeto do tipo TwoWire

#include "Biblioteca_IMU_quat.h"

// Declaracion de las IMUs
IMU  IMUs[num_sensors];

long _t[2] = {0}; //variavel para calculo de tempo
double *euler_ang, euler_angles_degree[3]; //variavel que recebe o retorno da funcao de transformacao de angulos
double Cw, Cnorm, Cxyz[3]; //variaveis auxiliares para calculo dos angulos

double **IMU_quatA, **IMU_quatB; //variaveis que recebem os valores dos quaternions retornados da funcao .get_data

double *offset_values; //variavel com os valores dos offsets

//HEADERS ***********************
double * calc_angle(double *quatA, double *quatB); //header da funcao de transformacao de angulos
double * offset_func(float Flex1_X, float Flex1_Y, float POF_value01, float Flex2_X, float Flex2_Y, float POF_value02, double euler_angles_degree);

//POF and Flex sensors******************************

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x6E, 0x97, 0x0C};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

ADS myFlexSensor1; //Create object of the ADS class
ADS myFlexSensor2; //Create object of the ADS class

//pinos de flag dos sensores flexiveis
const byte dataReadyPin_1 = 2;
const byte dataReadyPin_2 = 15;

//Variaveis auxiliares
float Flex1_X;
float Flex1_Y;
float Flex2_X;
float Flex2_Y;
float POF_value01;
float POF_value02;
int fail_data1;
int fail_data2;
int offset_cont = 0;

//pinos das POFs
const byte POF_01 = 39;
const byte POF_02 = 36;

//variavel auxiliar para o wathdog timer
int countdownMS;

//comunicacao ESPNOW
typedef struct struct_message {
  float NOW_S1_Xflex;
  float NOW_S1_Yflex;
  float NOW_S1_POF;
  float NOW_S2_Xflex;
  float NOW_S2_Yflex;
  float NOW_S2_POF;
  float NOW_IMU_Tetha;
  float NOW_IMU_Phi;
  float NOW_IMU_Psi;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(1000000);
  Serial.println();

  countdownMS = Watchdog.enable(3000); //watchdog timer

  //pinos de flag do sensor flexivel
  pinMode(dataReadyPin_1, INPUT);
  pinMode(dataReadyPin_2, INPUT);

  //pinos das POFs
  pinMode(POF_01, INPUT);
  pinMode(POF_02, INPUT);

  I2CIMUs.begin(I2C_SDA_IMU, I2C_SCL_IMU); //inicializa o objeto com as portas definidas de SDA e SCL
  I2CIMUs.setClock(100000);
  I2CFLEX.begin(I2C_SDA_FLEX, I2C_SCL_FLEX); //inicializa o objeto com as portas definidas de SDA e SCL
  I2CFLEX.setClock(100000);

  //define endereco das IMUs
  IMUs[0].begin_loop(IMU1_ADD, &I2CIMUs);
  IMUs[1].begin_loop(IMU2_ADD, &I2CIMUs);
  delay(50);

  //aplicando TARE
  int cont = 0;
  while (cont < 3) {
    Serial.print("Setting orientation *****");
    Serial.print("\n");
    TARE(IMU1_ADD);
    delay(200);
    TARE(IMU2_ADD);
    delay(200);
    cont++;
  }
  delay(50);


  //verificacoes funcionamento sensores flexiveis ***********************
  //Setup first sensor - look for it at the default address of 0x13 = 19
  if (myFlexSensor1.begin(45, I2CFLEX) == false)
  {
    Serial.println(F("First sensor not detected. Check wiring. Freezing..."));
    while (true);
  }

  //Setup second sensor - look for it at the I2C address of 45. You should have set this up in example 6
  if (myFlexSensor2.begin(19, I2CFLEX) == false)
  {
    Serial.println(F("Second sensor not detected. Check wiring. Freezing..."));
    while (true);
  }
  delay(50);

  //configuracoes ESPNOW **********************************

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //**********************************************

  //calculo de tempo de execucao
  _t[0] = micros();
  _t[1] = _t[0] + ms_send * 1000;
}

void loop() {
  //ofset_values = Flex1_X; Flex1_Y; Flex2_X; Flex2_Y; POF_value01; POF_value02; euler_angles_degree[0]; euler_angles_degree[1]; euler_angles_degree[2];

  //faz a leitura dos quaternions das IMUs
  for (int i = 0 ; i < num_sensors ; i++) IMUs[i].get_data();
  IMU_quatA = IMUs[0].reciev_data();
  IMU_quatB = IMUs[1].reciev_data();

  //chama a funcao de calculo dos angulos de euler
  euler_ang = calc_angle(*IMU_quatA, *IMU_quatB);
  euler_angles_degree[0] = (((euler_ang[0] * 180) / 3.141592)); //- offset_values[6]);
  euler_angles_degree[1] = (((euler_ang[1] * 180) / 3.141592)); //- offset_values[7]);
  euler_angles_degree[2] = (((euler_ang[2] * 180) / 3.141592)); //- offset_values[8]);

  //verifica funcionamento Flex sensor 1
  if (myFlexSensor1.available() == true) {
    Flex1_X = (myFlexSensor1.getX());// - offset_values[0]);
    Flex1_Y = (myFlexSensor1.getY());// - offset_values[1]);
  } else {
    fail_data1 ++;
  }

  //verifica funcionamento Flex sensor 1
  if (myFlexSensor2.available() == true) {
    Flex2_X = (myFlexSensor2.getX());// - offset_values[2]);
    Flex2_Y = (myFlexSensor2.getY());// - offset_values[3]);
  } else {
    fail_data2 ++;
  }

  //reinicia a ESP caso nao haja leitura dos Flex ensors
  if (fail_data1 >= 50 || fail_data2 >= 50) {
    ESP.restart();
  }

  POF_value01 = (analogRead(POF_01));// - offset_values[4]);
  POF_value02 = (analogRead(POF_02));// - offset_values[5]);

  if (offset_cont <= 50) {
    offset_values = offset_func( Flex1_X, Flex1_Y, POF_value01, Flex2_X, Flex2_Y, POF_value02, euler_angles_degree, offset_cont);
    offset_cont += 1;
  }

  //escreve no serial os angulos de euler a uma velocidade definida
  if (_t[1] <= micros() ) {
    _t[1] += ms_send * 1000;

    //ofset_values = Flex1_X; Flex1_Y; Flex2_X; Flex2_Y; POF_value01; POF_value02; euler_angles_degree[0]; euler_angles_degree[1]; euler_angles_degree[2];
    Serial.print((euler_angles_degree[0]) - (offset_values[6]) );
    Serial.print('\t');
    Serial.print((euler_angles_degree[1]) - (offset_values[7]));
    Serial.print('\t');
    Serial.print((euler_angles_degree[2]) - (offset_values[8]));
    Serial.print('\t');
    Serial.print(Flex1_X - offset_values[0]);
    Serial.print('\t');
    Serial.print(Flex1_Y - offset_values[1]);
    Serial.print('\t');
    Serial.print(Flex2_X - offset_values[2]);
    Serial.print('\t');
    Serial.print(Flex2_Y - offset_values[3]);
    Serial.print('\t');
    Serial.print(double(POF_value01) - offset_values[4]);
    Serial.print('\t');
    Serial.print(double(POF_value02) - offset_values[5]);
    Serial.print('\n');
    /*

      for (int i = 0; i < 9; i++ ) {
      Serial.print(offset_values[i]);
      Serial.print("\t");
      }
      Serial.print("\n");*/

    //envia os dados via ESPNOW
    myData.NOW_S1_Xflex = Flex1_X;
    myData.NOW_S1_Yflex = Flex1_Y;
    myData.NOW_S1_POF = POF_value01;
    myData.NOW_S2_Xflex = Flex2_X;
    myData.NOW_S2_Yflex = Flex2_Y;
    myData.NOW_S2_POF = POF_value02;
    myData.NOW_IMU_Tetha = euler_angles_degree[0];
    myData.NOW_IMU_Phi = euler_angles_degree[1];
    myData.NOW_IMU_Psi = euler_angles_degree[2];

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  }

  countdownMS = Watchdog.enable(3000);
  delay(5);
  return;
}

double * calc_angle(double *p, double *q) {
  Cw = ((q[3] * p[3]) + (q[0] * p[0]) + (q[1] * p[1]) + (q[2] * p[2])); //parte real quaternion
  //if (Cw < 0 ) Cw = Cw * (-1);
  Cxyz[0] = ((-q[3] * p[0]) + (q[0] * p[3]) + (-q[1] * p[2]) + (q[2] * p[1]));
  Cxyz[1] = ((-q[3] * p[1]) + (q[1] * p[3]) + (q[0] * p[2]) + (-q[2] * p[0]));
  Cxyz[2] = ((-q[3] * p[2]) + (q[2] * p[3]) + (-q[0] * p[1]) + (q[1] * p[0]));

  Cnorm = sqrt((Cxyz[0] * Cxyz[0]) + (Cxyz[1] * Cxyz[1]) + (Cxyz[2] * Cxyz[2]) + (Cw * Cw)); //calculo da norma do vetor

  //normaliza vetor
  for (int i = 0; i < 3; i++) Cxyz[i] = (Cxyz[i] / Cnorm);
  Cw = Cw / Cnorm;

  //theta = 2 * atan2(Cnorm, Cw); //calculo do angulo quaterninons

  //euler angles
  static double euler_angles[3];
  euler_angles[0] = atan2(2 * (Cw * Cxyz[0] + Cxyz[1] * Cxyz[2]), 1 - (2 * ((Cxyz[0] * Cxyz[0]) + (Cxyz[1] * Cxyz[1])))); //theta
  euler_angles[1] = asin(2 * ((Cw * Cxyz[1]) - (Cxyz[2] * Cxyz[0]))); //phi
  euler_angles[2] = atan2(2 * (Cw * Cxyz[2] + Cxyz[0] * Cxyz[1]), 1 - (2 * ((Cxyz[1] * Cxyz[1]) + (Cxyz[2] * Cxyz[2])))); //psi

  return euler_angles;

}

//funcao para calculo do offset
double * offset_func(float Flex1_X, float Flex1_Y, float POF_value01, float Flex2_X, float Flex2_Y, float POF_value02, double euler_angles_degree0[], int offset_cont) {

  static double Average[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0};

  if (offset_cont <=10) return Average;

  Serial.print("Calculando o Offset...");

  

  Average[0] += double(Flex1_X);
  Average[1] += double(Flex1_Y);
  Average[2] += double(Flex2_X);
  Average[3] += double(Flex2_Y);
  Average[4] += double(POF_value01);
  Average[5] += double(POF_value02);
  Average[6] += double(euler_angles_degree[0]);
  Average[7] += euler_angles_degree[1];
  Average[8] += euler_angles_degree[2];

  if (offset_cont == 50) {
    Average[0] = (Average[0] / 40.0);
    Average[1] = (Average[1] / 40.0);
    Average[2] = (Average[2] / 40.0);
    Average[3] = (Average[3] / 40.0);
    Average[4] = (Average[4] / 40.0);
    Average[5] = (Average[5] / 40.0);
    Average[6] = (Average[6] / 40.0);
    Average[7] = (Average[7] / 40.0);
    Average[8] = (Average[8] / 40.0);
  }

  return Average;


}

void TARE(uint8_t BNO_ADDRESS) {
  static uint8_t RV;               // rotation vector used to tare defined in quat_report
  if (quat_report == 0x05) RV = 0x00;
  if (quat_report == 0x08) RV = 0x01;
  if (quat_report == 0x09) RV = 0x02;
  uint8_t tare_now[16] = {16, 0, 2, 0, 0xF2, 0, 0x03, 0, 0x07, RV, 0, 0, 0, 0, 0, 0}; //0x07 means all axes 0x04 = Z axis only; based on rotation vector
  uint8_t tare_persist[16] = {16, 0, 2, 0, 0xF2, 0, 0x03, 0x01, RV, 0, 0, 0, 0, 0, 0, 0};
  Wire.beginTransmission(BNO_ADDRESS);
  Wire.write(tare_now, sizeof(tare_now));
  //Wire.write(tare_persist, sizeof(tare_persist));                                  // uncomment  for tare persist;
  Wire.endTransmission();

  return;
}
