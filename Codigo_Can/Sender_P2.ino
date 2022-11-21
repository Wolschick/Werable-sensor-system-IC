
#define _TIPO_PLACA 2
#define _ID 12
#define CAN_SPEED 1000E3
#define SHOW_SERIAL true

#if _TIPO_PLACA == 1
#define _N_SENSORS 4
#else
#define _N_SENSORS 8
#endif

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <CAN.h>

/*
  typedef struct packet_id_t{
    uint16_t tipo;
    uint16_t id_placa;
    uint16_t n;
  }packet_id;
*/

// Adafruit_ADS1115 ads1;  /* Use this for the 16-bit version */
//Adafruit_ADS1115 ads2;     /* Use this for the 12-bit version */
Adafruit_ADS1015 ads1;     /* Use this for the 12-bit version */
Adafruit_ADS1015 ads2;

long tempo = 0;
#define TS 5000

//nomeia as portas dos sensores conectados
const int pin_fsr5 = 36;
const int pin_fsr6 = 39;
const int pin_fsr7 = 34;
const int pin_fsr8 = 35;

int bits_fsr5;
int bits_fsr6;
int bits_fsr7;
int bits_fsr8;

//variavies tipo inteiro de 16 bits que receberam o mapeamento de cada sensor
int16_t fsr5 = 0.0;
int16_t fsr6 = 0.0;
int16_t fsr7 = 0.0;
int16_t fsr8 = 0.0;

//variaveis que receberam os valores das variaveis de mapeamento normalizado
float fsr5_value = 0.0;
float fsr6_value = 0.0;
float fsr7_value = 0.0;
float fsr8_value = 0.0;

//variaveis que receberao o valor final dos sensores
float FSR_1 = 0.0;
float FSR_2 = 0.0;
float FSR_3 = 0.0;
float FSR_4 = 0.0;
float FSR_5 = 0.0;
float FSR_6 = 0.0;
float FSR_7 = 0.0;
float FSR_8 = 0.0;

//declara variavel tipo int de 16 bits que recebera as comparacoes dos ads
int16_t results1, results2, results3, results4;

//constante de multiplicacao utilizada para o calculo das variaveis "result"
float   multiplier = 3.0F;

int k;

//vetor que recebe os valores iniciais para referencia
float comp_fsr1[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr2[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr3[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr4[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr5[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr6[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr7[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_fsr8[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//variavel que fara a media dos valores de referencia
float sum_comp_fsr1 = 0.0;
float sum_comp_fsr2 = 0.0;
float sum_comp_fsr3 = 0.0;
float sum_comp_fsr4 = 0.0;
float sum_comp_fsr5 = 0.0;
float sum_comp_fsr6 = 0.0;
float sum_comp_fsr7 = 0.0;
float sum_comp_fsr8 = 0.0;


void setup(void)
{
  Serial.begin(115200);

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // Inicia o barramento CAN a 500 kbps
  Serial.println("Iniciando CAN");
  CAN.setPins(4, 5);
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("Failed to initialize CAN");
    while (1);
  }

  //verifica os ads
  Serial.println("Conectando ads1");
  if (!ads1.begin(0x48, &Wire)) {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
  Serial.println("Conectando ads2");
  if (!ads2.begin(0x49, &Wire)) {
    Serial.println("Failed to initialize ADS2.");
    while (1);
  }

}

//long tt01[2];

void loop(void)
{
 // tt01[0] = micros();
  //tempo = micros();

  //Variavel results recebe a comparacao dos sensores obtidos nos ads's
  results1 = ads1.readADC_Differential_0_1() * multiplier;
  results2 = ads1.readADC_Differential_2_3() * multiplier;
  results3 = ads2.readADC_Differential_0_1() * multiplier;
  results4 = ads2.readADC_Differential_2_3() * multiplier;

  //faz a leitura dos sensores
  bits_fsr5 = analogRead(pin_fsr5);
  fsr5 = map(bits_fsr5, 0, 4095, 0, 3300);
  fsr5_value = (fsr5 /  1.000) / 1000; //normalizacao do valor lido

  bits_fsr6 = analogRead(pin_fsr6);
  fsr6 = map(bits_fsr6, 0, 4095, 0, 3300);
  fsr6_value = (fsr6 /  1.000) / 1000; //normalizacao do valor lido

  bits_fsr7 = analogRead(pin_fsr7);
  fsr7 = map(bits_fsr7, 0, 4095, 0, 3300);
  fsr7_value = (fsr7 /  1.000) / 1000; //normalizacao do valor lido

  bits_fsr8 = analogRead(pin_fsr8);
  fsr8 = map(bits_fsr8, 0, 4095, 0, 3300);
  fsr8_value = (fsr8 /  1.000) / 1000; //normalizacao do valor lido

  //recebe os valores iniciais para referencia
  if (k <= 7) {
    comp_fsr1[k] = results1;
    comp_fsr2[k] = results2;
    comp_fsr3[k] = results3;
    comp_fsr4[k] = results4;
    comp_fsr5[k] = fsr5_value;
    comp_fsr6[k] = fsr6_value;
    comp_fsr7[k] = fsr7_value;
    comp_fsr8[k] = fsr8_value;
  }
  else {
    //
  }
  k = k + 1; //incrementa o valor de k

  //faz a media dos valores de referencia
  sum_comp_fsr1 = (1 / 8.0) * (comp_fsr1[0] + comp_fsr1[1] + comp_fsr1[2] + comp_fsr1[3] + comp_fsr1[4] + comp_fsr1[5] + comp_fsr1[6] + comp_fsr1[7]);
  sum_comp_fsr2 = (1 / 8.0) * (comp_fsr2[0] + comp_fsr2[1] + comp_fsr2[2] + comp_fsr2[3] + comp_fsr2[4] + comp_fsr2[5] + comp_fsr2[6] + comp_fsr2[7]);
  sum_comp_fsr3 = (1 / 8.0) * (comp_fsr3[0] + comp_fsr3[1] + comp_fsr3[2] + comp_fsr3[3] + comp_fsr3[4] + comp_fsr3[5] + comp_fsr3[6] + comp_fsr3[7]);
  sum_comp_fsr4 = (1 / 8.0) * (comp_fsr4[0] + comp_fsr4[1] + comp_fsr4[2] + comp_fsr4[3] + comp_fsr4[4] + comp_fsr4[5] + comp_fsr4[6] + comp_fsr4[7]);
  sum_comp_fsr5 = (1 / 8.0) * (comp_fsr5[0] + comp_fsr5[1] + comp_fsr5[2] + comp_fsr5[3] + comp_fsr5[4] + comp_fsr5[5] + comp_fsr5[6] + comp_fsr5[7]);
  sum_comp_fsr6 = (1 / 8.0) * (comp_fsr6[0] + comp_fsr6[1] + comp_fsr6[2] + comp_fsr6[3] + comp_fsr6[4] + comp_fsr6[5] + comp_fsr6[6] + comp_fsr6[7]);
  sum_comp_fsr7 = (1 / 8.0) * (comp_fsr7[0] + comp_fsr7[1] + comp_fsr7[2] + comp_fsr7[3] + comp_fsr7[4] + comp_fsr7[5] + comp_fsr7[6] + comp_fsr7[7]);
  sum_comp_fsr8 = (1 / 8.0) * (comp_fsr8[0] + comp_fsr8[1] + comp_fsr8[2] + comp_fsr8[3] + comp_fsr8[4] + comp_fsr8[5] + comp_fsr8[6] + comp_fsr8[7]);

  //
  FSR_1 = (-1.0 * (results1 - sum_comp_fsr1));
  FSR_2 = (-1.0 * (results2 - sum_comp_fsr2));
  FSR_3 = (-1.0 * (results3 - sum_comp_fsr3));
  FSR_4 = (-1.0 * (results4 - sum_comp_fsr4));

  FSR_5 =  1000 * (1.0 * (fsr5_value - sum_comp_fsr5));
  FSR_6 =  1000 * (1.0 * (fsr6_value - sum_comp_fsr6));
  FSR_7 =  1000 * (1.0 * (fsr7_value - sum_comp_fsr7));
  FSR_8 =  1000 * (1.0 * (fsr8_value - sum_comp_fsr8));

  //tt01[1] = micros();

  /*
    Serial.print("\n Tempo de leitura: ");
    Serial.print((tt01[1]-tt01[0])/1E3);
  */
  //nao sei oq isso faz ainda
  uint8_t * toSend1 ;//= (uint8_t *)(results1 * multiplier);
  float Values[_N_SENSORS] = {FSR_1, FSR_2, FSR_3, FSR_4, FSR_5, FSR_6, FSR_7, FSR_8};
  toSend1 = (uint8_t *) &Values;

  for (int idx_packet = 1 ;  idx_packet <= _N_SENSORS / 2 ; idx_packet++) {
    long cur_id = (_TIPO_PLACA << (4 * 4)) | ((_ID << (2 * 4)) | (idx_packet));
    CAN.beginExtendedPacket(cur_id);

    CAN.write(toSend1 + (idx_packet - 1) * 8, 8); //1º byte

    CAN.endPacket();
  }
  if (SHOW_SERIAL) {
    Serial.print(Values[0])  ;
    Serial.print("\t")  ;
    Serial.print(Values[1])  ;
    Serial.print("\t")  ;
    Serial.print(Values[2])  ;
    Serial.print("\t")  ;
    Serial.print(Values[3])  ;
    Serial.print("\t");
    Serial.print(Values[4])  ;
    Serial.print("\t")  ;
    Serial.print(Values[5])  ;
    Serial.print("\t")  ;
    Serial.print(Values[6])  ;
    Serial.print("\t")  ;
    Serial.print(Values[7])  ;
    Serial.println();
  }
}
