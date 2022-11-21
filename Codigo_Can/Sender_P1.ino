
#define _TIPO_PLACA 1
#define _ID 10
#define CAN_SPEED 1000E3
#define SHOW_SERIAL false

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

int k;

float comp_pof1[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_pof2[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_pof3[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float comp_pof4[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float sum_comp_pof1 = 0.0;
float sum_comp_pof2 = 0.0;
float sum_comp_pof3 = 0.0;
float sum_comp_pof4 = 0.0;

float pof_1 = 0.0;
float pof_2 = 0.0;
float pof_3 = 0.0;
float pof_4 = 0.0;


void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads1.setGain(GAIN_FOUR); 
  ads2.setGain(GAIN_FOUR); 

  ads1.setDataRate(RATE_ADS1015_3300SPS);
  ads2.setDataRate(RATE_ADS1015_3300SPS);
  
  // Inicia o barramento CAN a 500 kbps
  Serial.println("Iniciando CAN");
  CAN.setPins(4,5);
  if (!CAN.begin(CAN_SPEED)) { // 1000E3
    Serial.println("Failed to initialize CAN"); 
    while (1);
  }

  
  Serial.println("Conectando ads1");
  if (!ads1.begin(0x48,&Wire)) {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
  Serial.println("Conectando ads2");
  if (!ads2.begin(0x49,&Wire)) {
    Serial.println("Failed to initialize ADS2.");
    while (1);
  }



}

long tt01[2];

void loop(void)
{
  tt01[0] = micros();
  //tempo = micros();
  int16_t results1, results2, results3, results4;

  /* Be sure to update this value based on the IC and the gain settings! */
  
    //float   multiplier_adc1 = 0.03125F; 
    float   multiplier_adc1 = 0.5F; 
    
    //float   multiplier_adc2 = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
    //float multiplier_adc2= 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
    float   multiplier_adc2 = 0.5F;    

  results1 = ads1.readADC_Differential_0_1() * multiplier_adc1;
  results2 = ads1.readADC_Differential_2_3() * multiplier_adc1;
  results3 = ads2.readADC_Differential_0_1() * multiplier_adc2;
  results4 = ads2.readADC_Differential_2_3() * multiplier_adc2;

  if (k <= 7) {
    comp_pof1[k] = results1;
    comp_pof2[k] = results2;
    comp_pof3[k] = results3;
    comp_pof4[k] = results4;
  }
  else {
    //
  }

  k = k+1;
  
  sum_comp_pof1 = (1/8.0)*(comp_pof1[0] + comp_pof1[1] + comp_pof1[2] + comp_pof1[3] + comp_pof1[4] + comp_pof1[5] + comp_pof1[6] + comp_pof1[7]);
  sum_comp_pof2 = (1/8.0)*(comp_pof2[0] + comp_pof2[1] + comp_pof2[2] + comp_pof2[3] + comp_pof2[4] + comp_pof2[5] + comp_pof2[6] + comp_pof2[7]);
  sum_comp_pof3 = (1/8.0)*(comp_pof3[0] + comp_pof3[1] + comp_pof3[2] + comp_pof3[3] + comp_pof3[4] + comp_pof3[5] + comp_pof3[6] + comp_pof3[7]);
  sum_comp_pof4 = (1/8.0)*(comp_pof4[0] + comp_pof4[1] + comp_pof4[2] + comp_pof4[3] + comp_pof4[4] + comp_pof4[5] + comp_pof4[6] + comp_pof4[7]);

  pof_1 = (-1.0 * (results1 - sum_comp_pof1));
  pof_2 = (-1.0 * (results2 - sum_comp_pof2));
  pof_3 = (-1.0 * (results3 - sum_comp_pof3));
  pof_4 = (-1.0 * (results4 - sum_comp_pof4));

  tt01[1] = micros();

  /*
   Serial.print("\n Tempo de leitura: ");
   Serial.print((tt01[1]-tt01[0])/1E3);
  */
  uint8_t * toSend1 ;//= (uint8_t *)(results1 * multiplier);
  float Values[_N_SENSORS] = {pof_1, pof_2, pof_3, pof_4};
  toSend1 = (uint8_t *) &Values;

  for(int idx_packet = 1 ;  idx_packet <= _N_SENSORS/2 ; idx_packet++){
    long cur_id = (_TIPO_PLACA<<(4*4)) | ((_ID<<(2*4)) | (idx_packet));
    CAN.beginExtendedPacket(cur_id);
    
    CAN.write(toSend1 + (idx_packet-1)*8,8); //1º byte

    CAN.endPacket();
  }  
  if(SHOW_SERIAL){
  Serial.print(Values[0])  ;
  Serial.print("\t")  ;
  Serial.print(Values[1])  ;
  Serial.print("\t")  ;
  Serial.print(Values[2])  ;
  Serial.print("\t")  ;
  Serial.print(Values[3])  ;
  Serial.println();
  }
}
