#define _N_SENSORS_MAX 8

#define CALC_FS     false
#define SHOW_SERIAL true
#define SHOW_HEADER false
#define CAN_SPEED 1000E3
#include <CAN.h>

float values_11[_N_SENSORS_MAX];
uint8_t * pValues_11;

float values_12[_N_SENSORS_MAX];
uint8_t * pValues_12;

float valSum11;
float valSum12;

long t1;
long samples = 0;

void setup() {
  pValues_11 = (uint8_t *) &values_11;
  pValues_12 = (uint8_t *) &values_12;
  Serial.begin(115200);
  while (!Serial);

  Serial.println("CAN Receiver");
  CAN.setPins(4, 5);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(CAN_SPEED)) { //1000E3
    Serial.println("Starting CAN failed!");
    while (1);
  }
  t1 = micros();
}

void loop() {
  if (CALC_FS)
    if (micros() > (t1 + 1E6)) {
      Serial.print("Se han leido muestas: ");
      Serial.print((int)samples / 2);
      Serial.print("\n");
      t1 = micros();
      samples = 0;
    }

  int packetSize = CAN.parsePacket();
  if (packetSize) {
    /*
      CAN.packetId() , HEX
    */
    if (CAN.packetExtended()) {
      long id_cur = CAN.packetId();
      uint16_t tipo = (id_cur & 0xFF0000) >> (4 * 4);
      uint16_t id =   (id_cur & 0x00FF00) >> (4 * 2);
      uint16_t idx_packet = (id_cur & 0x0000FF);

      if (SHOW_HEADER) {
        Serial.print("\n");
        Serial.print(tipo);
        Serial.print("\t");
        Serial.print(id);
        Serial.print("\t");
        Serial.print(idx_packet);
        Serial.print("\n");
      }

      if ((int)id == 11) {
        for (int iv = 0  ; iv < packetSize  ; iv++)
          pValues_11[iv + (idx_packet - 1) * 8 ] = (uint8_t) CAN.read();
      }
      if ((int)id == 12) {
        for (int iv = 0  ; iv < packetSize  ; iv++)
          pValues_12[iv + (idx_packet - 1) * 8 ] = (uint8_t) CAN.read();
      }

      samples++;
    }

    if (SHOW_SERIAL) {
      Serial.print("\n");
      for (int ix = 0; ix < _N_SENSORS_MAX; ix++) {
        Serial.print(values_11[ix]);
        Serial.print("\t");
      }
      for (int ix = 0; ix < _N_SENSORS_MAX; ix++) {
        Serial.print(values_12[ix]);
        Serial.print("\t");
      }
      Serial.print("\n");
    }
  }
}
