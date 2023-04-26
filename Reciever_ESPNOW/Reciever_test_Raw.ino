#include <esp_now.h>
#include <WiFi.h>


// Define a data structure
typedef struct struct_message {
  float NOW_S1_Xflex;
  float NOW_S1_Yflex;
  float NOW_S1_POF;
  float NOW_S2_Xflex;
  float NOW_S2_Yflex;
  float NOW_S2_POF;
} struct_message;

float sensors_values[9];

// Create a structured object
struct_message myData;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  //Left leg sensors
  sensors_values[0] = myData.NOW_S1_Xflex;
  sensors_values[1] = myData.NOW_S1_Yflex;
  sensors_values[2] = (myData.NOW_S1_POF / 10);
  //right leg sensors
  sensors_values[3] = myData.NOW_S2_Xflex;
  sensors_values[4] = myData.NOW_S2_Yflex;
  sensors_values[5] = (myData.NOW_S2_POF / 10);

  sensors_values[6] = 0;
  sensors_values[7] = 0;
  sensors_values[8] = 0;

  /*
    Serial.print(myData.NOW_S1_Xflex);
    Serial.print("\t");
    Serial.print(myData.NOW_S1_Yflex);
    Serial.print("\t");
    Serial.print(myData.NOW_S1_POF / 10);
    Serial.print("\t");
    Serial.print(myData.NOW_S2_Xflex);
    Serial.print("\t");
    Serial.print(myData.NOW_S2_Yflex);
    Serial.print("\t");
    Serial.print(myData.NOW_S2_POF / 10);
    Serial.print("\n");
  */
}

void serialEvent() {
  char cmd = Serial.read();
  if (cmd == 'p') {
    uint8_t * toSend1;
    toSend1 = (uint8_t *) &sensors_values;
    for (int i = 0; i < (9 * 4); i++) {
      Serial.write(*(toSend1 + i));
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  serialEvent();
}
