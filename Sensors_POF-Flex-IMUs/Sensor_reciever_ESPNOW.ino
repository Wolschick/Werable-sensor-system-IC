/*
  ESP-NOW Demo - Receive
  esp-now-demo-rcv.ino
  Reads data from Initiator
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Include Libraries
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
  float NOW_IMU_Tetha;
  float NOW_IMU_Phi;
  float NOW_IMU_Psi;
} struct_message;

// Create a structured object
struct_message myData;


// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("\t Data received: ");
  //Serial.print(len);
  //Serial.print("\t");
  
  Serial.print(myData.NOW_S1_Xflex);
  Serial.print("\t");
  Serial.print(myData.NOW_S1_Yflex);
  Serial.print("\t");
  
  Serial.print(myData.NOW_S1_POF/10);
  Serial.print("\t");
 
  Serial.print(myData.NOW_S2_Xflex);
  Serial.print("\t");
  Serial.print(myData.NOW_S2_Yflex);
  Serial.print("\t");
 
  Serial.print(myData.NOW_S2_POF/10);
  Serial.print("\t");

  Serial.print(myData.NOW_IMU_Tetha);
  Serial.print("\t");
  Serial.print(myData.NOW_IMU_Phi);
  Serial.print("\t");
  Serial.print(myData.NOW_IMU_Psi);
  Serial.print("\n");
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(1000000);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  
}
 
void loop() {

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

}
