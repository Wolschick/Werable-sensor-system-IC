#include <esp_now.h>
#include <WiFi.h>
#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

// ****** COnfigure NN ******
#include "left_model.h"
#include "right_model.h"

#define N_INPUTS 3
#define N_OUTPUTS 1
#define TENSOR_ARENA_SIZE 4*1024

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf_left;
Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf_right;

float input_left[3];
float input_right[3];
float predicted_left;
float predicted_right;

//values to normalization
float X_min_left[3] = { -3.505, -6.47, -10.48};
float X_min_right[3] = { -4.83, -10.2, -7.62};
float X_max_left[3] = {67.51, 30.38, 50.49};
float X_max_right[3] = {66.15, 40.16, 26.12};

float y_min_left = -5.5;
float y_min_right = -8.2;
float y_max_left = 78.8;
float y_max_right = 83.1;

// ************


// Define a data structure
typedef struct struct_message {
  float NOW_S1_Xflex;
  float NOW_S1_Yflex;
  float NOW_S1_POF;
  float NOW_S2_Xflex;
  float NOW_S2_Yflex;
  float NOW_S2_POF;
} struct_message;

// Create a structured object
struct_message myData;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  input_left[0] = ((myData.NOW_S1_Xflex - X_min_left[0])/(X_max_left[0] - X_min_left[0]));
  input_left[1] = ((myData.NOW_S1_Yflex - X_min_left[1])/(X_max_left[1] - X_min_left[1]));
  input_left[2] = ((myData.NOW_S1_POF - X_min_left[2])/(X_max_left[2] - X_min_left[2]));

  input_right[0] = ((myData.NOW_S2_Xflex - X_min_right[0])/(X_max_right[0] - X_min_right[0]));
  input_right[1] = ((myData.NOW_S2_Yflex - X_min_right[1])/(X_max_right[1] - X_min_right[1]));
  input_right[2] = ((myData.NOW_S2_POF - X_min_right[2])/(X_max_right[2] - X_min_right[2]));

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

    Serial.print(myData.NOW_IMU_Tetha);
    Serial.print("\n");

    Serial.print(myData.NOW_IMU_Phi);
    Serial.print("\t");
    Serial.print(myData.NOW_IMU_Psi);
    Serial.print("\n");*/

}

void setup() {
  Serial.begin(115200);

  //Inicialize models
  tf_left.begin(left_model);
  tf_right.begin(right_model);

  // check if left model loaded fine
  if (!tf_left.isOk()) {
    Serial.print("ERROR: ");
    Serial.println(tf_left.getErrorMessage());
    while (true) delay(1000);
  }
  // check if right model loaded fine
  if (!tf_right.isOk()) {
    Serial.print("ERROR: ");
    Serial.println(tf_right.getErrorMessage());
    while (true) delay(1000);
  }

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

  //predict values
  predicted_left = ((tf_left.predict(input_left)*(y_max_left - y_min_left)) + y_min_left);
  predicted_right = ((tf_right.predict(input_right)*(y_max_right - y_min_right)) + y_min_right);

  Serial.print("predicted left value: ");
  Serial.print(predicted_left);
  Serial.print("\t|\t");
  Serial.print("predicted right value: ");
  Serial.print(predicted_right);
  Serial.print("\n");
}
