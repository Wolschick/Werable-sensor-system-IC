#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

const bool print_raw_data = false;
const bool print_predict_data = true;

// ****** Configure NN ******
#define N_INPUTS 3
#define N_OUTPUTS 1

float input_left[3];
float input_right[3];
float predicted_left;
float predicted_right;

//values to normalization
float X_min_left[3] = { -2.628137, -11.220687, -13.683275};
float X_min_right[3] = { -7.5986523, -13.263739, -4.9133552};
float X_max_left[3] = {54.25338, 20.417468, 73.87925};
float X_max_right[3] = {31.203948, 34.922109, 74.696511};

float y_min_left = -0.33344511;
float y_min_right = -0.49819414;
float y_max_left = 1.5220011;
float y_max_right = 1.5034327;

using namespace BLA;

//left NN values
BLA::Matrix<10, 3> weights_left_01 = {2.6969, 0.25120, -0.20288,
                                      3.9267, 0.069862, -3.5376,
                                      -0.55657,  2.9686,  2.2228,
                                      -0.17401, 2.0986, -0.59466,
                                      1.9199, -0.82773,  2.3809,
                                      -1.4002,  3.7400,  0.26487,
                                      0.037023, 1.4778, -6.2603,
                                      0.0037620, 0.87886, 1.3885,
                                      -5.2455,  1.7575,  -1.6683,
                                      -1.9631,  1.1060,  -2.6025
                                     };

BLA::Matrix<10, 1> weights_left_02 = {0.5978, -0.5080,  0.6155,  0.7054,  1.1934, -0.8143,  0.5375, -0.4729, 1.1159, -0.8750};

BLA::Matrix<1, 10> bias_left_01 = {0.4384, -0.7287,  1.2950, -0.1125, -0.7230, -0.1742,  0.1798,  1.0424, -0.6126, -0.8978};
BLA::Matrix<1, 1> bias_left_02 = 0.0541;

//right NN values
BLA::Matrix<10, 3> weights_right_01 = { -0.0661, -0.6414,  0.2369,
                                        0.2263, -0.3797, -0.4111,
                                        0.7571, -0.1531,  0.3674,
                                        0.0255,  0.4422, -0.6321,
                                        0.9018,  0.2939,  0.1033,
                                        0.8670,  0.2903,  0.1774,
                                        0.9729, -1.1841, -1.3737,
                                        0.4242, -0.1431,  0.4690,
                                        -0.3586,  0.4974,  0.1811,
                                        -0.1579,  0.6293, -0.0979
                                      };

BLA::Matrix<10, 1> weights_right_02 = {0.0444,  0.2987,  0.4944, -0.2593,  0.4846, -0.2459, -0.6383, -0.2385, -0.3066,  0.3556};

BLA::Matrix<1, 10> bias_right_01 = { -0.0227, -0.2376,  0.5926,  0.0953,  0.6165, -0.4003,  0.4030, -0.2370, 0.1927, -0.3531};
BLA::Matrix<1, 1> bias_right_02 = 0.0354;

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

  input_left[0] = (2 * (myData.NOW_S1_Xflex - X_min_left[0]) / (X_max_left[0] - X_min_left[0])) - 1;
  input_left[1] = (2 * (-myData.NOW_S1_Yflex - X_min_left[1]) / (X_max_left[1] - X_min_left[1])) - 1;
  input_left[2] = (2 * ((-myData.NOW_S1_POF/10) - X_min_left[2]) / (X_max_left[2] - X_min_left[2])) - 1;

  input_right[0] = (2 * (-myData.NOW_S2_Xflex - X_min_right[0]) / (X_max_right[0] - X_min_right[0])) - 1;
  input_right[1] = (2 * (-myData.NOW_S2_Yflex - X_min_right[1]) / (X_max_right[1] - X_min_right[1])) - 1;
  input_right[2] = (2 * ((-myData.NOW_S2_POF/10) - X_min_right[2]) / (X_max_right[2] - X_min_right[2])) - 1;

  if (print_raw_data == true) {
    Serial.print(myData.NOW_S1_Xflex);
    Serial.print("\t");
    Serial.print(-myData.NOW_S1_Yflex);
    Serial.print("\t");

    Serial.print(-myData.NOW_S1_POF / 10);
    Serial.print("\t");

    Serial.print(-myData.NOW_S2_Xflex);
    Serial.print("\t");
    Serial.print(-myData.NOW_S2_Yflex);
    Serial.print("\t");

    Serial.print(-myData.NOW_S2_POF / 10);
    Serial.print("\n");
    /*
        Serial.print(myData.NOW_IMU_Tetha);
        Serial.print("\n");

        Serial.print(myData.NOW_IMU_Phi);
        Serial.print("\t");
        Serial.print(myData.NOW_IMU_Psi);
        Serial.print("\n");
    */
  }
}

void setup()
{
  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
}

void loop()
{

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  //predict values
  predicted_left = (((predict_left(input_left) + 1) * (y_max_left - y_min_left) / 2) + y_min_left);
  predicted_right = (((predict_right(input_right) + 1) * (y_max_right - y_min_right) / 2) + y_min_right);

  if (print_predict_data == true) {
    Serial.print("predicted left value: ");
    Serial.print((180 * predicted_left) / 3.141592);
    Serial.print("\t|\t");
    Serial.print("predicted right value: ");
    Serial.print((180 * predicted_right) / 3.141592);
    Serial.print("\n");
  }
}

float predict_left(float input_val[])
{
  BLA::Matrix<1, 3> X_values;
  X_values(0, 0) = input_val[0];
  X_values(0, 1) = input_val[1];
  X_values(0, 2) = input_val[2];

  //first layer
  BLA::Matrix<1, 10>Matx_result = ((X_values * (~weights_left_01)) + bias_left_01);
  BLA::Matrix<1, 10>Matx_activ;

  for (int j = 0; j < 10; j++) {
    Matx_activ(0, j) = tanh(Matx_result(0, j));
  }

  //second layer
  BLA::Matrix<1, 1> Matx_predicted = ((Matx_activ * weights_left_02) + bias_left_02);
  float predicted_value = Matx_predicted(0, 0);

  return predicted_value;
}

float predict_right(float input_val[])
{
  BLA::Matrix<1, 3> X_values;
  X_values(0, 0) = input_val[0];
  X_values(0, 1) = input_val[1];
  X_values(0, 2) = input_val[2];

  //first layer
  BLA::Matrix<1, 10>Matx_result = ((X_values * (~weights_right_01)) + bias_right_01);
  BLA::Matrix<1, 10>Matx_activ;

  for (int j = 0; j < 10; j++) {
    Matx_activ(0, j) = tanh(Matx_result(0, j));
  }

  //second layer
  BLA::Matrix<1, 1> Matx_predicted = ((Matx_activ * weights_right_02) + bias_right_02);
  float predicted_value = Matx_predicted(0, 0);

  return predicted_value;
}
