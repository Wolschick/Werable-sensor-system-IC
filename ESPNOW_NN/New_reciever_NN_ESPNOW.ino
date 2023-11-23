#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

const bool print_raw_data = false;
const bool print_predict_data = true;
const bool send_serial_data = false;
const bool print_new_pof_sens = false;

float predicted_values[2];

// ****** Configure NN ******
#define N_INPUTS 3
#define N_OUTPUTS 1

float input_left[3];
float input_right[3];
float predicted_left;
float predicted_right;

//values to normalization
float X_min_left[3] = { -2.628137, -11.220687, -13.683275 };
float X_min_right[3] = { -7.5986523, -13.263739, -4.9133552 };
float X_max_left[3] = { 54.25338, 20.417468, 73.87925 };
float X_max_right[3] = { 31.203948, 34.922109, 74.696511 };

float y_min_left = -0.33344511;
float y_min_right = -0.49819414;
float y_max_left = 1.5220011;
float y_max_right = 1.5034327;

using namespace BLA;

/*
  *
  *
  HIPERPARAM LEFT NN
  *
  *
*/

//wights
BLA::Matrix<10, 3> weights_left_01 = { 1.8279, 1.8014, -0.6827,
                                       3.6316, 0.3359, -2.7213,
                                       3.9101, -1.4592, 1.3439,
                                       -1.4816, 1.0118, 0.8364,
                                       0.5414, 0.3522, 0.9590,
                                       1.5381, -0.0461, -2.1328,
                                       0.0178, 4.8426, -0.6682,
                                       0.1437, -1.1565, -0.1560,
                                       -2.9897, -2.0152, -0.6888,
                                       -1.8883, -0.1373, -2.5139 };

BLA::Matrix<10, 10> weights_left_02 = { 0.9098, -1.9595, 0.6570, 1.2161, 0.3708, 0.3839, 1.1169, -0.0539, 0.9469, -0.4290,
                                        0.5676, -0.3877, -1.5497, 2.5699, 0.7216, -0.0266, 0.9424, -1.1674, 1.2148, -0.3331,
                                        -0.0528, 0.5379, 1.4088, -0.7648, 0.1583, 0.3531, -0.3242, 0.2789, 0.3814, -0.1307,
                                        -0.0252, -0.5772, 0.5797, 0.8800, -0.0397, -0.2350, -0.4590, -0.1440, -0.2951, -0.0478,
                                        -0.6898, -0.5691, 0.5058, -0.5279, 1.0544, -0.9848, -0.1601, -0.7279, -0.4572, 0.6726,
                                        -0.3336, 0.3611, 1.5508, -0.3973, -0.1733, -0.1349, 0.2123, 0.2385, 0.0411, -0.3200,
                                        0.3553, 1.0241, -0.3854, -0.8696, -0.0352, 0.3657, -0.6096, -0.0578, -0.8746, -0.0751,
                                        1.2116, -1.3192, 1.6560, -2.4680, -0.3140, 0.3179, -1.2033, 0.4961, -0.3870, -1.2448,
                                        0.0691, -0.5403, -0.4048, -0.5167, -0.2551, -0.8708, -0.6172, -0.4705, 0.0579, -0.1349,
                                        -0.9611, 0.6183, -1.6420, 0.2523, -0.1036, -0.6776, -0.4897, -0.6380, -0.1040, 0.3480 };

BLA::Matrix<10, 1> weights_left_03 = { 0.7885, -0.6347, -0.9617, -0.5842, 0.9938, -0.5177,
                                       1.0064, 0.4143, 0.6428, -0.6978 };

//bias
BLA::Matrix<1, 10> bias_left_01 = { 1.2320, 0.3851, 0.9245, -0.2693, -0.5522, 0.2158,
                                    0.5749, 0.5941, -0.1594, -0.7440 };

BLA::Matrix<1, 10> bias_left_02 = { -0.2165, -0.9386, 0.4714, -0.4549, -0.0884, 0.3019,
                                    0.1480, -0.0771, 0.0421, -0.4153 };

BLA::Matrix<1, 1> bias_left_03 = 0.0316;


/*
  *
  *
  HIPERPARAM LEFT NN
  *
  *
*/

//weights
BLA::Matrix<10, 3> weights_right_01 = { 0.0531, -0.0946, 0.8168,
                                        -0.8389, -1.3760, -0.6275,
                                        0.3832, -0.5196, 0.2573,
                                        1.0091, -0.1782, -0.8803,
                                        0.7399, 0.1714, -0.0042,
                                        -0.3653, -0.0667, 0.1403,
                                        1.1073, -1.1690, -1.0515,
                                        0.3917, 0.0454, 0.3597,
                                        -1.3621, 0.6268, -0.5685,
                                        -0.4773, 0.2121, -0.2731 };

BLA::Matrix<10, 10> weights_right_02 = { -0.2199, 0.1311, 0.0816, -0.1520, -0.0643, 0.0215, -0.0736, -0.3448, -0.0447, 0.3456,
                                         0.1123, 0.2152, -0.2323, 0.2795, 0.2642, 0.1035, -0.2401, 0.0530, -0.4221, -0.4275,
                                         -0.2089, 0.0319, -0.5281, 0.0500, 0.1875, -0.3672, -0.0759, 0.0111, 0.3063, 0.0470,
                                         0.1997, 0.1625, 0.7176, 0.2146, 0.0120, -0.0531, 0.3349, 0.0658, -0.3252, -0.4571,
                                         0.4610, -0.4284, -0.4319, -0.8664, -0.0803, 0.2839, -1.1876, 0.1171, 0.8381, -0.1687,
                                         0.0627, 0.0967, 0.3850, 0.1173, 0.2675, 0.2206, 0.2762, 0.2184, -0.1840, -0.1885,
                                         0.0994, 0.1659, 0.3328, 0.2272, -0.0657, 0.0152, 0.1167, 0.0649, -0.2492, 0.1745,
                                         0.0118, 0.3020, -0.1282, -0.3816, -0.6176, 0.2356, -0.5515, -0.3576, -0.1750, 0.7037,
                                         0.1851, 0.0945, 0.2142, 0.2049, 0.4160, -0.0298, -0.2221, 0.3582, -0.0107, -0.4143,
                                         0.0336, -0.3175, 0.1371, -0.1181, -0.1071, -0.1699, -0.0909, 0.3167, 0.0490, 0.2299 };

BLA::Matrix<10, 1> weights_right_03 = { -0.1423, 0.4658, -0.0842, -0.1255, 0.4224,
                                        -0.0799, 0.1704, -0.3922, 0.3509, -0.0736 };

//bias
BLA::Matrix<1, 10> bias_right_01 = { 0.1114, -0.3746, 0.2991, 0.3797, 0.5945,
                                     -0.2910, 0.3716, 0.2443, 0.5771, -0.4506 };

BLA::Matrix<1, 10> bias_right_02 = { 0.0554, 0.2051, 0.2948, -0.0547, -0.1986,
                                     0.1408, -0.0541, 0.0179, 0.2634, -0.2762 };

BLA::Matrix<1, 1> bias_right_03 = -0.2372;

// ************
// Define a data structure
typedef struct struct_message {
  float NOW_S1_Xflex;
  float NOW_S1_Yflex;
  float NOW_S1_POF;
  float NOW_S2_Xflex;
  float NOW_S2_Yflex;
  float NOW_S2_POF;
  float NOW_FINT_POF_A;
  float NOW_FINT_POF_B;
} struct_message;

// Create a structured object
struct_message myData;

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  input_left[0] = (2 * (myData.NOW_S1_Xflex - X_min_left[0]) / (X_max_left[0] - X_min_left[0])) - 1;
  input_left[1] = (2 * (-myData.NOW_S1_Yflex - X_min_left[1]) / (X_max_left[1] - X_min_left[1])) - 1;
  input_left[2] = (2 * ((-myData.NOW_S1_POF / 10) - X_min_left[2]) / (X_max_left[2] - X_min_left[2])) - 1;

  input_right[0] = (2 * (-myData.NOW_S2_Xflex - X_min_right[0]) / (X_max_right[0] - X_min_right[0])) - 1;
  input_right[1] = (2 * (-myData.NOW_S2_Yflex - X_min_right[1]) / (X_max_right[1] - X_min_right[1])) - 1;
  input_right[2] = (2 * ((-myData.NOW_S2_POF / 10) - X_min_right[2]) / (X_max_right[2] - X_min_right[2])) - 1;

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

  predicted_values[2] = 0;
  predicted_values[3] = 0;
  predicted_values[4] = 0;
  predicted_values[5] = 0;
  predicted_values[6] = 0;
  predicted_values[7] = 0;
  //predicted_values[8] = 0;
}

void loop() {

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

  if (print_new_pof_sens == true) {
    Serial.print(myData.NOW_FINT_POF_A);
    Serial.print("\t");
    Serial.print(myData.NOW_FINT_POF_B);
    Serial.print("\n");
  }

  predicted_values[0] = predicted_left;
  predicted_values[1] = predicted_right;

  if (send_serial_data == true) {
    serialEvent();
  }
}

void serialEvent() {
  char cmd = Serial.read();
  if (cmd == 'p') {
    uint8_t *toSend1;
    toSend1 = (uint8_t *)&predicted_values;
    for (int i = 0; i < (9 * 4); i++) {
      Serial.write(*(toSend1 + i));
    }
  }
}

float predict_left(float input_val[]) {
  BLA::Matrix<1, 3> X_values;
  X_values(0, 0) = input_val[0];
  X_values(0, 1) = input_val[1];
  X_values(0, 2) = input_val[2];

  //first layer
  BLA::Matrix<1, 10> Matx_result_first = ((X_values * (~weights_left_01)) + bias_left_01);
  BLA::Matrix<1, 10> Matx_activ_first;

  for (int j = 0; j < 10; j++) {
    Matx_activ_first(0, j) = tanh(Matx_result_first(0, j));
  }

  //second layer
  BLA::Matrix<1, 10> Matx_result_second = ((Matx_activ_first * (~weights_left_02)) + bias_left_02);
  BLA::Matrix<1, 10> Matx_activ_second;

  for (int j = 0; j < 10; j++) {
    for (int i = 0; i < 10; i++) {
      Matx_activ_second(i, j) = tanh(Matx_result_second(i, j));
    }
  }

  //third layer
  BLA::Matrix<1, 1> Matx_predicted = ((Matx_activ_second * weights_left_03) + bias_left_03);
  float predicted_value = Matx_predicted(0, 0);

  return predicted_value;
}

float predict_right(float input_val[]) {
  BLA::Matrix<1, 3> X_values;
  X_values(0, 0) = input_val[0];
  X_values(0, 1) = input_val[1];
  X_values(0, 2) = input_val[2];

  //first layer
  BLA::Matrix<1, 10> Matx_result_first = ((X_values * (~weights_right_01)) + bias_right_01);
  BLA::Matrix<1, 10> Matx_activ_first;

  for (int j = 0; j < 10; j++) {
    Matx_activ_first(0, j) = tanh(Matx_result_first(0, j));
  }

  //second layer
  BLA::Matrix<1, 10> Matx_result_second = ((Matx_activ_first * (~weights_right_02)) + bias_right_02);
  BLA::Matrix<1, 10> Matx_activ_second;

  for (int j = 0; j < 10; j++) {
    for (int i = 0; i < 10; i++) {
      Matx_activ_second(i, j) = tanh(Matx_result_first(i, j));
    }
  }

  //third layer
  BLA::Matrix<1, 1> Matx_predicted = ((Matx_activ_second * weights_right_03) + bias_right_03);
  float predicted_value = Matx_predicted(0, 0);

  return predicted_value;
}
