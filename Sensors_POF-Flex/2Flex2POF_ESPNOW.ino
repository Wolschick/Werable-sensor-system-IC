#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#include <Adafruit_SleepyDog.h>

#include <esp_now.h>
#include <WiFi.h>

#include <BasicLinearAlgebra.h>
using namespace BLA;

template <int dim, class ElemT>
struct Diagonal
{
    mutable ElemT m[dim];
    mutable ElemT offDiagonal;

    // The only requirement on this class is that it implement the () operator like so:
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
        if (row == col)
            return m[row];
        else
            // Otherwise return a zero
            return (offDiagonal = 0);
    }
};


// Variables for test data
int int_value;
float float_value;
bool bool_value = true;
float Flex1_X_lpfilt; 
float Flex1_X;
float Flex1_X_ant = 0; 
float Flex1_Y_lpfilt; 
float Flex1_Y;
float Flex1_Y_ant = 0; 

float Flex2_X_lpfilt; 
float Flex2_X;
float Flex2_X_ant = 0; 
float Flex2_Y_lpfilt; 
float Flex2_Y;
float Flex2_Y_ant = 0; 


float pof1_lpfilt; 
float pof1_ant = 0; 
float pof2_lpfilt; 
float pof2_ant = 0; 

float low_pass_param_1 = 0.0;
float low_pass_param_2 = 1.0;

      float unit = 1.0;
      float Rval = 100.0;
      float Qval = 1.0;
      int ii = 0;
      
      BLA::Matrix<6, 6> Pk = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6, 6> Pk_update = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6, 6> Pk_prev = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6> Xk_est = {0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6> Xk_update = {0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6> Xk_prev = {0.0,0.0,0.0,0.0,0.0,0.0}; 
      BLA::Matrix<6, 6> Kk = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6, 6> Rk = {Rval,0.0,0.0,0.0,0.0,0.0,0.0,Rval,0.0,0.0,0.0,0.0,0.0,0.0,Rval,0.0,0.0,0.0,0.0,0.0,0.0,Rval,0.0,0.0,0.0,0.0,0.0,0.0,Rval,0.0,0.0,0.0,0.0,0.0,0.0,Rval};
      BLA::Matrix<6, 6> Qk = {Qval,0.0,0.0,0.0,0.0,0.0,0.0,Qval,0.0,0.0,0.0,0.0,0.0,0.0,Qval,0.0,0.0,0.0,0.0,0.0,0.0,Qval,0.0,0.0,0.0,0.0,0.0,0.0,Qval,0.0,0.0,0.0,0.0,0.0,0.0,Qval};
      BLA::Matrix<6, 6> Pkup_Rk = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6> SensorData = {0.0,0.0,0.0,0.0,0.0,0.0};
      BLA::Matrix<6> KalmanFilterData = {0.0,0.0,0.0,0.0,0.0,0.0}; 
      BLA::Matrix<6, 6> I6 = {unit,0.0,0.0,0.0,0.0,0.0,0.0,unit,0.0,0.0,0.0,0.0,0.0,0.0,unit,0.0,0.0,0.0,0.0,0.0,0.0,unit,0.0,0.0,0.0,0.0,0.0,0.0,unit,0.0,0.0,0.0,0.0,0.0,0.0,unit};
  
      

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x6E, 0x97, 0x0C};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


ADS myFlexSensor1; //Create object of the ADS class
ADS myFlexSensor2; //Create object of the ADS class

const byte dataReadyPin_1 = 2;
const byte dataReadyPin_2 = 15;

const byte POF_01 = 39;
const byte POF_02 = 36;

int countdownMS;

typedef struct struct_message {
  float NOW_S1_Xflex;
  float NOW_S1_Yflex;
  float NOW_S1_POF;
  float NOW_S2_Xflex;
  float NOW_S2_Yflex;
  float NOW_S2_POF;
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


void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  countdownMS = Watchdog.enable(1000); //watchdog timer

  pinMode(dataReadyPin_1, INPUT);
  pinMode(dataReadyPin_2, INPUT);

  pinMode(POF_01, INPUT);
  pinMode(POF_02, INPUT);

  Wire.begin();

  //Setup first sensor - look for it at the default address of 0x13 = 19
  if (myFlexSensor1.begin(45) == false)
  {
    Serial.println(F("First sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }

  //Setup second sensor - look for it at the I2C address of 45. You should have set this up in example 6
  if (myFlexSensor2.begin(19) == false)
  {
    Serial.println(F("Second sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }

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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  ///////////////
      
       

} // end void setup


float data[6];

int fail_data2 = 0;
int fail_data1 = 0;



void loop()
{

  if (myFlexSensor1.available() == true) {
    
    //Flex1_X = myFlexSensor1.getX();
    //Flex1_Y = myFlexSensor1.getY();
    Flex1_X_lpfilt  = low_pass_param_1*Flex1_X_ant + low_pass_param_2*myFlexSensor1.getX(); // low-pass filter
    Flex1_Y_lpfilt  = low_pass_param_1*Flex1_Y_ant + low_pass_param_2*myFlexSensor1.getY(); // low-pass filter
    
    data[0] = Flex1_X_lpfilt;
    data[1] = Flex1_Y_lpfilt;
    
  } else {
    fail_data1 ++;
  }

  if (myFlexSensor2.available() == true) {
    
    Flex2_X_lpfilt  = low_pass_param_1*Flex2_X_ant + low_pass_param_2*myFlexSensor2.getX(); // low-pass filter
    Flex2_Y_lpfilt  = low_pass_param_1*Flex2_Y_ant + low_pass_param_2*myFlexSensor2.getY(); // low-pass filter
    
    data[2] = Flex2_X_lpfilt;
    data[3] = Flex2_Y_lpfilt;
    
  } else {
    fail_data2 ++;
  }

  if (fail_data1 >= 50 || fail_data2 >= 50) {
    ESP.restart();
  }

  pof1_lpfilt  = low_pass_param_1*pof1_ant + low_pass_param_2*analogRead(POF_01); // low-pass filter
  pof2_lpfilt  = low_pass_param_1*pof2_ant + low_pass_param_2*analogRead(POF_02); // low-pass filter
  
  data[4] = pof1_lpfilt;
  data[5] = pof2_lpfilt;


//////////////// kalman filter

  Xk_update = Xk_prev; // 0
  Pk_update = Pk_prev + Qk; // 1
  
  Pkup_Rk = Pk_update + Rk; // 100 + 1 = 101 pero (0.01)
  auto Pkup_Rk_inv = Pkup_Rk;
  Invert(Pkup_Rk_inv);
  
  Kk = Pk_update * Pkup_Rk_inv;
  
  SensorData = {Flex1_X_lpfilt, Flex1_Y_lpfilt, pof1_lpfilt, Flex2_X_lpfilt, Flex2_Y_lpfilt, pof2_lpfilt};
  Xk_est = Xk_update + ( Kk * (SensorData - Xk_update));
  Pk = (I6 - Kk) * Pk_update;
  
  Xk_prev = Xk_est;
  Pk_prev = Pk;
  

    ii = ii + 1;
   //delay(100);


  //if (digitalRead(dataReadyPin_1) == LOW && digitalRead(dataReadyPin_2) == LOW) {
  if (true)
  {
   
   
    Serial.print(Xk_est(0));
    Serial.print("\t");
    Serial.print(Xk_est(1));
    Serial.print("\t");
    Serial.print(Xk_est(2));
    Serial.print("\t");
    Serial.print(Xk_est(3));
    Serial.print("\t");
    Serial.print(Xk_est(4));
    Serial.print("\t");
    Serial.print(Xk_est(5));
    Serial.print("\t");
    
  

     
   /*  // Set values to send
  myData.NOW_S1_Xflex = data[0];
  myData.NOW_S1_Yflex = data[1];
  myData.NOW_S2_Xflex = data[2];
  myData.NOW_S2_Yflex = data[3];
  myData.NOW_S1_POF   = data[4];
  myData.NOW_S2_POF   = data[5];
  */
  
  myData.NOW_S1_Xflex = Xk_est(0);
  myData.NOW_S1_Yflex = Xk_est(1);
  myData.NOW_S1_POF   = Xk_est(4);
  myData.NOW_S2_Xflex = Xk_est(2);
  myData.NOW_S2_Yflex = Xk_est(3);
  myData.NOW_S2_POF   = Xk_est(5);
  
  
    Serial.println();
    //countdownMS = Watchdog.enable(1000);
  }

  // Create test data

  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    //Serial.println("Error sending the data");
  }
  //delay(2000);
  
  
  countdownMS = Watchdog.enable(1000);
  delay(4);

  //}

}
