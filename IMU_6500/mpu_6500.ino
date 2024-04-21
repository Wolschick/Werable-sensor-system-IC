#include <Wire.h>
#include <math.h>
#include <MPU6050.h>

MPU6050 IMU_A(0x68);
MPU6050 IMU_B(0x69);

const float alpha = 0.95;  //fusion factor
const float dt = 0.01;     // interval time (10ms)

//angle variables
float pitch_A = 0, roll_A = 0;
float pitch_B = 0, roll_B = 0;

// acelerometer variables
int16_t ax_A;
int16_t ay_A;
int16_t az_A;
int16_t ax_B;
int16_t ay_B;
int16_t az_B;

//gyroscope variables
int16_t gx_A;
int16_t gy_A;
int16_t gz_A;
int16_t gx_B;
int16_t gy_B;
int16_t gz_B;

void setup() {
  Serial.begin(115200);
  Wire.begin();        // Initialize I2C communication
  IMU_A.initialize();  // Initialize IMU_A
  IMU_B.initialize();  // Initialize IMU_B

  if (IMU_A.testConnection()) {
    Serial.println("Connection with IMU_A OK");
  } else {
    Serial.println("Connection with IMU_A ERROR");
    // while(true);
  }
  Serial.print(IMU_B.testConnection());
  if (IMU_B.testConnection()) {
    Serial.println("Connection with IMU_B OK");
  } else {
    Serial.println("Connection with IMU_B ERROR");
    //while (true);
  }
}

void loop() {
  // Lê os valores de aceleração e giroscópio

  IMU_A.getMotion6(&ax_A, &ay_A, &az_A, &gx_A, &gy_A, &gz_A);
  IMU_B.getMotion6(&ax_B, &ay_B, &az_B, &gx_B, &gy_B, &gz_B);

  updateSensorReadings(ax_A, ay_A, az_A, gx_A, gy_A, gz_A, &pitch_A, &roll_A);
  updateSensorReadings(ax_B, ay_B, az_B, gx_B, gy_B, gz_B, &pitch_B, &roll_B);
  Serial.print(pitch_A);
  Serial.print("\t");
  Serial.print(roll_A);
  Serial.print("\t");
  Serial.print(pitch_B);
  Serial.print("\t");
  Serial.println(roll_B);

  delay(10);  // Espera um segundo entre leituras
}

// calculate angle values by accelerometer data
void computeAccelAngles(float ax, float ay, float az, float **pitch, float **roll) {
  float pitchAcc, rollAcc;
  pitchAcc = atan2f(ax, sqrtf(ay * ay + az * az)) * 180 / M_PI;
  rollAcc = atan2f(ay, sqrtf(ax * ax + az * az)) * 180 / M_PI;
  **pitch = **pitch * alpha + pitchAcc * (1 - alpha);
  **roll = **roll * alpha + rollAcc * (1 - alpha);
}

// update values of angle by gyroscope data
void updateSensorReadings(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll) {
  // Primeiro, atualize com giroscópio
  *pitch += gx * dt;
  *roll += gy * dt;

  // Agora, aplique o filtro complementar com acelerômetro
  computeAccelAngles(ax, ay, az, &pitch, &roll);
}
