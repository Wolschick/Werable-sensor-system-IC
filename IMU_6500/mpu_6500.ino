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
int16_t ax_A, ay_A, az_A;
int16_t ax_B, ay_B, az_B;
float axS_A, ayS_A, azS_A;
float axS_B, ayS_B, azS_B;

//gyroscope variables
int16_t gx_A, gy_A, gz_A;
int16_t gx_B, gy_B, gz_B;
float gxS_A, gyS_A, gzS_A;
float gxS_B, gyS_B, gzS_B;

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

  //correct scale from accelerometer and gyroscope
  sensy_raw_data(ax_A, ay_A, az_A, gx_A, gy_A, gz_A, &axS_A, &ayS_A, &azS_A, &gxS_A, &gyS_A, &gzS_A);
  sensy_raw_data(ax_B, ay_B, az_B, gx_B, gy_B, gz_B, &axS_B, &ayS_B, &azS_B, &gxS_B, &gyS_B, &gzS_B);

  //calculate angle
  computeAccelAngles(axS_A, ayS_A, azS_A, gxS_A, gyS_A, gzS_A, &pitch_A, &roll_A);
  computeAccelAngles(axS_B, ayS_B, azS_B, gxS_B, gyS_B, gzS_B, &pitch_B, &roll_B);

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
void computeAccelAngles(float ax, float ay, float az, float gx, float gy, float gz, float* pitch, float* roll) {
  float pitchAcc, rollAcc;

  //Calculate angle by accelerometer data
  pitchAcc = atan2f(ax, sqrtf(ay * ay + az * az)) * 180 / M_PI;
  rollAcc = atan2f(ay, sqrtf(ax * ax + az * az)) * 180 / M_PI;

  // integration of gyroscope data
  *pitch += gx * dt * 180 / M_PI;
  *roll += gy * dt * 180 / M_PI;

  //filter
  *pitch = *pitch * alpha + pitchAcc * (1 - alpha);
  *roll = *roll * alpha + rollAcc * (1 - alpha);

}

void sensy_raw_data(float ax, float ay, float az, float gx, float gy, float gz, float *axS, float *ayS, float *azS, float *gxS, float *gyS, float *gzS) {
  // Convert int16 to float and apply scale
  *axS = ax / 16384.0;  // Sensibility of accelerometer +-2g (16384 LSB/g)
  *ayS = ay / 16384.0;
  *azS = az / 16384.0;
  *gxS = gx / 131.0;  // Sensibility of gyroscope +-250 deg/s (131 LSB/(deg/s))
  *gyS = gy / 131.0;
  *gzS = gz / 131.0;

  //Convert gyro of degrees to rad/s
  *gxS *= (M_PI / 180.0);
  *gyS *= (M_PI / 180.0);
  *gzS *= (M_PI / 180.0);
}
