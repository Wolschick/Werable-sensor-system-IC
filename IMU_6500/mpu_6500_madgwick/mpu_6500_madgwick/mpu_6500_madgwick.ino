#include <Wire.h>
#include <MadgwickAHRS.h>

// IMU config
const int MPU_ADDR_A = 0x68;  // I2C address of MPU6050
const int MPU_ADDR_B = 0x69;

int16_t acelX_A, acelY_A, acelZ_A, gyroX_A, gyroY_A, gyroZ_A;
int16_t acelX_B, acelY_B, acelZ_B, gyroX_B, gyroY_B, gyroZ_B;

// Create Madgwick filter
Madgwick filterA;
Madgwick filterB;

float roll_A, pitch_A, yaw_A;
float roll_B, pitch_B, yaw_B;
float beta = 0.2;  // Convergence rate of filter

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init MPU6050_A
  Serial.print("Init MPU A");
  Wire.beginTransmission(MPU_ADDR_A);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Init MPU6050_B
  Serial.print("Init MPU B");
  Wire.beginTransmission(MPU_ADDR_B);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Config Madgwick filter
  filterA.begin(100);  // set attualization rate
  filterB.begin(100);
}

void loop() {

  readIMU(MPU_ADDR_A, &acelX_A, &acelY_A, &acelZ_A, &gyroX_A, &gyroY_A, &gyroZ_A);
  readIMU(MPU_ADDR_B, &acelX_B, &acelY_B, &acelZ_B, &gyroX_B, &gyroY_B, &gyroZ_B);

  filterA.updateIMU(gyroX_A / 131.0, gyroY_A / 131.0, gyroZ_A / 131.0, acelX_A / 16384.0, acelY_A / 16384.0, acelZ_A / 16384.0);
  filterB.updateIMU(gyroX_B / 131.0, gyroY_B / 131.0, gyroZ_B / 131.0, acelX_B / 16384.0, acelY_B / 16384.0, acelZ_B / 16384.0);

  // Obtain Euler angles
  roll_A = filterA.getRoll();
  pitch_A = filterA.getPitch();
  yaw_A = filterA.getYaw();
  roll_B = filterB.getRoll();
  pitch_B = filterB.getPitch();
  yaw_B = filterB.getYaw();

  Serial.print(roll_A);
  Serial.print("\t");
  Serial.print(pitch_A);
  Serial.print("\t");
  Serial.print(yaw_A);
  Serial.print("\t");
  Serial.print(roll_B);
  Serial.print("\t");
  Serial.print(pitch_B);
  Serial.print("\t");
  Serial.println(yaw_B);

  delay(10);
}

void readIMU(int MPU_ADDR, int16_t *acelX, int16_t *acelY, int16_t *acelZ, int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Começa a ler do registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // Solicita 14 registros, que contêm acel e gyro

  *acelX = Wire.read() << 8 | Wire.read();
  *acelY = Wire.read() << 8 | Wire.read();
  *acelZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();  // Ignora TEMP_OUT
  *gyroX = Wire.read() << 8 | Wire.read();
  *gyroY = Wire.read() << 8 | Wire.read();
  *gyroZ = Wire.read() << 8 | Wire.read();
}
