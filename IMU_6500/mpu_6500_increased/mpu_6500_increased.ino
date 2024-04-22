#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 IMU_A(Wire, 0x68);
MPU6050 IMU_B(Wire, 0x69);

bool print_raw_angle = false;
bool print_relative_angle = true;

// Anlge variables
float roll_A, pitch_A, yaw_A;
float roll_B, pitch_B, yaw_B;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Init I2C communication

  IMU_A.begin();                // Init MPU 6500
  IMU_B.begin();                // Init MPU 6500
  IMU_A.calcGyroOffsets(true);  // Calculate offsets
  IMU_B.calcGyroOffsets(true);  // Calculate offsets
}

void loop() {
  IMU_A.update();  // Att sensor data
  IMU_B.update();  // Att sensor data

  // obtain euler angles
  roll_A = IMU_A.getAngleX();
  pitch_A = IMU_A.getAngleY();
  yaw_A = IMU_A.getAngleZ();

  roll_B = IMU_B.getAngleX();
  pitch_B = IMU_B.getAngleY();
  yaw_B = IMU_B.getAngleZ();

  if (print_raw_angle) {
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
  }

  if (print_relative_angle) {
    Serial.print(roll_A - roll_B);
    Serial.print("\t");
    Serial.print(pitch_A - pitch_B);
    Serial.print("\t");
    Serial.println(yaw_A - yaw_B);
  }

  delay(10);  // Delay entre leituras
}
