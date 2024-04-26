#include "Wire.h"

int MPU_addr_A = 0x68;
int MPU_addr_B = 0x69;

//float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float A_calA[6] = { 0.0, 0.0, 0.0, 1.000, 1.000, 1.000 };  // 0..2 offset xyz, 3..5 scale xyz
float A_calB[6] = { 0.0, 0.0, 0.0, 1.000, 1.000, 1.000 };  // 0..2 offset xyz, 3..5 scale xyz

float G_offA[3] = { 0, 0, 0 };  //raw offsets, determined for gyro at rest
float G_offB[3] = { 0, 0, 0 };  //raw offsets, determined for gyro at rest

bool flag_calib_gyro = true;

#define gscale ((250. / 32768.0) * (PI / 180.0))  //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float qA[4] = { 1.0, 0.0, 0.0, 0.0 };
float qB[4] = { 1.0, 0.0, 0.0, 0.0 };

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

// with MPU-6050, some instability observed at Kp=100 Now set to 30.

// char s[60]; //snprintf buffer, if needed

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0;  //millis() timers

// print interval
unsigned long print_ms = 10;   //print angles every "print_ms" milliseconds
float yaw_A, pitch_A, roll_A;  //Euler angle output
float yaw_B, pitch_B, roll_B;  //Euler angle output

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("starting MPU 6500 A ...");

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr_A);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.println("starting MPU 6500 B ...");

  Wire.beginTransmission(MPU_addr_B);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.println("Finish start");
}

// AHRS loop

void loop() {
  static float deltat = 0;                 //loop time in seconds
  static unsigned long now = 0, last = 0;  //micros() timers
  static long gsum_A[3] = { 0 };
  static long gsum_B[3] = { 0 };
  static unsigned int cont_calib_gyro = 0;

  //raw data
  int16_t ax_A, ay_A, az_A;
  int16_t gx_A, gy_A, gz_A;
  int16_t Tmp_A;  //temperature
  int16_t ax_B, ay_B, az_B;
  int16_t gx_B, gy_B, gz_B;
  int16_t Tmp_B;  //temperature

  //scaled data as vector
  float Axyz_A[3];
  float Gxyz_A[3];
  float Axyz_B[3];
  float Gxyz_B[3];

  update_acc_gyr(&MPU_addr_A, &ax_A, &ay_A, &az_A, &gx_A, &gy_A, &gz_A, &Tmp_A);
  update_acc_gyr(&MPU_addr_B, &ax_B, &ay_B, &az_B, &gx_B, &gy_B, &gz_B, &Tmp_B);

  //calib gyroscope
  if (flag_calib_gyro) {
    cont_calib_gyro++;
    calib_gyro(&gx_A, &gy_A, &gz_A, G_offA, gsum_A, cont_calib_gyro);
    calib_gyro(&gx_B, &gy_B, &gz_B, G_offB, gsum_B, cont_calib_gyro);
  }

  //convert to float and add to an array
  Axyz_A[0] = (float)ax_A;
  Axyz_A[1] = (float)ay_A;
  Axyz_A[2] = (float)az_A;
  Axyz_B[0] = (float)ax_B;
  Axyz_B[1] = (float)ay_B;
  Axyz_B[2] = (float)az_B;

  //apply offsets and scale factors from Magneto
  for (int i = 0; i < 3; i++) {
    Axyz_A[i] = (Axyz_A[i] - A_calA[i]) * A_calA[i + 3];
    Axyz_B[i] = (Axyz_B[i] - A_calB[i]) * A_calB[i + 3];
  }

  //250 LSB(d/s) default to radians/s
  Gxyz_A[0] = ((float)gx_A - G_offA[0]) * gscale;
  Gxyz_A[1] = ((float)gy_A - G_offA[1]) * gscale;
  Gxyz_A[2] = ((float)gz_A - G_offA[2]) * gscale;
  Gxyz_B[0] = ((float)gx_B - G_offB[0]) * gscale;
  Gxyz_B[1] = ((float)gy_B - G_offB[1]) * gscale;
  Gxyz_B[2] = ((float)gz_B - G_offB[2]) * gscale;

  //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
  //  Serial.println(s);

  now = micros();
  deltat = (now - last) * 1.0e-6;  //seconds since last update
  last = now;

  Mahony_update(Axyz_A, Gxyz_A, qA, deltat);
  Mahony_update(Axyz_B, Gxyz_B, qB, deltat);

  calc_euler_angles(&roll_A, &pitch_A, &yaw_A, qA);
  calc_euler_angles(&roll_B, &pitch_B, &yaw_B, qB);

  now_ms = millis();  //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    // print angles for serial plotter...
    //  Serial.print("ypr ");
    Serial.print(yaw_A, 0);
    Serial.print("\t");
    Serial.print(pitch_A, 0);
    Serial.print("\t");
    Serial.print(roll_A, 0);
    Serial.print("\t");
    Serial.print(yaw_B, 0);
    Serial.print("\t");
    Serial.print(pitch_B, 0);
    Serial.print("\t");
    Serial.println(roll_B, 0);
  }
}

void update_acc_gyr(int *MPU_addr, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *Tmp) {
  Wire.beginTransmission(*MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(*MPU_addr, 14);  // request a total of 14 registers
  int t = Wire.read() << 8;
  *ax = t | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  *ay = t | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  *az = t | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  *Tmp = t | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  *gx = t | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  *gy = t | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  *gz = t | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void calib_gyro(int16_t *gx, int16_t *gy, int16_t *gz, float *G_off, long int *gsum, unsigned int i) {
  gsum[0] += *gx;
  gsum[1] += *gy;
  gsum[2] += *gz;
  if (i == 500) {
    flag_calib_gyro = false;  //turn off calibration and print results

    for (char k = 0; k < 3; k++) G_off[k] = ((float)gsum[k]) / 500.0;
  }
}

void Mahony_update(float Axyz[3], float Gxyz[3], float *q, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  float ax = Axyz[0], ay = Axyz[1], az = Axyz[2];
  float gx = Gxyz[0], gy = Gxyz[1], gz = Gxyz[2];
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];  //to normalize these terms, multiply each by 2.0
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;  // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

void calc_euler_angles(float *roll, float *pitch, float *yaw, float *q) {
  *roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  *pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
  *yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
  // to degrees
  *yaw *= 180.0 / PI;
  if (*yaw < 0) *yaw += 360.0;  //compass circle
  *pitch *= 180.0 / PI;
  *roll *= 180.0 / PI;
}
