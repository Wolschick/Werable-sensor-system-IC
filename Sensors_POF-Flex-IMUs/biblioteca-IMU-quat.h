
class IMU : public BNO080 {
  public:
    double quaternion[sizedata];
    double *pointer[sizedata];
    void print();
    void get_data();
    double **reciev_data();
    void begin_loop(uint8_t deviceAddress, TwoWire * wirePort = &Wire, uint8_t intPin = 255);

};

void IMU::print() {

  for (int i = 0; i < sizedata ; i++) {
    Serial.print(quaternion[i]);
    Serial.print("\t");
  }
  //Serial.print("\n");
}

void IMU::begin_loop(uint8_t deviceAddress, TwoWire * wirePort, uint8_t intPin) {
  while (begin(deviceAddress, (*wirePort)) == false) {
    Serial.printf("\nIMU %d no detectada", deviceAddress);
    delay(10);
  }

  //calibrateAll(); //calibra a IMU
  enableRotationVector(ms_send);    //We must enable the accel in order to get MEMS readings even if we don't read the reports.

  if (intPin != 255) {
    pinMode(intPin, INPUT);
    //attachInterrupt(intPin, IMU::callbackISR, FALLING);
  }

  Serial.printf("\nIMU %d iniciada\n", deviceAddress);
}

void IMU::get_data() {

  if (!dataAvailable()) return;

  quaternion[0] = getQuatI();
  quaternion[1] = getQuatJ();
  quaternion[2] = getQuatK();
  quaternion[3] = getQuatReal();
  //quaternion[4] = getQuatRadianAccuracy();

  return;
}

double **IMU::reciev_data() {
  
  pointer[0] = &quaternion[0];
  pointer[1] = &quaternion[1];
  pointer[2] = &quaternion[2];
  pointer[3] = &quaternion[3];

  return pointer;
}
