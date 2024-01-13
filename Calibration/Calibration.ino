#include <Wire.h>

#define MPU_ADDRESS 0x68

int n = 2500;  // number of samples used for calibration of IMU
int i = 0;

int16_t AccelX = 0, AccelY = 0, AccelZ = 0;  // variables to store accelerometer data
int16_t GyroX = 0, GyroY = 0, GyroZ = 0;     // variables to store gyroscope data

// variasbles to calibrate accelerometer and gyroscope
float AccelX_offset = 0, AccelY_offset = 0, AccelZ_offset = 0;
float GyroX_offset = 0, GyroY_offset = 0, GyroZ_offset = 0;

// Variables to calculate accelerometer and gyroscope error
float sum_accelX = 0, sum_accelY = 0, sum_accelZ = 0;
float sum_GyroX = 0, sum_GyroY = 0, sum_GyroZ = 0;

void setup() {


  Wire.begin();                         //Begin i2c communication
  Wire.beginTransmission(MPU_ADDRESS);  //Begin transmission with MPU6050
  Wire.write(0x6B);
  Wire.write(0x00);  //Reset MPU
  Wire.endTransmission(true);

  //Configure Accelerometer Maximum range
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x00);  // maximum range (+/-)2g
  Wire.endTransmission(true);

  //Configure Gyroscope Maximum range
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08);  //Maximum range (+/-) 500 deg/sec
  Wire.endTransmission(true);

  delay(100);

  Serial.begin(19200);

  Serial.println("Starting Calibration");
  delay(5000);  
  
  CalibrateIMU();

  Serial.print("Number of samples taken: ");
  Serial.print(n);
  Serial.print('\n');

  Serial.print('\n');
  Serial.print("Accelerometer Offsets:");
  Serial.print('\n');
  Serial.print("AccelX_offset = ");
  Serial.print(AccelX_offset,4);
  Serial.print('\t');
  Serial.print("AccelY_offset = ");
  Serial.print(AccelY_offset,4);
  Serial.print('\t');
  Serial.print("AccelZ_offset = ");
  Serial.print(AccelZ_offset,4);
  Serial.print('\n');

  Serial.print('\n');
  Serial.print("Gyroscope Offsets:");
  Serial.print('\n');
  Serial.print("GyroX_offset = ");
  Serial.print(GyroX_offset,4);
  Serial.print('\t');
  Serial.print("GyroY_offset = ");
  Serial.print(GyroY_offset,4);
  Serial.print('\t');
  Serial.print("GyroZ_offset = ");
  Serial.print(GyroZ_offset,4);
  Serial.print('\t');
}

void loop() {
}

void CalibrateIMU(void) {
  while (i < n) {
    // Read Accelerometer Data
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);

    //read from 6 consectuive registers, where each axis data is stored in 2 registers (OUT_H and OUT_L)

    float(AccelX) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelX_offset;
    float(AccelY) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelY_offset;
    float(AccelZ) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelZ_offset;

    // Read Gyroscope Data
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);

    //read from 6 consectuive registers, where each axis data is stored in 2 registers (OUT_H and OUT_L)

    float(GyroX) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroX_offset;
    float(GyroY) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroY_offset;
    float(GyroZ) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroZ_offset;

    sum_accelX = sum_accelX + AccelX;
    sum_accelY = sum_accelY + AccelY;
    sum_accelZ = sum_accelZ + AccelZ;

    sum_GyroX = sum_GyroX + GyroX;
    sum_GyroY = sum_GyroY + GyroY;
    sum_GyroZ = sum_GyroZ + GyroZ;

    //Calculate error in each axis of accelerometer
    AccelX_offset = sum_accelX / n;              //set point is 0
    AccelY_offset = sum_accelY / n;              //set point is 0
    AccelZ_offset = (sum_accelZ + n * (1)) / n;  //set point is -1

    GyroX_offset = sum_GyroX / n;  //set point is 0
    GyroY_offset = sum_GyroY / n;  //set point is 0
    GyroZ_offset = sum_GyroZ / n;  //set point is 0

    i++;
  }
}
