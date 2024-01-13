#include <Wire.h>
#include <math.h>

#define MPU_ADDRESS 0x68
#define sign(x) (((x) > 0) - ((x) < 0))  //sign function, returns 1 if x is +ve and -1 if x is -ve

// Variables for motor driver
#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 11

int16_t AccelX = 0, AccelY = 0, AccelZ = 0;  // variables to store accelerometer data
int16_t GyroX = 0, GyroY = 0, GyroZ = 0;     // variables to store gyroscope data

// offsets taken after calibration
float AccelX_offset = 0.0575, AccelY_offset = 0.0273, AccelZ_offset = -0.3565;
float GyroX_offset = -6.7757, GyroY_offset = -0.2757, GyroZ_offset = 0.2661;

float pitchAngle = 0;
float pitchAngle_accel = 0, pitchAngle_accel_filtered = 0;
float pitchAngle_gyro = 0, pitchAngle_gyro_filtered = 0;

float dTgyro = 0, currentTime = 0, lastTime = 0;

float fusion_weight = 0.4;

// Variables for PID
volatile float err = 0, last_err = 0, err_sum = 0, err_dot = 0;  // error terms
volatile float PTerm = 0, ITerm = 0, DTerm = 0;
float Kp = 5, ki = 0.5, kd = 0.125;  // PID gains
float desiredPitchAngle = 0;
volatile float pwm = 0;  //PID output

// Timer1 Variables
float dt = 0.1;
int prescaler = 64;
float preLoadValue = 65535 - (16 * pow(10, 6) * dt) / prescaler;
volatile bool timerFlag = false;

void setup() {

  noInterrupts();
  TCCR1A = 0;  //Timer1 control register A
  TCCR1B = 0;  //Timer1 control register B

  //preload Timer1
  TCNT1 = preLoadValue;

  //select prescaler
  TCCR1B |= (1 << CS10) | (1 << CS11);
  //enable Timer1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);
  interrupts();

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

  Serial.begin(19200);
}

void loop() {
  //Read Accelerometer Data
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  //read from 6 consectuive registers, where each axis data is stored in 2 registers (OUT_H and OUT_L)
  float(AccelX) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelX_offset;
  float(AccelY) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelY_offset;
  float(AccelZ) = ((Wire.read() << 8 | Wire.read()) / 16384.0) - AccelZ_offset;

  // Read Gyroscope Data
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x45);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDRESS, 2, true);

  // read GyroY data only as it is the axis of rotation
  float(GyroY) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroY_offset;

  // float(GyroX) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroX_offset;
  // float(GyroZ) = ((Wire.read() << 8 | Wire.read()) / 65.5) - GyroZ_offset;

  //Check sensor readings

  // Serial.print("AccelX = "); Serial.print(AccelX,4); Serial.print("   ");
  // Serial.print("AccelY = "); Serial.print(AccelY,4); Serial.print("   ");
  // Serial.print("AccelZ = "); Serial.print(AccelZ,4); Serial.print("      ");

  // Serial.print("GyroX = "); Serial.print(GyroX,4); Serial.print("   ");
  // Serial.print("GyroY = "); Serial.print(GyroY,4); Serial.print("   ");
  // Serial.print("GyroZ = "); Serial.println(GyroZ,4);
  // delay(500);

  // Calculate Pitch angle (axis of rotation) using Accelerometer
  pitchAngle_accel = atan(AccelX / AccelZ) * 180 / 3.141;
  pitchAngle_accel_filtered = exp_moving_avg(pitchAngle_accel_filtered, pitchAngle_accel, 0.7);

  // Calculate Pitch angle using Gyroscope through integration
  currentTime = millis();
  dTgyro = (currentTime - lastTime) / 1000;
  pitchAngle_gyro = (pitchAngle_gyro + GyroY * dTgyro);
  lastTime = currentTime;
  // pitchAngle_gyro_filtered = exp_moving_avg(pitchAngle_gyro_filtered, pitchAngle_gyro, 0.5);

  // Fuse readings of pitch angle from Gyrscope and Accelerometer using weighted fusion
  pitchAngle = fusion_weight * pitchAngle_accel_filtered+ (1 - fusion_weight) * pitchAngle_gyro* (-1);  // *(-1) due to opposite signs between pitchAngle_accel and pitchAngle_gyro

  //Check pitch angles
  // Serial.print(90); Serial.print(" "); Serial.print(-90);Serial.print(" ");
  // Serial.print("Pitch_accel ");Serial.print(pitchAngle_accel); Serial.print(",");
  // Serial.print("Pitch_accel_filtered "); Serial.print(pitchAngle_accel_filtered);Serial.print(",");
  // Serial.print("Pitch_gyro ");  Serial.print(pitchAngle_gyro);Serial.print(",");
  // Serial.print("Pitch_gyro_filtered "); Serial.print(pitchAngle_gyro_filtered);Serial.print(",");
  // Serial.println(pitchAngle);
  
  if (timerFlag) {
    timerFlag = false;

    err = desiredPitchAngle - pitchAngle;
    err_dot = (err - last_err) / dt;
    err_sum = err_sum + err * dt;
    last_err = err;

    PTerm = Kp * err;
    ITerm = ki * err_sum;
    DTerm = kd * err_dot;

    if (ITerm >= 255 || ITerm <= -255) {
      ITerm = 255 * sign(ITerm);
    }
    pwm = PTerm + ITerm + DTerm;
    if (pwm >= 255 || pwm <= -255) {
      pwm = 255 * sign(pwm);
    }
    
    if(abs(pitchAngle)<8.5){
      pwm = 0;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }          
    if (pwm >= 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, abs(pwm));
    analogWrite(ENB, abs(pwm));    
  }
}

float exp_moving_avg(float lastAvg, float newReading, float alpha) {
  float newAvg = alpha * newReading + (1 - alpha) * lastAvg;
  return newAvg;
}

ISR(TIMER1_OVF_vect) {
  timerFlag = true;
  TCNT1 = preLoadValue;
}
