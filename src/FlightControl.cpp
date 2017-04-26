/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

MPU6050 mpu;
double sensorRoll, sensorPitch, sensorYaw;

double setpointRoll, setpointPitch, setpointYaw, setpointThrottle;

double roll, pitch, yaw, throttle;
const double consKp = 0.75, consKi = 0.4, consKd = 0.08;
const double aggKp = 0.875, aggKi = 0.5, aggKd = 0.1;
const double consYawKp = 0, consYawKi = 0, consYawKd = 0.2;
const double aggYawKp = 0, aggYawKi = 0, aggYawKd = 0.5;
PID rollPID(&sensorRoll, &roll, &setpointRoll, consKp, consKi, consKd, DIRECT);
PID pitchPID(&sensorPitch, &pitch, &setpointPitch, consKp, consKi, consKd, DIRECT);
PID yawPID(&sensorYaw, &yaw, &setpointYaw, consYawKp, consYawKi, consYawKd, DIRECT);

double esc1, esc2, esc3, esc4;

const int ch1 = 2;
const int ch2 = 3;
const int ch3 = 4;
const int ch4 = 5;
Servo motor1, motor2, motor3, motor4;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.print("Initializing MPU6050 ...");
  mpu.initialize();
  Serial.println(" done");

  Serial.print("Testing MPU6050 connection...");
  Serial.println(mpu.testConnection() ? " successful" : " failed");

  delay(100); // Wait for sensor to stabilize

  Serial.print("Calibrating MPU6050 ...");
  double accSumX = 0, accSumY = 0, accSumZ = 0;
  double gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  int16_t gyroX, gyroY, gyroZ;
  int16_t accX, accY, accZ;
  for (int i = 0; i < 2000 ; i++){
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    accSumX += accX;
    accSumY += accY;
    accSumZ += accZ;
    gyroSumX += gyroX;
    gyroSumY += gyroY;
    gyroSumZ += gyroZ;
    delay(3);
  }

  mpu.setXAccelOffset(accSumX / 2000);
  mpu.setYAccelOffset(accSumY / 2000);
  // z should be 1g
  mpu.setZAccelOffset(16384 - (accSumZ / 2000));
  mpu.setXGyroOffset(gyroSumX / 2000);
  mpu.setYGyroOffset(gyroSumY / 2000);
  mpu.setZGyroOffset(gyroSumZ / 2000);
  Serial.println(" done");

  Serial.println("Initializing DMP...");
  mpu.dmpInitialize();
  Serial.println(" done");

  Serial.print("Enabling DMP...");
  mpu.setDMPEnabled(true);
  Serial.println(" done");

  Serial.print("Setting up motors ...");
  motor1.attach(ch1);
  motor2.attach(ch2);
  motor3.attach(ch3);
  motor4.attach(ch4);
  Serial.println("done");
}

void loop()
{
  uint16_t fifoCount = mpu.getFIFOCount();
  if(fifoCount < mpu.dmpGetFIFOPacketSize())
  {
    delay(1);
    return;
  }

  // check for overflow
  if (fifoCount >= 1024)
  {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println("FIFO overflow!");
  }
  else
  {
      // get sensor yaw pitch and roll
      uint8_t fifoBuffer[64];
      Quaternion q;
      VectorFloat gravity;
      float yawPitchRollSensorData[3];
      mpu.getFIFOBytes(fifoBuffer, mpu.dmpGetFIFOPacketSize());
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(yawPitchRollSensorData, &q, &gravity);
      sensorYaw = yawPitchRollSensorData[0] * RAD_TO_DEG;
      sensorPitch = yawPitchRollSensorData[1] * RAD_TO_DEG;
      sensorRoll = yawPitchRollSensorData[2] * RAD_TO_DEG;

      // receive remote control set points
      setpointRoll = 0;
      setpointPitch = 0;
      setpointYaw = 0;
      setpointThrottle = 20;

      // calculate pids
      if(abs(sensorRoll - setpointRoll) < 10)
      {
        rollPID.SetTunings(consKp, consKi, consKd);
      }
      else
      {
        rollPID.SetTunings(aggKp, aggKi, aggKd);
      }

      rollPID.Compute();

      if(abs(sensorPitch - setpointPitch) < 10)
      {
        pitchPID.SetTunings(consKp, consKi, consKd);
      }
      else
      {
        pitchPID.SetTunings(aggKp, aggKi, aggKd);
      }

      pitchPID.Compute();

      if(abs(sensorYaw - setpointYaw) < 10)
      {
        yawPID.SetTunings(consYawKp, consYawKi, consYawKd);
      }
      else
      {
        yawPID.SetTunings(aggYawKp, aggYawKi, aggYawKd);
      }

      yawPID.Compute();

      // calculate escs
      throttle = map(setpointThrottle, 0, 100, 1000, 2000);
      throttle = min(throttle, 1800);
      esc1 = throttle - pitch + roll - yaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc2 = throttle + pitch + roll + yaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc3 = throttle + pitch - roll - yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc4 = throttle - pitch - roll + yaw; //Calculate the pulse for esc 4 (front-left - CW)

      /*if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
        esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
      }*/

      esc1 = max(esc1, 1100);
      esc2 = max(esc2, 1100);
      esc3 = max(esc3, 1100);
      esc4 = max(esc4, 1100);
      esc1 = min(esc1, 2000);
      esc2 = min(esc2, 2000);
      esc3 = min(esc3, 2000);
      esc4 = min(esc4, 2000);

      // set motor speeds
      motor1.writeMicroseconds(esc1);
      motor2.writeMicroseconds(esc2);
      motor3.writeMicroseconds(esc3);
      motor4.writeMicroseconds(esc4);
  }
}
