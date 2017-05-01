#include <IMU_Kalman.h>

void IMU_Kalman::Setup() {
  Serial.print("Initializing MPU 6050...");
  mpu.initialize();
  Serial.println(" done");

  Serial.print("Testing MPU6050 connection...");
  Serial.println(mpu.testConnection() ? " successful" : " failed");

  delay(100); // Wait for sensor to stabilize

  Serial.print("Calibrating MPU6050...");
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
  mpu.setZAccelOffset(16384 - (accSumZ / 2000));
  mpu.setXGyroOffset(gyroSumX / 2000);
  mpu.setYGyroOffset(gyroSumY / 2000);
  mpu.setZGyroOffset(gyroSumZ / 2000);
  Serial.println(" done");

  Serial.print("Detecting start angles...");
  mpu.getAcceleration(&accX, &accY, &accZ);
  lastSensorDataTime = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double accRoll, accPitch;
  accRoll  = atan2(accY, accZ) * RAD_TO_DEG;
  accPitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(accRoll);
  kalmanY.setAngle(accPitch);

  Serial.println(" done");
}

IMU_Kalman::RollPitchYaw IMU_Kalman::CalculateRollPitchYaw() {
  int16_t gyroX, gyroY, gyroZ;
  int16_t accX, accY, accZ;
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  double deltaSensorDataTime = (double)(micros() - lastSensorDataTime) / 1000000;
  lastSensorDataTime = micros();

  RollPitchYaw rpy;
  double accRoll  = atan2(accY, accZ) * RAD_TO_DEG;
  double gyroRoll = gyroX / 131.0;
  rpy.roll = kalmanX.getAngle(accRoll, gyroRoll, deltaSensorDataTime);

  double accPitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroPitch = gyroY / 131.0;
  rpy.pitch = kalmanY.getAngle(accPitch, gyroPitch, deltaSensorDataTime);

  double gyroYaw = gyroZ / 131.0;
  rpy.yaw = gyroYaw;

  return rpy;
}
