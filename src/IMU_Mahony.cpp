#include <IMU_Mahony.h>

void IMU_Mahony::setup() {
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
}

IMU_Mahony::RollPitchYaw IMU_Mahony::CalculateRollPitchYaw() {
  int16_t gyroX, gyroY, gyroZ;
  int16_t accX, accY, accZ;
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

  RollPitchYaw rpy;
  rpy.roll = filter.getRoll();
  rpy.pitch = filter.getPitch();
  rpy.yaw = filter.getYaw();

  return rpy;
}
