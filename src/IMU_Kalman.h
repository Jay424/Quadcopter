/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <MPU6050.h>
#include <Kalman.h>
#include <math.h>

class IMU_Kalman {
  private:
    MPU6050 mpu;
    Kalman kalmanX, kalmanY;
    uint32_t lastSensorDataTime;
  public:
    struct RollPitchYaw
    {
      float roll;
      float pitch;
      float yaw;
    };

    void Setup();
    RollPitchYaw CalculateRollPitchYaw();
};
