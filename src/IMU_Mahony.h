#include <MPU6050.h>
#include <MahonyAHRS.h>

class IMU_Mahony {
  private:
    MPU6050 mpu;
    Mahony filter;

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
