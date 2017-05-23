/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

class MPU9250
{
  public:
    struct RollPitchYaw
    {
      double roll;
      double pitch;
      double yaw;
    };

    void Setup();
    bool CalculateRollPitchYaw(RollPitchYaw *rpy);
};
