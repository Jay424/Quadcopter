/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Servo.h>

class Motor
{
  private:
    Servo motor;
  public:
    void Setup(int motorPin);
    void SetSpeed(double speed);
};
