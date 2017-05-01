#include <Servo.h>

class Motor {
  private:
    Servo motor;

  public:
    void Setup(int motorPin);
    void SetSpeed(double speed);
};
