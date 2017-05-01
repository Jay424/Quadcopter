#include <Motor.h>

void Motor::Setup(int motorPin) {
  motor.attach(motorPin);
}

void Motor::SetSpeed(double speed) {
  // ensure max/min values
  speed = max(speed, 1100);
  speed = min(speed, 2000);

  motor.writeMicroseconds(speed);
}
