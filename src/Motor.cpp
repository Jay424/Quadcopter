/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Motor.h>

void Motor::Setup(int motorPin) {
  Serial.print("Setting up motor on pin ");
  Serial.print(motorPin);
  Serial.print(" ... ");
  motor.attach(motorPin);
  Serial.println("done");
}

void Motor::SetSpeed(double speed) {
  // ensure max/min values
  speed = max(speed, 1100);
  speed = min(speed, 2000);

  motor.writeMicroseconds(speed);
}
