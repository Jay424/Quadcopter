/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <PIDWrapper.h>
#include <stdlib.h>

double PIDWrapper::Compute(double newInput, double newSetpoint){
  input = newInput;
  setpoint = newSetpoint;

  if(abs(input - setpoint) < 10) {
    pid.SetTunings(consKp, consKi, consKd);
  }
  else {
    pid.SetTunings(aggKp, aggKi, aggKd);
  }

  pid.Compute();

  return output;
}
