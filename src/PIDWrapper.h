/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
#include <PID_v1.h>

class PIDWrapper
{
private:
  double consKp, consKi, consKd;
  double aggKp, aggKi, aggKd;
  double input, output, setpoint;
  PID pid;
public:
  PIDWrapper(double consKp, double consKi, double consKd, double aggKp, double aggKi, double aggKd, double minOutput, double maxOutput)
    : pid(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT), consKp(consKp), consKi(consKi), consKd(consKd), aggKp(aggKp), aggKi(aggKi), aggKd(aggKd)
  {
      pid.SetMode(AUTOMATIC);
      pid.SetOutputLimits(minOutput, maxOutput);
      pid.SetSampleTime(250);
  }
  double Compute(double newInput, double newSetpoint);
};
