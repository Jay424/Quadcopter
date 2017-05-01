#include <PID_v1.h>

class PIDWrapper{
private:
  double consKp, consKi, consKd;
  double aggKp, aggKi, aggKd;
  double input, output, setpoint;
  PID pid;

public:
  PIDWrapper(double consKp, double consKi, double consKd, double aggKp, double aggKi, double aggKd) : pid(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT), consKp(consKp), consKi(consKi), consKd(consKd), aggKp(aggKp), aggKi(aggKi), aggKd(aggKd) {}
  double Compute(double newInput, double newSetpoint);
};
