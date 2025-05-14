#ifndef PID_PID_CONTROLLER_H_
#define PID_PID_CONTROLLER_H_

namespace pid
{

class PIDController
{
public:
  PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0);
  void reset();
  double compute(double error, double dt);

private:
  double kp_, ki_, kd_;
  double integral_, prev_error_;
};

}  // namespace pid

#endif  // PID_PID_CONTROLLER_H_
