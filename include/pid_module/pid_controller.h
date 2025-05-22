#ifndef PID_MODULE_PID_CONTROLLER_H_
#define PID_MODULE_PID_CONTROLLER_H_

namespace pid_module {

class PIDController {
public:
  /// @param kp Proportional gain
  /// @param ki Integral gain
  /// @param kd Derivative gain
  PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0);

  /// Reset integral accumulator and previous error
  void reset();

  /**
   * Compute one step of PID.
   * @param error Current error
   * @param dt    Time step (seconds)
   * @return      Controller output
   */
  double compute(double error, double dt);

private:
  double kp_, ki_, kd_;
  double integral_;
  double prev_error_;
};

}  // namespace pid_module

#endif  // PID_MODULE_PID_CONTROLLER_H_
