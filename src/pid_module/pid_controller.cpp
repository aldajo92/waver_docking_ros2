#include "pid_controller.h"

namespace pid_module
{

PIDController::PIDController(double kp, double ki, double kd)
: kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0)
{}

void PIDController::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
}

double PIDController::compute(double error, double dt)
{
  double p = kp_ * error;
  integral_ += error * dt;
  double i = ki_ * integral_;
  double derivative = (dt > 0.0) ? ((error - prev_error_) / dt) : 0.0;
  double d = kd_ * derivative;
  prev_error_ = error;
  return p + i + d;
}

}  // namespace pid_module
