#ifndef WAVER_DOCKING__PID_PARAMETERS_HPP_
#define WAVER_DOCKING__PID_PARAMETERS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pid_controller.h"

namespace waver_docking
{

class PIDParameters
{
public:
  PIDParameters(rclcpp::Node & node, const std::string & ns)
  : kp_(0.0), ki_(0.0), kd_(0.0)
  {
    node.declare_parameter(ns + ".kp", 0.0);
    node.declare_parameter(ns + ".ki", 0.0);
    node.declare_parameter(ns + ".kd", 0.0);

    node.get_parameter(ns + ".kp", kp_);
    node.get_parameter(ns + ".ki", ki_);
    node.get_parameter(ns + ".kd", kd_);
  }

  pid_module::PIDController make_controller() const
  {
    return pid_module::PIDController(kp_, ki_, kd_);
  }

  double kp() const { return kp_; }
  double ki() const { return ki_; }
  double kd() const { return kd_; }

private:
  double kp_;
  double ki_;
  double kd_;
};

}  // namespace waver_docking

#endif  // WAVER_DOCKING__PID_PARAMETERS_HPP_