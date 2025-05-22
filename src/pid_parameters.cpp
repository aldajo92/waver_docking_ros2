#include "waver_docking/pid_parameters.h"

namespace waver_docking {

PIDParameters::PIDParameters(rclcpp::Node& node, const std::string& ns)
    : kp_(0.0), ki_(0.0), kd_(0.0) {
  node.declare_parameter(ns + ".kp", 0.0);
  node.declare_parameter(ns + ".ki", 0.0);
  node.declare_parameter(ns + ".kd", 0.0);

  node.get_parameter(ns + ".kp", kp_);
  node.get_parameter(ns + ".ki", ki_);
  node.get_parameter(ns + ".kd", kd_);

  // Print the parameters
  RCLCPP_INFO(node.get_logger(),
              "PID parameters for %s: kp=%.2f, ki=%.2f, kd=%.2f",
              ns.c_str(),
              kp_,
              ki_,
              kd_);
}

pid_module::PIDController PIDParameters::make_controller() const {
  return pid_module::PIDController(kp_, ki_, kd_);
}

double PIDParameters::kp() const {
  return kp_;
}
double PIDParameters::ki() const {
  return ki_;
}
double PIDParameters::kd() const {
  return kd_;
}

}  // namespace waver_docking
