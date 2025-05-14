#ifndef WAVER_DOCKING__PID_PARAMETERS_HPP_
#define WAVER_DOCKING__PID_PARAMETERS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pid_controller.hpp"  // Include the PIDController header

namespace waver_docking
{

class PIDParameters
{
public:
  PIDParameters(rclcpp::Node & node, const std::string & ns);

  pid::PIDController make_controller() const;

  double kp() const;
  double ki() const;
  double kd() const;

private:
  double kp_;
  double ki_;
  double kd_;
};

}  // namespace waver_docking

#endif  // WAVER_DOCKING__PID_PARAMETERS_HPP_