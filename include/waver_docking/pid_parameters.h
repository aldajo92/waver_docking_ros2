#ifndef WAVER_DOCKING__PID_PARAMETERS_H_
#define WAVER_DOCKING__PID_PARAMETERS_H_

#include <string>

#include "pid_controller.h"
#include "rclcpp/rclcpp.hpp"

namespace waver_docking {

/**
 * @brief Class to manage PID parameters for a controller.
 *
 * This class handles the retrieval and storage of PID parameters
 * (proportional, integral, derivative) from a ROS2 node.
 */
class PIDParameters {
public:
  PIDParameters(rclcpp::Node& node, const std::string& ns);

  pid_module::PIDController make_controller() const;

  double kp() const;
  double ki() const;
  double kd() const;

private:
  double kp_;
  double ki_;
  double kd_;
};

}  // namespace waver_docking

#endif  // WAVER_DOCKING__PID_PARAMETERS_H_
