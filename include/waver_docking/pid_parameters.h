#ifndef WAVER_DOCKING__PID_PARAMETERS_H_
#define WAVER_DOCKING__PID_PARAMETERS_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pid_controller.h"

namespace waver_docking
{

/**
 * @brief Struct to represent a 2D point.
 *
 * This struct is used to define points in a 2D space, for
 * positions or coordinates.
 */
struct Point
{
  double x;
  double y;
};

/**
 * @brief Class to manage PID parameters for a controller.
 *
 * This class handles the retrieval and storage of PID parameters
 * (proportional, integral, derivative) from a ROS2 node.
 */
class PIDParameters
{
public:
  PIDParameters(rclcpp::Node & node, const std::string & ns);

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
