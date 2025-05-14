#ifndef WAVER_DOCKING__DOCKING_PARAMETERS_H_
#define WAVER_DOCKING__DOCKING_PARAMETERS_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pid_parameters.h"

namespace waver_docking
{

class DockingParameters
{
public:
  /**
   * @brief Declare and load all docking-related parameters
   * @param node ROS2 node handle
   */
  explicit DockingParameters(rclcpp::Node & node);

  /**
   * @brief Create PIDController for angular control
   */
  pid_module::PIDController createAngularController() const;

  /**
   * @brief Create PIDController for linear control
   */
  pid_module::PIDController createLinearController() const;

  // Raw parameter values
  double x_vel_offset;
  double max_vel_linear;
  double max_vel_angular;
  double x_rotation_vertical_factor;
  double x_rotation_factor_slow;
  double x_rotation_factor_fast;
  double stop_error_angle;
  double stop_error_linear;

  std::string fiducial_positions_file;

private:
    waver_docking::PIDParameters angular_pid_params;
    waver_docking::PIDParameters linear_pid_params;
};

}  // namespace waver_docking

#endif  // WAVER_DOCKING__DOCKING_PARAMETERS_H_