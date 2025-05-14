#include "waver_docking/docking_parameters.h"

namespace waver_docking
{

DockingParameters::DockingParameters(rclcpp::Node & node)
: angular_pid_params(node, "docking.angular_pid"),
  linear_pid_params(node,  "docking.linear_pid")
{
  // Declare parameters with defaults
  node.declare_parameter("docking.x_vel_offset", 0.0);
  node.declare_parameter("docking.max_vel_linear", 0.0);
  node.declare_parameter("docking.max_vel_angular", 0.0);
  node.declare_parameter("docking.x_rotation_vertical_factor", 0.0);
  node.declare_parameter("docking.x_rotation_factor_slow", 0.0);
  node.declare_parameter("docking.x_rotation_factor_fast", 0.0);
  node.declare_parameter("docking.stop_error_angle", 0.0);
  node.declare_parameter("docking.stop_error_linear", 0.0);

  // Retrieve parameters
  node.get_parameter("docking.x_vel_offset", x_vel_offset);
  node.get_parameter("docking.max_vel_linear", max_vel_linear);
  node.get_parameter("docking.max_vel_angular", max_vel_angular);
  node.get_parameter("docking.x_rotation_vertical_factor", x_rotation_vertical_factor);
  node.get_parameter("docking.x_rotation_factor_slow", x_rotation_factor_slow);
  node.get_parameter("docking.x_rotation_factor_fast", x_rotation_factor_fast);
  node.get_parameter("docking.stop_error_angle", stop_error_angle);
  node.get_parameter("docking.stop_error_linear", stop_error_linear);
}

pid_module::PIDController DockingParameters::createAngularController() const
{
  return angular_pid_params.make_controller();
}

pid_module::PIDController DockingParameters::createLinearController() const
{
  return linear_pid_params.make_controller();
}

}  // namespace waver_docking
