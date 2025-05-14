#include "rclcpp/rclcpp.hpp"
#include "waver_docking/docking_parameters.h"
#include "pid_module/pid_controller.h"

class DockingNode : public rclcpp::Node
{
public:
  DockingNode()
  : Node("docking_node"),
    docking_parameters_(*this),
    angular_pid_(docking_parameters_.createAngularController()),
    linear_pid_(docking_parameters_.createLinearController())
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DockingNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Docking Node has been started.");

    // Print loaded parameters
    RCLCPP_INFO(this->get_logger(), "x_vel_offset: %f", docking_parameters_.x_vel_offset);
    RCLCPP_INFO(this->get_logger(), "max_vel_linear: %f", docking_parameters_.max_vel_linear);
    RCLCPP_INFO(this->get_logger(), "max_vel_angular: %f", docking_parameters_.max_vel_angular);
    RCLCPP_INFO(this->get_logger(), "x_rotation_vertical_factor: %f", docking_parameters_.x_rotation_vertical_factor);
    RCLCPP_INFO(this->get_logger(), "x_rotation_factor_slow: %f", docking_parameters_.x_rotation_factor_slow);
    RCLCPP_INFO(this->get_logger(), "x_rotation_factor_fast: %f", docking_parameters_.x_rotation_factor_fast);
    RCLCPP_INFO(this->get_logger(), "stop_error_angle: %f", docking_parameters_.stop_error_angle);
    RCLCPP_INFO(this->get_logger(), "stop_error_linear: %f", docking_parameters_.stop_error_linear);
    RCLCPP_INFO(this->get_logger(), "fiducial_positions_file: %s", docking_parameters_.fiducial_positions_file.c_str());
  }

private:
  void controlLoop()
  {
    // Example usage: simulate error for angular and linear controllers
    double angular_error = 1.0; // Replace with real error
    double linear_error = 2.0;  // Replace with real error
    double dt = 0.1;

    double angular_output = angular_pid_.compute(angular_error, dt);
    double linear_output = linear_pid_.compute(linear_error, dt);

    RCLCPP_INFO(this->get_logger(), "Angular Output: %.2f, Linear Output: %.2f",
                angular_output, linear_output);
  }

  waver_docking::DockingParameters docking_parameters_;
  pid_module::PIDController angular_pid_;
  pid_module::PIDController linear_pid_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingNode>());
  rclcpp::shutdown();
  return 0;
}