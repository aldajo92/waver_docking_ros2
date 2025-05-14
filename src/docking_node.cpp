#include "rclcpp/rclcpp.hpp"
#include "waver_docking/pid_parameters.h"
#include "pid_module/pid_controller.h"

class PIDNode : public rclcpp::Node
{
public:
  PIDNode()
  : Node("pid_node"), pid_parameters_(*this, "pid"), pid_(pid_parameters_.make_controller())
  {
    // Initialize the PID controller using the parameters
    pid_ = pid_parameters_.make_controller();

    // Create a timer to simulate the control loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&PIDNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "PID Node has been started.");
  }

private:
  void controlLoop()
  {
    // Simulate an error signal (e.g., target - current position)
    double error = 10.0 - current_position_;
    double dt = 0.1;  // Time step (100 ms)

    // Compute control output using the PID controller
    double control_output = pid_.compute(error, dt);

    // Update the current position (for simulation purposes)
    current_position_ += control_output * dt;

    // Log the control output and current position
    RCLCPP_INFO(this->get_logger(), "Error: %.2f, Control Output: %.2f, Position: %.2f",
                error, control_output, current_position_);
  }

  waver_docking::PIDParameters pid_parameters_;  // PIDParameters instance
  pid_module::PIDController pid_;               // PIDController instance
  double current_position_ = 0.0;               // Simulated current position
  rclcpp::TimerBase::SharedPtr timer_;          // Timer for periodic updates
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDNode>());
  rclcpp::shutdown();
  return 0;
}