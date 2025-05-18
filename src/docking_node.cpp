#include "rclcpp/rclcpp.hpp"
#include "waver_docking/pid_parameters.h"
#include "pid_module/pid_controller.h"
#include <Eigen/Dense>

class DockingNode : public rclcpp::Node
{
public:
  DockingNode()
      : Node("docking_node"),
        distance_pid_params_(*this, "distance_pid"),
        angle_pid_params_(*this, "angle_pid"),
        distance_pid_(distance_pid_params_.make_controller()),
        angle_pid_(angle_pid_params_.make_controller())
  {

    this->declare_parameter("P.x", 0.0);
    this->declare_parameter("P.y", 0.0);
    this->declare_parameter("Q.x", 0.0);
    this->declare_parameter("Q.y", 0.0);

    P_.x() = this->get_parameter("P.x").as_double();
    P_.y() = this->get_parameter("P.y").as_double();
    Q_.x() = this->get_parameter("Q.x").as_double();
    Q_.y() = this->get_parameter("Q.y").as_double();

    PQMid_ = (P_ + Q_) / 2.0;
    e_X_ = Q_ - PQMid_;
    e_Y_ = Eigen::Vector2d::Zero(); // Or however you want to define it

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10 Hz
        std::bind(&DockingNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "PID Node has been started.");
  }

private:
  void controlLoop()
  {
    double distance_error = 10.0 - current_distance_;
    double angle_error = 1.0 - current_angle_;
    double dt = 0.1;

    double distance_output = distance_pid_.compute(distance_error, dt);
    double angle_output = angle_pid_.compute(angle_error, dt);

    current_distance_ += distance_output * dt;
    current_angle_ += angle_output * dt;

    RCLCPP_INFO(this->get_logger(), "P: (%.2f, %.2f), Q: (%.2f, %.2f), PQMid: (%.2f, %.2f)",
                P_.x(), P_.y(), Q_.x(), Q_.y(), PQMid_.x(), PQMid_.y());
    RCLCPP_INFO(this->get_logger(),
                "\nDist \t Error: %.2f, Output: %.2f, Distance: %.2f \nAngle \t Error: %.2f, Output: %.2f, Angle: %.2f",
                distance_error, distance_output, current_distance_,
                angle_error, angle_output, current_angle_);
  }

  waver_docking::PIDParameters distance_pid_params_;
  waver_docking::PIDParameters angle_pid_params_;
  Eigen::Vector2d P_, Q_, PQMid_;
  Eigen::Vector2d e_X_, e_Y_;

  pid_module::PIDController distance_pid_;
  pid_module::PIDController angle_pid_;
  double current_distance_ = 0.0;
  double current_angle_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingNode>());
  rclcpp::shutdown();
  return 0;
}