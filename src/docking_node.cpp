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

    // Create a 2D rotation matrix for 90 degrees (PI/2 radians, counterclockwise)
    double angle = M_PI / 2.0; // 90 degrees in radians
    Eigen::Matrix2d rot90;
    rot90 << cos(angle), -sin(angle),
         sin(angle),  cos(angle);

    PQMid_ = (P_ + Q_) / 2.0;
    e_X_ = (Q_ - PQMid_).normalized();
    e_Y_ = rot90 * e_X_;
    e_Y_angle_ = vector_angle(e_Y_);

    Eigen::Matrix2d matrix_basis;
    matrix_basis.col(0) = e_X_;
    matrix_basis.col(1) = e_Y_;

    Eigen::Matrix2d matrix_basis_inv = matrix_basis.inverse();

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
    RCLCPP_INFO(this->get_logger(), "e_X: (%.2f, %.2f), e_Y: (%.2f, %.2f)",
                e_X_.x(), e_X_.y(), e_Y_.x(), e_Y_.y());
    RCLCPP_INFO(this->get_logger(), "e_Y_angle: %.2f", e_Y_angle_);
    RCLCPP_INFO(this->get_logger(), "e_Y_angle in degrees: %.2f", e_Y_angle_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(),
                "\nDist \t Error: %.2f, Output: %.2f, Distance: %.2f \nAngle \t Error: %.2f, Output: %.2f, Angle: %.2f",
                distance_error, distance_output, current_distance_,
                angle_error, angle_output, current_angle_);
  }

  double vector_angle(Eigen::Vector2d vector) const
  {
    // Calculate the angle in radians
    return std::atan2(vector.y(), vector.x());
  }

  waver_docking::PIDParameters distance_pid_params_;
  waver_docking::PIDParameters angle_pid_params_;
  Eigen::Vector2d P_, Q_, PQMid_;
  Eigen::Vector2d e_X_, e_Y_;
  double e_Y_angle_ = 0.0;

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