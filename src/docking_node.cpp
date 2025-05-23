#include "rclcpp/rclcpp.hpp"
#include "waver_docking/pid_parameters.h"
#include "pid_module/pid_controller.h"
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"

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

    // Declare parameters
    this->declare_parameter("fixed_frame", "map");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("P.x", 0.0);
    this->declare_parameter("P.y", 0.0);
    this->declare_parameter("Q.x", 0.0);
    this->declare_parameter("Q.y", 0.0);

    // Get parameters
    this->get_parameter("fixed_frame", fixed_frame_);
    this->get_parameter("robot_frame", robot_frame_);
    P_.x() = this->get_parameter("P.x").as_double();
    P_.y() = this->get_parameter("P.y").as_double();
    Q_.x() = this->get_parameter("Q.x").as_double();
    Q_.y() = this->get_parameter("Q.y").as_double();

    // Print parameters
    RCLCPP_INFO(this->get_logger(), "fixed_frame: %s", fixed_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "robot_frame: %s", robot_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "P: (%.2f, %.2f)", P_.x(), P_.y());
    RCLCPP_INFO(this->get_logger(), "Q: (%.2f, %.2f)", Q_.x(), Q_.y());

    ////////////////////////// Docking parameters initialization

    // Create a 2D rotation matrix for 90 degrees (PI/2 radians, counterclockwise)
    double angle = M_PI / 2.0; // 90 degrees in radians
    Eigen::Matrix2d rot90;
    rot90 << cos(angle), -sin(angle),
        sin(angle), cos(angle);

    PQMid_ = (P_ + Q_) / 2.0;
    e_X_ = (Q_ - PQMid_).normalized();
    e_Y_ = rot90 * e_X_;
    e_Y_angle_ = vector_angle(e_Y_);

    Eigen::Matrix2d matrix_basis;
    matrix_basis.col(0) = e_X_;
    matrix_basis.col(1) = e_Y_;

    Eigen::Matrix2d matrix_basis_inv = matrix_basis.inverse();

    // Print the matrices
    RCLCPP_INFO(this->get_logger(), "Matrix Basis:\n[%.2f, %.2f]\n[%.2f, %.2f]",
                matrix_basis(0, 0), matrix_basis(0, 1),
                matrix_basis(1, 0), matrix_basis(1, 1));

    RCLCPP_INFO(this->get_logger(), "Matrix Basis Inverse:\n[%.2f, %.2f]\n[%.2f, %.2f]",
                matrix_basis_inv(0, 0), matrix_basis_inv(0, 1),
                matrix_basis_inv(1, 0), matrix_basis_inv(1, 1));

    ////////////////////////// Robot Position

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ////////////////////////// Timers for control and position loop
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), // 2 Hz
        std::bind(&DockingNode::controlLoop, this));

    position_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5 Hz
        std::bind(&DockingNode::positionLoop, this));

    ////////////////////////// Publisher for control commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "PID Node has been started.");
  }

private:
  void controlLoop()
  {

    double distance_error;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      // Calculate distance error from robot to PQMid_
      distance_error = (robot_position_ - PQMid_).norm();

      // RCLCPP_INFO(this->get_logger(),
      //             "Robot position: (%.2f, %.2f), yaw: %.2f, distance error: %.2f",
      //             robot_position_.x(), robot_position_.y(), robot_yaw_, distance_error);
    }

    double distance_action = distance_pid_.compute(distance_error, 0.5);
    RCLCPP_INFO(this->get_logger(),
                "Distance error: %.2f, Distance action: %.2f",
                distance_error,
                distance_action);

    // TODO: Used as a code reference to use the PID, remove it later
    // double distance_error = 10.0 - current_distance_;
    // double angle_error = 1.0 - current_angle_;
    // double dt = 0.1;

    // double distance_output = distance_pid_.compute(distance_error, dt);
    // double angle_output = angle_pid_.compute(angle_error, dt);

    // current_distance_ += distance_output * dt;
    // current_angle_ += angle_output * dt;

    // TODO: This logs can be enabled by a parameter
    // RCLCPP_INFO(this->get_logger(), "P: (%.2f, %.2f), Q: (%.2f, %.2f), PQMid: (%.2f, %.2f)",
    //             P_.x(), P_.y(), Q_.x(), Q_.y(), PQMid_.x(), PQMid_.y());
    // RCLCPP_INFO(this->get_logger(), "e_X: (%.2f, %.2f), e_Y: (%.2f, %.2f)",
    //             e_X_.x(), e_X_.y(), e_Y_.x(), e_Y_.y());
    // RCLCPP_INFO(this->get_logger(), "e_Y_angle: %.2f", e_Y_angle_);
    // RCLCPP_INFO(this->get_logger(), "e_Y_angle in degrees: %.2f", e_Y_angle_ * 180.0 / M_PI);
    // RCLCPP_INFO(this->get_logger(),
    //             "\nDist \t Error: %.2f, Output: %.2f, Distance: %.2f \nAngle \t Error: %.2f, Output: %.2f, Angle: %.2f",
    //             distance_error, distance_output, current_distance_,
    //             angle_error, angle_output, current_angle_);
  }

  void positionLoop()
  {
    // Check if the transform is available
    if (tf_buffer_->canTransform(
            fixed_frame_,                   // target frame
            robot_frame_,                   // source frame
            tf2::TimePointZero,             // latest available
            std::chrono::milliseconds(10))) // timeout
    {
      try
      {
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_buffer_->lookupTransform(
                fixed_frame_,
                robot_frame_,
                tf2::TimePointZero);
        auto trans = transformStamped.transform.translation;
        auto rot = transformStamped.transform.rotation;

        // Convert quaternion to yaw
        double siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y);
        double cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // Update robot position and yaw
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          robot_position_ = Eigen::Vector2d(trans.x, trans.y);
          robot_yaw_ = yaw;
        }

        // RCLCPP_INFO(this->get_logger(), "Robot translation in %s: (%.2f, %.2f, %.2f)",
        //             fixed_frame_.c_str(), trans.x, trans.y, trans.z);
        // RCLCPP_INFO(this->get_logger(), "Robot rotation in %s: (%.2f, %.2f, %.2f, %.2f)",
        //             fixed_frame_.c_str(), rot.x, rot.y, rot.z, rot.w);
      }
      catch (const tf2::TransformException &ex)
      {
        // RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available yet.",
                  robot_frame_.c_str(), fixed_frame_.c_str());
    }
  }

  double vector_angle(Eigen::Vector2d vector) const
  {
    return std::atan2(vector.y(), vector.x());
  }

  // Docking input parameters (reordered)
  std::string fixed_frame_;
  std::string robot_frame_;

  // Docking internal parameters
  waver_docking::PIDParameters distance_pid_params_;
  waver_docking::PIDParameters angle_pid_params_;
  Eigen::Vector2d P_, Q_, PQMid_;
  Eigen::Vector2d e_X_, e_Y_;
  double e_Y_angle_ = 0.0;

  // Robot position
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex pose_mutex_;
  Eigen::Vector2d robot_position_{0.0, 0.0};
  double robot_yaw_{0.0};

  // PID controllers
  pid_module::PIDController distance_pid_;
  pid_module::PIDController angle_pid_;
  double current_distance_ = 0.0;
  double current_angle_ = 0.0;

  // Publisher for control commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr position_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingNode>());
  rclcpp::shutdown();
  return 0;
}
