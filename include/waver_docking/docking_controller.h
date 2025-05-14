#ifndef WAVER_DOCKING__DOCKING_CONTROLLER_H_
#define WAVER_DOCKING__DOCKING_CONTROLLER_H_

#include <memory>
#include <thread>
#include <atomic>
#include <array>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pid/pid_controller.h"
#include "docking/docking_parameters.h"
#include "robot_docking/action/robot_docking.hpp"

namespace waver_docking {

using RobotDocking = robot_docking::action::RobotDocking;
using GoalHandleDocking = rclcpp_action::ServerGoalHandle<RobotDocking>;

/**
 * @class RobotDocking
 * @brief Encapsulates the PID-based control loop for docking.
 */
class RobotDocking
{
public:
  RobotDocking(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    const std::array<double,2>& point_left,
    const std::array<double,2>& point_right,
    pid::PIDController angular_pid,
    pid::PIDController linear_pid,
    const DockingParameters& params);

  /// Publish velocity commands
  void setPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);
  /// Start the docking control thread
  void start();
  /// Stop the docking control thread
  void stop();

private:
  void computeGeometry();
  void run();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  pid::PIDController angular_pid_;
  pid::PIDController linear_pid_;
  DockingParameters params_;

  std::array<double,2> point_left_, point_right_;
  std::array<double,2> e1_, e2_;
  Eigen::Matrix2d matrix_inv_;
  double x_mid_, y_mid_, ref_rot_;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

/**
 * @class DockingServiceNode
 * @brief ROS2 node offering a docking action server
 */
class DockingServiceNode : public rclcpp::Node
{
public:
  DockingServiceNode();

private:
  rclcpp_action::Server<RobotDocking>::SharedPtr action_server_;
  std::shared_ptr<RobotDocking> docking_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotDocking::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleDocking> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleDocking> goal_handle);
};

}  // namespace waver_docking

#endif  // WAVER_DOCKING__DOCKING_CONTROLLER_H_
