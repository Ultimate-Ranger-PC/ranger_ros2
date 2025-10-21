/**
* @file ranger_messenger.hpp
* @date 2021-04-20
* @brief
*
*/

#ifndef RANGER_ACTION_HPP
#define RANGER_ACTION_HPP

//std and c++ inlclude
#include <string>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>

//ros include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <rclcpp/rclcpp.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
//third libaray inclue
#include "ugv_sdk/details/robot_base/ranger_base.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include <eigen3/Eigen/Core>

//user msg include
#include <ranger_msgs/action/blink_light_action.hpp>


#include "ranger_base/ranger_params.hpp"

namespace westonrobot {
class RangerROSAction : public std::enable_shared_from_this<RangerROSAction>
{
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_central;
    double max_steer_angle_parallel;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType { kRanger = 0, kRangerMiniV1, kRangerMiniV2 };

 public:
  RangerROSAction(rclcpp::Node::SharedPtr& node);

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  //void SetupServices();  // Function for setting up ROS2 service
  void PublishStateToROS();

  rclcpp_action::Server<Blink>::SharedPtr action_server_;
  bool light_on_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Blink::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleBlink> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleBlink> goal_handle);

  void execute(const std::shared_ptr<GoalHandleBlink> goal_handle);

  void toggle_light();

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;
  RobotParams robot_params_;

  // constants
  const double steer_angle_tolerance_ = 0.005;  // ~+-0.287 degrees

  // parameters
  std::string robot_model_;
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_;

  /// Covariance diagonal
  double position_covariance_;
  double orientation_covariance_;
  double linear_velocity_covariance_;
  double angular_velocity_covariance_;

  uint8_t motion_mode_ = 0;

  rclcpp::Publisher<ranger_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::MotionState>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::ActuatorStateArray>::SharedPtr actuator_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // ROS2 Service to Reset the Odometry Frame
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_light_service_;

  // odom variables
  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
