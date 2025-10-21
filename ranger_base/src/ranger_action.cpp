/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_action.hpp"

#include "ranger_base/kinematics_model.hpp"

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
// namespace {
// double DegreeToRadian(double x) { return x * M_PI / 180.0; }
// }  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node){

  node_ = node;
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(true);
  } else {
    robot_ = std::make_shared<RangerRobot>(false);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(),"Failed to connect to the CAN port");
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(),"Invalid port name: %s", port_name_.c_str());
    return;
  }

  //SetupServices();  // Initialize the services
  SetupAction();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  position_covariance_ = node_->declare_parameter<double>("position_covariance", 0.1);
  orientation_covariance_ = node_->declare_parameter<double>("orientation_covariance", 0.1);
  linear_velocity_covariance_ = node_->declare_parameter<double>("linear_velocity_covariance", 0.1);
  angular_velocity_covariance_ = node_->declare_parameter<double>("angular_velocity_covariance", 0.1);

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model:  odom_frame: %s\n base_frame: %s/%s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
    } else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
    }
  }
}

void RangerROSMessenger::SetupAction() {
  // publisher
  
  // subscriber

  // service server
  light_action_server_ = rclcpp_action::create_server<Blink>(
      this,
      "blink_light",
      std::bind(&BlinkActionServer::handle_goal, this, _1, _2),
      std::bind(&BlinkActionServer::handle_cancel, this, _1),
      std::bind(&BlinkActionServer::handle_accepted, this, _1)
    );

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

// void RangerROSMessenger::SetupServices() {
//     RCLCPP_INFO(node_->get_logger(), "Action Services are.");
// }

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // publish system state
//   {
//     ranger_msgs::msg::SystemState system_msg;
//     if (state.light_state.front_light.mode == CONST_ON) {
//       system_msg.light_state = 1;
//     } else if (state.light_state.front_light.mode == CONST_OFF) {
//       system_msg.light_state = 0;
//     } else {
//       system_msg.light_state = 2;
//     }

//     system_state_pub_->publish(system_msg);
//   }

  // publish BMS state
}

rclcpp_action::Server<Blink>::SharedPtr action_server_;

rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Blink::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "New Goal: %d iterations, ON=%.2fs, OFF=%.2fs",
      goal->repetitions, goal->on_time_seconds, goal->off_time_seconds);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleBlink>)
{
    RCLCPP_INFO(get_logger(), "Cancel action.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BlinkActionServer::handle_accepted(const std::shared_ptr<GoalHandleBlink> goal_handle)
{
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void BlinkActionServer::execute(const std::shared_ptr<GoalHandleBlink> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Blink::Feedback>();
    auto result = std::make_shared<Blink::Result>();

    if (goal->repetitions == -1) {
        toggleLight()
        // Set success response
        robot_->SetLightCommand(f_mode, f_value, f_mode, f_value);
        result->success = true;
        goal_handle->succeed(result);
        return;
    }

    for (int i = 1; i <= goal->repetitions; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Blink-Action canceled.");
        return;
      }

      toggle_light()
      rclcpp::sleep_for(std::chrono::duration<float>(goal->on_time_seconds));


      toggle_light()
      rclcpp::sleep_for(std::chrono::duration<float>(goal->off_time_seconds));

      feedback->current_blink = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Blink %d iteration, from %d repetitions", i, repetitions);
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Blink-Action %d." result);
}


void BlinkActionServer::toggle_light()
{
    AgxLightMode f_mode;
    uint8_t f_value = 0;
    const char* set_light = "off";
    auto state = robot_->GetRobotState();
    if (state.light_state.front_light.mode == CONST_ON) {
        f_mode = AgxLightMode::CONST_ON;
    } else {
        f_mode = AgxLightMode::CONST_OFF;
        set_light = "on";
    }
    RCLCPP_INFO(node_->get_logger(), "Light has been set %s.", set_light);


}


}  // namespace westonrobot
