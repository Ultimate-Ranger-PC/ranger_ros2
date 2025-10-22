#pragma once
/**
 * @file ranger_action.hpp
 * @brief Header‑only ROS 2 Action‑Server zur Blinklichtsteuerung
 */

#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include "ranger_msgs/action/blink_light_action.hpp"

namespace westonrobot {

class RangerROSAction : public std::enable_shared_from_this<RangerROSAction>
{
public:
  using Blink = ranger_msgs::action::BlinkLightAction;
  using GoalHandleBlink = rclcpp_action::ServerGoalHandle<Blink>;

  // Konstruktor erhält Node und Robot‑Referenz
  inline RangerROSAction(rclcpp::Node::SharedPtr node,
                         std::shared_ptr<RangerRobot> robot)
  : node_(std::move(node)), robot_(std::move(robot))
  {
    SetupAction();
  }

private:
  inline void SetupAction();
  inline rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Blink::Goal> goal);
  inline rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleBlink> goal_handle);
  inline void handle_accepted(const std::shared_ptr<GoalHandleBlink> goal_handle);
  inline void execute(const std::shared_ptr<GoalHandleBlink> goal_handle);
  inline void toggle_light();

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RangerRobot> robot_;
  rclcpp_action::Server<Blink>::SharedPtr action_server_;
};

// -----------------------------------------------------------------------------
// Inline‑Implementierungen
// -----------------------------------------------------------------------------

inline void RangerROSAction::SetupAction()
{
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<Blink>(
      node_,
      "blink_light",
      std::bind(&RangerROSAction::handle_goal, this, _1, _2),
      std::bind(&RangerROSAction::handle_cancel, this, _1),
      std::bind(&RangerROSAction::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "Blink Light Action Server initialized.");
}

inline rclcpp_action::GoalResponse RangerROSAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Blink::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(),
              "New goal: repetitions=%d, ON=%.2fs, OFF=%.2fs",
              goal->repetitions, goal->frequency_on, goal->frequency_off);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

inline rclcpp_action::CancelResponse RangerROSAction::handle_cancel(
    const std::shared_ptr<GoalHandleBlink>)
{
  RCLCPP_INFO(node_->get_logger(), "Cancel request received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

inline void RangerROSAction::handle_accepted(
    const std::shared_ptr<GoalHandleBlink> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

inline void RangerROSAction::execute(
    const std::shared_ptr<GoalHandleBlink> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Blink::Feedback>();
  auto result = std::make_shared<Blink::Result>();
  bool endless = false;
  int goal_repetitions = goal->repetitions;

  if (goal->repetitions == 0)
  {
    toggle_light();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Single toggle executed.");
    return;
  } else if (goal->repetitions == -1)
  {
    goal_repetitions= 2;
    endless = true;
  }
  

  for (int i = 1; i <= goal_repetitions; ++i)
  {
    if (goal_handle->is_canceling())
    {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Blink action canceled.");
      return;
    }
    if (endless){
      i--;
    }

    toggle_light();
    std::this_thread::sleep_for(std::chrono::duration<float>(goal->frequency_on));

    toggle_light();
    std::this_thread::sleep_for(std::chrono::duration<float>(goal->frequency_off));

    feedback->current_iteration = i;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(node_->get_logger(), "Blink %d/%d done", i, goal->repetitions);
  }

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "Blink action completed successfully.");
}


inline void RangerROSAction::toggle_light()
{
  AgxLightMode f_mode;
  uint8_t f_value = 0;
  const char *state = "off";

  auto robot_state = robot_->GetRobotState();
  if (robot_state.light_state.front_light.mode == CONST_ON)
  {
    f_mode = AgxLightMode::CONST_OFF;
    state = "off";
  }
  else
  {
    f_mode = AgxLightMode::CONST_ON;
    state = "on";
  }

  robot_->SetLightCommand(f_mode, f_value, f_mode, f_value);
  RCLCPP_INFO(node_->get_logger(), "Light switched %s.", state);
}

}  // namespace westonrobot
