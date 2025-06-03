// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "Apache License");
// you may not use this file except in compliance with the Apache License.
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Modifications made by Nicolás García Moncho, 2025
// This file is now distributed under the GNU General Public License v3.0 (GPLv3)
// for the purpose of integrating it into the project Reasignación Dinámica de Tareas
// en Sistemas de Planificación Multi-Robot.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You may obtain a copy of the GPLv3 at:
//
//     https://www.gnu.org/licenses/gpl-3.0.html




#include <memory>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Sim
  : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  Nav2Sim()
  : Node("navigate_to_pose_server") {}

  void start_server()
  {
    using namespace std::placeholders;

    repeat_sentence_action_server_ = rclcpp_action::create_server<NavigateToPose>(
        shared_from_this(), "navigate_to_pose", std::bind(&Nav2Sim::handle_goal, this, _1, _2),
        std::bind(&Nav2Sim::handle_cancel, this, _1),
        std::bind(&Nav2Sim::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr repeat_sentence_action_server_;
  NavigateToPose::Goal current_goal_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    std::string node_namespace = this->get_namespace();
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    auto pose_cmd = goal_handle->get_goal()->pose.pose;
    tf2::Quaternion q;
    tf2::fromMsg(pose_cmd.orientation, q);

    RCLCPP_INFO(this->get_logger(), "Starting navigation to %lf, %lf, %lf", pose_cmd.position.x,
                pose_cmd.position.y, q.getAngle());

    auto start = now();
    int current_times = 0;
    while (rclcpp::ok() && current_times++ < 5) {
      RCLCPP_INFO(this->get_logger(), "%s navigating %d ", node_namespace.c_str(), current_times);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);

        RCLCPP_INFO(this->get_logger(), "Action Canceled");

        return;
      }

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Nav2Sim::execute, this, _1), goal_handle}.detach();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Nav2Sim>();

  node->start_server();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
