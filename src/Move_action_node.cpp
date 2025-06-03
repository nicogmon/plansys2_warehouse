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

#include <iostream>
#include <string>

#include "plansys2_warehouse/Move.hpp"

using namespace std::chrono_literals;

namespace plansys2_warehouse
{
Move::Move()
: plansys2::ActionExecutorClient("move", 1s)
{
  declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>());
  get_parameter_or("specialized_arguments", specialized_arguments_, std::vector<std::string>({""}));

  RCLCPP_INFO(get_logger(), "Move created");

  std::vector<std::string> wp_names;
  get_parameter_or("waypoints", wp_names, {});

  for (const auto & wp : wp_names) {
    declare_parameter<std::vector<double>>("waypoint_coords." + wp);
    std::vector<double> coords;

    if (get_parameter_or("waypoint_coords." + wp, coords, {})) {
      if (coords.size() == 3) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = coords[0];
        pose.pose.position.y = coords[1];
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        waypoints_[wp] = pose;
      } else {
        RCLCPP_ERROR(get_logger(), "Waypoint [%s] has incorrect coordinate size!", wp.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "No coordinate configured for waypoint [%s]", wp.c_str());
    }
  }

  using namespace std::placeholders;
  pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&Move::current_pos_callback, this, _1));

}

void Move::current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pos_ = msg->pose.pose;
}



rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Move::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  send_feedback(0.0, "Move starting");

  std::string robot = get_arguments()[0];

  std::string service_name = robot + "/navigate_to_pose";

  navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(), service_name);
  bool is_action_server_ready = false;
  do {
    is_action_server_ready =
      navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  } while (!is_action_server_ready);


  auto wp_to_navigate = get_arguments()[2];
  std::cout << robot <<  " start navigation to " <<  wp_to_navigate.c_str() << std::endl;


  goal_pos_ = waypoints_[wp_to_navigate];
  navigation_goal_.pose = goal_pos_;

  dist_to_move = getDistance(goal_pos_.pose, current_pos_);

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback = [this](NavigationGoalHandle::SharedPtr,
    NavigationFeedback feedback) {
      send_feedback(std::min(1.0,
        std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
    };

  send_goal_options.result_callback = [this](auto result) {
      std::cout << get_arguments()[0].c_str() <<  " reached " <<  get_arguments()[2].c_str() << std::endl;
      finish(true, 1.0, "Move completed");
    };

  future_navigation_goal_handle_ =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Move::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // RCLCPP_INFO(get_logger(), "Deactivate received");
  if (future_navigation_goal_handle_.valid()) {
    auto goal_handle = future_navigation_goal_handle_.get();

    if (goal_handle) {
      auto status = goal_handle->get_status();

      switch (status) {
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
          RCLCPP_INFO(get_logger(), "Goal is running, cancelling...");
          navigation_action_client_->async_cancel_goal(goal_handle);
          finish(false, 0.0, "Move canceled");
          break;
        default:
          break;
      }
    }
  }
  return ActionExecutorClient::on_deactivate(previous_state);
}

double Move::getDistance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2)
{
  return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
            (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
}

void Move::do_work()
{
}
}  
// namespace plansys2_house_problem

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2_warehouse::Move>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
