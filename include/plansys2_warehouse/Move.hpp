
#ifndef PLANSYS2_WAREHOUSE__MOVE_HPP_
#define PLANSYS2_WAREHOUSE__MOVE_HPP_

#include <math.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace plansys2_warehouse {

class Move : public plansys2::ActionExecutorClient {
 public:
  Move();
  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void cancel_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state);

 private:
  double getDistance(const geometry_msgs::msg::Pose& pos1, const geometry_msgs::msg::Pose& pos2);
  void do_work() override;
  int counter_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub_;

  double dist_to_move;
};

}  // namespace plansys2_warehouse
#endif  // PLANSYS2_WAREHOUSE__BEHAVIOR_TREE_NODES__LOAD_BOX_HPP_