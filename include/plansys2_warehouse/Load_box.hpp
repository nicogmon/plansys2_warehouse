// Copyright 2025 Nicolás García Moncho

#ifndef PLANSYS2_WAREHOUSE__LOAD_BOX_HPP_
#define PLANSYS2_WAREHOUSE__LOAD_BOX_HPP_

#include <string>
#include <random>

#include "behaviortree_cpp_v3/basic_types.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2_warehouse
{

class Load_box : public plansys2::ActionExecutorClient
{
public:
  Load_box();
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

private:
  void do_work() override;

  int counter_;
  bool fail_flag_ = true; // true for action failure simulation

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
  int prob_ = 10;
};

}  // namespace plansys2_warehouse
#endif  // PLANSYS2_WAREHOUSE__LOAD_BOX_HPP_
