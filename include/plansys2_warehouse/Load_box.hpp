/*
 * Copyright (C) 2025 Nicolás García Moncho
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
