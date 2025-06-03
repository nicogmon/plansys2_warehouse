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

#include <iostream>
#include <string>

#include "plansys2_warehouse/Load_box.hpp"
using namespace std::chrono_literals;

namespace plansys2_warehouse
{

Load_box::Load_box()
: plansys2::ActionExecutorClient("load_box", 1s),
  gen_(rd_())
{
  counter_ = 0;
  dis_ = std::uniform_int_distribution<>(0, prob_ -1 );
  get_parameter_or("specialized_arguments", specialized_arguments_, std::vector<std::string>({""}));

  RCLCPP_INFO(get_logger(), "Load_box created");
  auto esp_size = specialized_arguments_.size();
  RCLCPP_INFO(get_logger(), "Specialized arguments size: %ld", esp_size);
  RCLCPP_INFO(get_logger(), "Specialized argument: %s", specialized_arguments_[0].c_str());
  // if (strcmp(specialized_arguments_[0].c_str(), "small_robot_1") == 0) {  //Permite simular el fallo de un único robot 
  //   fail_flag_ = true;
  // }
}

void Load_box::do_work()
{
  if (counter_ == 0){

  std::cout << get_arguments()[0].c_str() << " loading box " << get_arguments()[1].c_str() << std::endl;
  }
  counter_++;
  

  // if (strcmp(specialized_arguments_[0].c_str(), "small_robot_1") == 0) {  //Permite modificar la probabilidad de fallo de un robot concreto
  //   prob = 2;
  // }
  if ((dis_(gen_) == 0) && fail_flag_) {
    RCLCPP_ERROR(get_logger(), "Load_box failed");
    fail_flag_ = false;
    counter_ = 0;
    finish(false, 0.1, "Load_box failed");
    return;
  }

  if (counter_ < 5) {
    send_feedback(counter_, "Load running");
  } else {
    counter_ = 0;
    std::cout << get_arguments()[1].c_str() << " unloaded at " << get_arguments()[0].c_str() << std::endl;
    finish(true, 1.0, "Load completed");
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Load_box::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  counter_ = 0;
  return ActionExecutorClient::on_deactivate(previous_state);
}

}  //  namespace plansys2_warehouse

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2_warehouse::Load_box>();

  node->set_parameter(rclcpp::Parameter("action_name", "load_box"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
