// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_WAREHOUSE__BEHAVIOR_TREE_NODES__UNLOAD_BOX_HPP_
#define PLANSYS2_WAREHOUSE__BEHAVIOR_TREE_NODES__UNLOAD_BOX_HPP_

#include <string>
#include <iostream>

#include "plansys2_warehouse/Unload_box.hpp"

using namespace std::chrono_literals;

namespace plansys2_warehouse
{
Unload_box::Unload_box()
: plansys2::ActionExecutorClient("unload_box", 1s)
{
  counter_ = 0;
  get_parameter_or("specialized_arguments", specialized_arguments_, std::vector<std::string>({""}));

  
  RCLCPP_INFO(get_logger(), "Unload_box created");
  auto esp_size = specialized_arguments_.size();
  RCLCPP_INFO(get_logger(), "Specialized arguments size: %ld", esp_size);
  RCLCPP_INFO(get_logger(), "Specialized argument: %s", specialized_arguments_[0].c_str());
}



void Unload_box::do_work()
{
  RCLCPP_INFO(get_logger(), "Unload_box tick %d", counter_);
  
  counter_++;

  // if (rand() % 1 == 0) {
  //   std::cout << "Unload_box failed" << std::endl;
  //   counter_ = 0;
  //   finish(false, counter / 5, "Unload_box failed");
  // }
  if (counter_ < 5) {
    send_feedback(counter_, "Unload running");

  } else {
    counter_ = 0;
    finish(true, 1.0, "Unload completed");//segundo parametro nos puede servir
                                          //para indicar fallo a mitad de ejecucion
                                          // y el rpimero indicar que ha fallado la accion
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Unload_box::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Unload_box deactivated");
  counter_ = 0;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2_warehouse::Unload_box>();

  node->set_parameter(rclcpp::Parameter("action_name", "unload_box"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
#endif // PLANSYS2_WAREHOUSE__BEHAVIOR_TREE_NODES__UNLOAD_BOX_HPP_