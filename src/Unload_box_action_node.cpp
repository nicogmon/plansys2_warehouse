// Copyright 2025 Nicolás García Moncho
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

#include <iostream>
#include <string>

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
  if (counter_ == 0){
  // RCLCPP_INFO(get_logger(), "%s unloading box %s",get_arguments()[0].c_str(), get_arguments()[1].c_str());
  std::cout << get_arguments()[0].c_str() << " unloading box " << get_arguments()[1].c_str() << std::endl;
  }

  counter_++;

  if ((rand() % 5 == 0) && fail_flag_) {
    RCLCPP_ERROR(get_logger(), "Unload_box failed");
    fail_flag_ = false;
    counter_ = 0;
    finish(false, 0.1, "Unload_box failed");
    return;
  }

  if (counter_ < 5) {
    send_feedback(counter_, "Unload running");
  } else {
    // RCLCPP_INFO(get_logger(), "%s unloaded at %s",get_arguments()[1].c_str(), get_arguments()[2].c_str());
    std::cout << get_arguments()[1].c_str() << " unloaded at " << get_arguments()[2].c_str() << std::endl;
    counter_ = 0;
    finish(true, 1.0, "Unload completed");  // segundo parametro nos puede servir
                                            // para indicar fallo a mitad de ejecucion
                                            //  y el rpimero indicar que ha fallado la accion
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Unload_box::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // RCLCPP_INFO(get_logger(), "Unload_box deactivated");
  counter_ = 0;
  // finish(true, 1.0, "Unload_box deactivated");
  return ActionExecutorClient::on_deactivate(previous_state);
}

}  // namespace plansys2_warehouse

// namespace plansys2_warehouse
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2_warehouse::Unload_box>();

  node->set_parameter(rclcpp::Parameter("action_name", "unload_box"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}  // namespace plansys2_warehouse
