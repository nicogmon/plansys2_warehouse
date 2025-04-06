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

#include <string>
#include <iostream>

#include "plansys2_warehouse/Load_box.hpp"
using namespace std::chrono_literals;

namespace plansys2_warehouse
{
Load_box::Load_box()
: plansys2::ActionExecutorClient("load_box", 1s)
{
  counter_ = 0;
  
  get_parameter_or("specialized_arguments", specialized_arguments_, std::vector<std::string>({""}));

  
  RCLCPP_INFO(get_logger(), "Load_box created");
  auto esp_size = specialized_arguments_.size();
  RCLCPP_INFO(get_logger(), "Specialized arguments size: %ld", esp_size);
  RCLCPP_INFO(get_logger(), "Specialized argument: %s", specialized_arguments_[0].c_str());
  
}



void Load_box::do_work()
{
 
  
  RCLCPP_INFO(get_logger(), "Load_box tick %d", counter_);
  
  counter_++;
  // if (rand() % 1 == 0 && fail_flag_) {
  //   RCLCPP_ERROR(get_logger(), "Load_box failed");
  //   fail_flag_ = false;
  //   counter_ = 0;
  //   finish(false, counter_ / 5, "Load_box failed");
  // }
  auto status = this->get_internal_status();

  RCLCPP_INFO(get_logger(), "Load_box %d", status.state);
  if (status.state == 3 || status.state == 4) {
    RCLCPP_ERROR(get_logger(), "Load_box failed or cancelled");
    action_cancelled_ = true;
    
  }

  // if (status == BT::NodeStatus::IDLE) {  
  //   std::cout << "Move action was canceled!" << std::endl;
  //   finish(false, 0.0, "Move action canceled");
  //   return;
  // }

  // if (status == BT::NodeStatus::FAILURE) {
  //   std::cout << "Move action encountered an error!" << std::endl;
  //   finish(false, 0.0, "Move action failed");
  //   return;
  // }
  if (counter_ < 5) {
    send_feedback(counter_, "Load running");

  } else {
    counter_ = 0;
    finish(true, 1.0, "Load completed");//segundo parametro nos puede servir
                                          //para indicar fallo a mitad de ejecucion
  }
}
//on deactivate para acbar y resetao contador 

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Load_box::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  if (action_cancelled_) {
    RCLCPP_ERROR(get_logger(), "Load_box cancelled if");
    action_cancelled_ = false;
    counter_ = 0;
    finish(false, 1.0, "Load_box deactivated");
    return ActionExecutorClient::on_deactivate(previous_state);
  }
  else{
    RCLCPP_INFO(get_logger(), "Load_box deactivated else");
    counter_ = 0;
    finish(true, 1.0, "Load_box deactivated");
    return ActionExecutorClient::on_deactivate(previous_state);
  }
  
}



} // namespace plansys2_house_problem

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
 