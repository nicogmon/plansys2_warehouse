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
}



void Unload_box::do_work()
{
  std::cout << "Unload_box tick " << counter_ << std::endl;
  // if (rand() % 1 == 0) {
  //   std::cout << "Unload_box failed" << std::endl;
  //   return BT::NodeStatus::FAILURE;
  // }
  counter_++;
  if (counter_ < 5) {
    send_feedback(counter_, "Patrol running");

  } else {
    counter_ = 0;
    finish(true, 1.0, "Patrol completed");//segundo parametro nos puede servir
                                          //para indicar fallo a mitad de ejecucion
  }
}

} // namespace plansys2_house_problem

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