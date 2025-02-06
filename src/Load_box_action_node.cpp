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
}



void Load_box::do_work()
{
  std::cout << "Load_box tick " << counter_ << std::endl;
  // if (rand() % 1 == 0) {
  //   std::cout << "Load_box failed" << std::endl;
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

// namespace plansys2_house_problem
} 
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
 