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

#include <plansys2_pddl_parser/Utils.hpp>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <random>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

class WarehouseController : public rclcpp::Node
{
public:
  WarehouseController()
  : rclcpp::Node("warehouse_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();

    // std::string goal = "(and (box_at s_box_1 warehouse_2_sh) (box_at s_box_2 warehouse_2_sh) (box_at s_box_3 warehouse_2_sh) (box_at m_box_1 warehouse_2_sh) (box_at m_box_2 warehouse_2_sh) (box_at m_box_3 warehouse_2_sh))";
    // "(and( (bed_tied princ_bed) (table_setup princ_table) (cleaned_dishes dishes1) (trash_pickup trashbin) (person_attended person1 livingroom) (robot_at kobuki entrance)))" (person_attended person1 livingroom)
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("plansys2_warehouse");
    std::cout << "package_share_directory: " << package_share_directory << std::endl;
    std::ifstream file(package_share_directory + "/config/Goals.txt");
    
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo Goals.txt" << std::endl;
        return false;
    }
    std::string line;
    std::getline(file, line);
    std::ostringstream p_goal;
    p_goal << "(and (" << line << "))" << std::endl;
    std::string f_goal = p_goal.str();
    RCLCPP_INFO(get_logger(), "Goal: %s", f_goal.c_str());
    // std::string goal = "(and (box_at s_box_1 warehouse_2_sh) (box_at s_box_2 warehouse_2_sh) )";

    problem_expert_->setGoal(plansys2::Goal(f_goal));
    

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        return false;
    }

    // Execute the plan
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
    return true;
  }

  void init_knowledge()
  {
    
    problem_expert_->addInstance(plansys2::Instance{"small_robot", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"medium_robot", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"big_robot", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"small_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"medium_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"inter_zone", "zone"});

    problem_expert_->addInstance(plansys2::Instance{"s_central", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"s_sh_1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"s_sh_2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"s_sh_3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"m_central", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"m_sh_1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"m_sh_2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"m_sh_3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"warehouse_2_sh", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"common_zone", "waypoint"});

    problem_expert_->addInstance(plansys2::Instance{"s_box_1", "box"});
    problem_expert_->addInstance(plansys2::Instance{"s_box_2", "box"});
    problem_expert_->addInstance(plansys2::Instance{"s_box_3", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_1", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_2", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_3", "box"});
    

    
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at small_robot s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone small_robot small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot small_robot)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity small_robot) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load small_robot) 0)"));


    problem_expert_->addPredicate(plansys2::Predicate("(robot_at medium_robot m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone medium_robot medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot medium_robot)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity medium_robot) 3)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load medium_robot) 0)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at big_robot warehouse_2_sh)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone big_robot inter_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot big_robot)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity big_robot) 5)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load big_robot) 0)"));

    problem_expert_->addPredicate(plansys2::Predicate("(box_at s_box_1 s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(box_at s_box_2 s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(box_at s_box_3 s_sh_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(box_at m_box_1 m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(box_at m_box_2 m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(box_at m_box_3 m_sh_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone s_central small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone s_sh_1 small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone s_sh_2 small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone s_sh_3 small_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_central medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_1 medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_2 medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_3 medium_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone warehouse_2_sh inter_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone common_zone inter_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone common_zone small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone common_zone medium_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone s_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone s_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected common_zone warehouse_2_sh)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_1 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_2 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_3 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_central common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_central common_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected warehouse_2_sh common_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_1 s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_1 s_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_1 s_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_2 s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_2 s_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_2 s_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_3 s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_3 s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_sh_3 s_central)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_central)"));


  }

  void step()
  {
    auto feedback = executor_client_->getFeedBack();
    
    for (const auto &action_feedback : feedback.action_execution_status) {
          RCLCPP_INFO(get_logger(), "Acción: %s, Estado: %d",
                      action_feedback.action_full_name.c_str(),
                      action_feedback.status);
    }
    // estado 1 esperando 2 ejectuando 3 fallido 4 exitoso 5 cancelado

    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }



private:
  // typedef enum {STARTING, DESTROY } StateType;
  // StateType state_;
  // std::string planet;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WarehouseController>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok() ) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}