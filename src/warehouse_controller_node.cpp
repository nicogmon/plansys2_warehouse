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
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "problem_checker.cpp"


class WarehouseController : public rclcpp::Node
{
public:
  WarehouseController()
  : rclcpp::Node("warehouse_controller")
  {
    goal_suscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/goal", 10, std::bind(&WarehouseController::goal_callback, this, std::placeholders::_1));
    cancel_publisher_ = this->create_publisher<std_msgs::msg::String>("/cancel", 10);



  }
  void goal_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
    goal_ = msg->data.c_str();
  }

  int init()
  {
    if (goal_.empty()) {
      RCLCPP_INFO(get_logger(), "Goal not received yet");
      return 0;
    }
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
    problem_checker_ = std::make_shared<ProblemChecker>(problem_expert_);
    problem_checker_->get_necesary_predicates();

    // std::string goal = "(and (box_at s_box_1 warehouse_2_sh) (box_at s_box_2 warehouse_2_sh) (box_at s_box_3 warehouse_2_sh) (box_at m_box_1 warehouse_2_sh) (box_at m_box_2 warehouse_2_sh) (box_at m_box_3 warehouse_2_sh))";
    // "(and( (bed_tied princ_bed) (table_setup princ_table) (cleaned_dishes dishes1) (trash_pickup trashbin) (person_attended person1 livingroom) (robot_at kobuki entrance)))" (person_attended person1 livingroom)
   
    problem_expert_->setGoal(plansys2::Goal(goal_));
    RCLCPP_INFO(get_logger(), "Goal set successfully: %s", goal_.c_str());

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        return -1;
    }

    // Execute the plan
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
    return 1;
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
    

    
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at small_robot s_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone small_robot small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot small_robot)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity small_robot) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load small_robot) 0)"));


    problem_expert_->addPredicate(plansys2::Predicate("(robot_at medium_robot m_central)"));
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
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_central s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_central s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s_central s_sh_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_1 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_2 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_sh_3 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_central m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_central m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected m_central m_sh_3)"));

    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone s_sh_1) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone s_sh_2) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone s_sh_3) 120)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone s_central) 100)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone m_sh_1) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone m_sh_2) 60)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone m_sh_3) 52)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone m_central) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s common_zone warehouse_2_sh) 80)"));
    
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_1 common_zone) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_2 common_zone) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_3 common_zone) 120)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_central common_zone) 100)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_1 common_zone) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_2 common_zone) 60)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_3 common_zone) 52)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_central common_zone) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s warehouse_2_sh common_zone) 80)"));
    
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_1 s_sh_2) 25)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_1 s_sh_3) 30)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_1 s_central) 12)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_2 s_sh_1) 25)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_2 s_sh_3) 32)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_2 s_central) 12)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_3 s_sh_1) 30)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_3 s_sh_2) 32)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_3 s_central) 25)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_central s_sh_1) 12)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_central s_sh_2) 12)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_central s_sh_3) 25)"));
    
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_1 m_sh_2) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_1 m_sh_3) 65)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_1 m_central) 45)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_2 m_sh_1) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_2 m_sh_3) 90)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_2 m_central) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_3 m_sh_1) 65)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_3 m_sh_2) 90)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_3 m_central) 46)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_central m_sh_1) 45)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_central m_sh_2) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_central m_sh_3) 46)"));
    



  }

  void step()
  {
    auto feedback = executor_client_->getFeedBack();
    
    


    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_NOT_EXECUTED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_EXECUTING;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_FAILED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_SUCCEEDED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_CANCELLED;


    for (const auto &action_feedback : feedback.action_execution_status) {
          if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED) {
            action_NOT_EXECUTED.push_back(action_feedback);
          } else if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
            action_EXECUTING.push_back(action_feedback);
          } else if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
            action_FAILED.push_back(action_feedback);
          } else if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
            action_SUCCEEDED.push_back(action_feedback);
          } else if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
            action_CANCELLED.push_back(action_feedback);
          }
    }
    // for (const auto &action : action_NOT_EXECUTED) {
    //   RCLCPP_INFO(get_logger(), "Action %s NOT_EXECUTED", action.action_full_name.c_str());
    // }
    RCLCPP_INFO(get_logger(), "ACTIONS STATTEEEEE ");
    for (const auto &action : action_EXECUTING) {
      RCLCPP_INFO(get_logger(), "Action %s EXECUTING", action.action_full_name.c_str());
    }
    for (auto &action : action_FAILED) {
      RCLCPP_INFO(get_logger(), "Action %s FAILED", action.action_full_name.c_str());
      problem_checker_->restore_action(action);
    }
    if (!action_FAILED.empty()){
      RCLCPP_INFO(get_logger(), "Actions reverted after failure");
      executor_client_->cancel_plan_execution();
    }
    feedback = executor_client_->getFeedBack();
    action_CANCELLED.clear();
    for (const auto &action_feedback : feedback.action_execution_status) {
          if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
            action_CANCELLED.push_back(action_feedback);
          }
    }
    for (auto &action : action_CANCELLED) {
      RCLCPP_INFO(get_logger(), "Action %s CANCELLED", action.action_full_name.c_str());
      problem_checker_->restore_action(action);
    }
    std::cout << "\n\n";

    plansys2::Goal actual_goal = problem_expert_->getGoal();
    // std::cout << "Goal actual: " << parser::pddl::toString(actual_goal) << std::endl;
    // std::cout << "Goal esperado: " << goal_ << std::endl;

    if (parser::pddl::toString(actual_goal) != goal_) {
      RCLCPP_INFO(get_logger(), "Goal changed");
      //to-do 
      // cancelar plan actual
      executor_client_->cancel_plan_execution();
      auto cancel_msg = std_msgs::msg::String();
      cancel_msg.data = "plan_cancelled";
      cancel_publisher_->publish(cancel_msg);
      
      
      problem_expert_->setGoal(plansys2::Goal(goal_));
      RCLCPP_INFO(get_logger(), "Goal set successfully: %s", goal_.c_str());
      for (auto &action : action_EXECUTING) {
        problem_checker_->restore_action(action);
      }
    }

    
    // if (!action_CANCELLED.empty()){
    //   RCLCPP_INFO(get_logger(), "Actions reverted after cancellation");
    //   action_CANCELLED.clear();
    //   // si se cancella move mandar stop a navigation server
    // }
    
    // estado 1 esperando, 2 ejectuando, 3 fallido, 4 exitoso, 5 cancelado
    
    
    


    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
        exit(0);
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        bool problem_ok = problem_checker_->check_problem();
        if (problem_ok) {
          RCLCPP_INFO(get_logger(), "Problem OK");
        } else {
          RCLCPP_ERROR(get_logger(), "Problem NOT OK");
        }
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              
              exit(0);
        }
        if (!executor_client_->start_plan_execution(plan.value())) {
          RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
        }
        // en principio se restaura todon los predicados de move y se encuentra plan 
      }
    }
  }


  std::string goal_;
private:
  // typedef enum {STARTING, DESTROY } StateType;
  // StateType state_;
  // std::string planet;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_suscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cancel_publisher_;
  std::shared_ptr<ProblemChecker> problem_checker_;
  // std::string goal_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WarehouseController>();
  

  int result = node->init();
  while (result == 0) {
      rclcpp::spin_some(node->get_node_base_interface());
      result = node->init();  
  }
  if (result == -1) {
      std::cout << "Error en la inicialización" << std::endl;
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