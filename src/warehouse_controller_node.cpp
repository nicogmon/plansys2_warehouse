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


#include <chrono>
#include <fstream>
#include <memory>
#include <random>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "add_robot.hpp"
#include "problem_checker.hpp"

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
  void goal_callback(const std_msgs::msg::String::SharedPtr msg) {goal_ = msg->data.c_str();}

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
    problem_checker_->get_necessary_predicates();

    add_robot_ =
      std::make_shared<AddRobot>(problem_expert_);

    problem_expert_->setGoal(plansys2::Goal(goal_));
    RCLCPP_INFO(get_logger(), "Goal set successfully: %s", goal_.c_str());

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal "
                << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return -1;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return 1;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"small_robot_1", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"medium_robot_1", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"big_robot_1", "robot"});

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
    problem_expert_->addInstance(plansys2::Instance{"unknown_point", "waypoint"});

    problem_expert_->addInstance(plansys2::Instance{"s_box_1", "box"});
    problem_expert_->addInstance(plansys2::Instance{"s_box_2", "box"});
    problem_expert_->addInstance(plansys2::Instance{"s_box_3", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_1", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_2", "box"});
    problem_expert_->addInstance(plansys2::Instance{"m_box_3", "box"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at small_robot_1 unknown_point)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone small_robot_1 small_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot small_robot_1)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity small_robot_1) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load small_robot_1) 0)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at medium_robot_1 m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone medium_robot_1 medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot medium_robot_1)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity medium_robot_1) 3)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load medium_robot_1) 0)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at big_robot_1 warehouse_2_sh)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_zone big_robot_1 inter_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(idle_robot big_robot_1)"));
    problem_expert_->addFunction(plansys2::Function("(= (robot_capacity big_robot_1) 5)"));
    problem_expert_->addFunction(plansys2::Function("(= (current_robot_load big_robot_1) 0)"));

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

    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone m_central medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_1 medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_2 medium_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(waypoint_from_zone m_sh_3 medium_zone)"));

    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone warehouse_2_sh inter_zone)"));

    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone common_zone inter_zone)"));
    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone common_zone small_zone)"));
    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone common_zone medium_zone)"));

    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone unknown_point inter_zone)"));
    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone unknown_point small_zone)"));
    problem_expert_->addPredicate(
        plansys2::Predicate("(waypoint_from_zone unknown_point medium_zone)"));

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

    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point s_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point s_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point s_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point s_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point m_sh_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point m_sh_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point m_sh_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point m_central)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point warehouse_2_sh)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected unknown_point common_zone)"));

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
    problem_expert_->addFunction(
        plansys2::Function("(= (distance_s common_zone warehouse_2_sh) 80)"));

    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_1 common_zone) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_2 common_zone) 110)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_sh_3 common_zone) 120)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s s_central common_zone) 100)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_1 common_zone) 55)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_2 common_zone) 60)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_sh_3 common_zone) 52)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s m_central common_zone) 55)"));
    problem_expert_->addFunction(
        plansys2::Function("(= (distance_s warehouse_2_sh common_zone) 80)"));

    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point s_sh_1) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point s_sh_2) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point s_sh_3) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point s_central) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point m_sh_1) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point m_sh_2) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point m_sh_3) 50)"));
    problem_expert_->addFunction(plansys2::Function("(= (distance_s unknown_point m_central) 50)"));
    problem_expert_->addFunction(
        plansys2::Function("(= (distance_s unknown_point warehouse_2_sh) 50)"));
    problem_expert_->addFunction(
        plansys2::Function("(= (distance_s unknown_point common_zone) 50)"));

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
    check_actions_state();
    /////////////////////////////// GOAL ////////////////////////////////////
    plansys2::Goal actual_goal = problem_expert_->getGoal();

    if (parser::pddl::toString(actual_goal) != goal_) {
      if (idle_system){
        RCLCPP_INFO(get_logger(), "Starting new plan");
        problem_expert_->setGoal(plansys2::Goal(goal_));
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);
        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal "
                    << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          exit(0);
        }
        if (!executor_client_->start_plan_execution(plan.value())) {
          RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
        }
        idle_system = false;
      } else
      {
      bool result = handleGoalChange();
      if (!result) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Goal changed");
      }
    }
    // /////////////////////////////////////////////////////////////////////////
    ////////////////////////////// DEAD ROBOT ///////////////////////////////
    if (dead_flag_ && rand() % 10 == 0)
    {
      RCLCPP_ERROR(get_logger(), "Robot dead");
      robots_list_.clear();
      robots_list_ = add_robot_->get_robots_list();
      std::srand(std::time(nullptr));  // solo una vez por ejecución

      int index = std::rand() % robots_list_.size();
      std::cout << "Robots list: ";
      for (const auto& robot : robots_list_)
      {
        std::cout << robot << " ";
      }
      // std::vector<std::string> list= { "big_robot_1","medium_robot_1","small_robot_1" };
      handleDeadRobot(robots_list_[index]);
      // robot_deads++;
      // if (robot_deads == 3){
      dead_flag_ = false;
      // }
    }
    /////////////////////////////////////////////////////////////////////////
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
        idle_system = true;
        // exit(0);
        
      } else {
        // exit(1);
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);
        if (new_robot_flag_) {
          problem_checker_->get_necessary_predicates();
          new_robot_flag_ = false;
        }
        bool problem_ok = problem_checker_->check_problem();
        if (problem_ok) {
          RCLCPP_INFO(get_logger(), "Problem OK");
        } else {
          RCLCPP_ERROR(get_logger(), "Problem NOT OK");
        }

        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal "
                    << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          exit(0);
        }
        if (!executor_client_->start_plan_execution(plan.value())) {
          RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
        }
      }
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_suscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cancel_publisher_;
  std::shared_ptr<ProblemChecker> problem_checker_;
  std::shared_ptr<AddRobot> add_robot_;
  std::string goal_;
  bool dead_flag_ = true;
  bool new_robot_flag_ = false;
  std::vector<std::string> robots_list_;
  int robot_deads = 0;
  bool idle_system = false;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  void check_actions_state()
  {
    auto feedback = executor_client_->getFeedBack();

    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_NOT_EXECUTED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_EXECUTING;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_FAILED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_SUCCEEDED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_CANCELLED;

    for (const auto & action_feedback : feedback.action_execution_status) {
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

    RCLCPP_INFO(get_logger(), "ACTIONS STATE ");

    for (const auto & action : action_EXECUTING) {
      RCLCPP_INFO(get_logger(), "Action %s EXECUTING", action.action_full_name.c_str());
    }
    for (auto & action : action_FAILED) {
      RCLCPP_INFO(get_logger(), "Action %s FAILED", action.action_full_name.c_str());
      // problem_checker_->restore_action(action);
    }
    if (!action_FAILED.empty()) {
      executor_client_->cancel_plan_execution();
      revert_actions();
      RCLCPP_INFO(get_logger(), "Actions reverted after failure");
      action_FAILED.clear();
    }
  }

  void revert_actions()
  {
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_EXECUTING;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_FAILED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_CANCELLED;

    auto feedback = executor_client_->getFeedBack();

    for (const auto & action_feedback : feedback.action_execution_status) {
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
        action_CANCELLED.push_back(action_feedback);
      }
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
        action_FAILED.push_back(action_feedback);
      }
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        action_EXECUTING.push_back(action_feedback);
      }
    }

    if (!action_CANCELLED.empty()) {
      RCLCPP_INFO(get_logger(), "Actions CANCELLED");
      for (auto & action : action_CANCELLED) {
        RCLCPP_ERROR(get_logger(), "Action %s CANCELLED", action.action_full_name.c_str());
        problem_checker_->restore_action(action);
      }
      action_CANCELLED.clear();
    }
    if (!action_FAILED.empty()) {
      RCLCPP_INFO(get_logger(), "Actions FAILED");
      for (auto & action : action_FAILED) {
        RCLCPP_ERROR(get_logger(), "Action %s FAILED", action.action_full_name.c_str());
        problem_checker_->restore_action(action);
      }
      action_FAILED.clear();
    }
    if (!action_EXECUTING.empty()) {
      RCLCPP_INFO(get_logger(), "Actions EXECUTING");
      for (auto & action : action_EXECUTING) {
        RCLCPP_ERROR(get_logger(), "Action %s EXECUTING", action.action_full_name.c_str());
        problem_checker_->restore_action(action);
      }
      action_EXECUTING.clear();
    }
  }

  bool handleGoalChange()
  {
    RCLCPP_INFO(get_logger(), "Goal changed");
    // cancelar plan actual
    RCLCPP_ERROR(get_logger(), "Cancelling plan");
    if (!executor_client_) {
      RCLCPP_ERROR(get_logger(), "executor_client_ is NULL in handleGoalChange!");
      return false;
    }
    executor_client_->cancel_plan_execution();
    RCLCPP_ERROR(get_logger(), "Plan cancelled");

    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_EXECUTING;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_FAILED;
    std::vector<plansys2_msgs::msg::ActionExecutionInfo> action_CANCELLED;

    revert_actions();

    auto cancel_msg = std_msgs::msg::String();
    cancel_msg.data = "plan_cancelled";
    cancel_publisher_->publish(cancel_msg);
    if (goal_.empty()) {
      RCLCPP_ERROR(get_logger(), "Empty goal waiting until valid goal exists.");
      return false;
    }
    problem_expert_->setGoal(plansys2::Goal(goal_));
    RCLCPP_INFO(get_logger(), "Goal set successfully: a%sa", goal_.c_str());

    return true;
  }

  void handleDeadRobot(std::string robot_name)
  {
    RCLCPP_ERROR(get_logger(), "Cancelling plan");
    if (!executor_client_) {
      RCLCPP_ERROR(get_logger(), "executor_client_ is NULL in handleGoalChange!");
      return;
    }
    executor_client_->cancel_plan_execution();

    std::array<std::string, 2> robot_info = add_robot_->get_robot_info(robot_name);
    std::string robot_type = robot_info[0];
    int robot_id = std::stoi(robot_info[1]);
    revert_actions();
    if (robot_id > 1) {
      RCLCPP_ERROR(get_logger(), "NO MORE ROBOTS OF THIS TYPE AVAILABLE");
    } else {
      std::string robot_name_new = robot_type + "_robot_" + std::to_string(robot_id + 1);
      std::string robot_zone;
      if (robot_type != "big") {
        robot_zone = robot_info[0] + "_zone";
      } else {
        robot_zone = "inter_zone";
      }

      std::string robot_init_wp = "unknown_point";

      int result = add_robot_->add_robot(robot_name_new, robot_init_wp, robot_zone);
      if (result == 0) {
        RCLCPP_INFO(get_logger(), "Robot %s added successfully", robot_name_new.c_str());
        new_robot_flag_ = true;
      } else {
        RCLCPP_ERROR(get_logger(), "Error adding robot %s", robot_name_new.c_str());
      }
    }
    bool add_result = add_robot_->delete_robot(robot_name);
    if (add_result) {
      RCLCPP_INFO(get_logger(), "Robot %s deleted successfully", robot_name.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Error deleting robot %s", robot_name.c_str());
    }
  }
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
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
