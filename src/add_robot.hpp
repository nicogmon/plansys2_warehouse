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

#ifndef ADD_ROBOT_HPP
#define ADD_ROBOT_HPP

#include <iostream>
#include <memory>
#include <set>
#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

class AddRobot
{
public:
  explicit AddRobot(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
  : problem_expert_(problem_expert)
  {}

  int add_robot(
    const std::string & robot_id, const std::string & initial_location,
    const std::string & robot_zone)
  {
    if (robot_id.empty() || initial_location.empty() || robot_zone.empty()) {
      std::cerr << "Error: robot_id, initial_location or robot_zone is empty." << std::endl;
      return -1;
    }
    std::string robot_capacity;
    if (robot_id.find("small") != std::string::npos) {
      robot_capacity = "1";
    } else if (robot_id.find("medium") != std::string::npos) {
      robot_capacity = "3";
    } else if (robot_id.find("big") != std::string::npos) {
      robot_capacity = "5";
    } else {
      std::cerr << "Error: Unknown robot type." << std::endl;
      return -1;
    }

    bool result = true;

    result &= problem_expert_->addInstance(plansys2::Instance{robot_id, "robot"});
    result &= problem_expert_->addPredicate(
        plansys2::Predicate("(robot_at " + robot_id + " " + initial_location + ")"));
    result &= problem_expert_->addPredicate(
        plansys2::Predicate("(robot_zone " + robot_id + " " + robot_zone + ")"));
    result &= problem_expert_->addPredicate(plansys2::Predicate("(idle_robot " + robot_id + ")"));
    result &= problem_expert_->addFunction(
        plansys2::Function("(= (robot_capacity " + robot_id + ") " + robot_capacity + ")"));
    result &= problem_expert_->addFunction(
        plansys2::Function("(= (current_robot_load " + robot_id + ") 0)"));

    return result ? 0 : -1;
  }
  bool delete_robot(const std::string & robot_id)
  {
    if (robot_id.empty()) {
      std::cerr << "Error: robot_id is empty." << std::endl;
      return -1;
    }
    bool result = true;

    result &= problem_expert_->removeInstance(plansys2::Instance{robot_id, "robot"});

    return result;
  }

  std::array<std::string, 2> get_robot_info(const std::string & robot_id)
  {
    std::array<std::string, 2> robot_info;

    std::stringstream ss(robot_id);
    std::string token;
    std::vector<std::string> parts;

    while (std::getline(ss, token, '_')) {
      parts.push_back(token);
    }

    if (parts.size() >= 3) {
      robot_info[0] = parts[0];  // "small"
      robot_info[1] = parts[2];  // "1"
    } else {
      std::cerr << "Formato incorrecto." << std::endl;
      return {"", ""};
    }

    return robot_info;
  }

  std::vector<std::string> get_robots_list()
  {
    std::vector<std::string> robots;
    auto instances = problem_expert_->getInstances();
    for (const auto & instance : instances) {
      if (instance.type == "robot") {
        robots.push_back(instance.name);
      }
    }
    return robots;
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
};
#endif // ADD_ROBOT_HPP
