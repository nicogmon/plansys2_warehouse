#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include <memory>
#include <string>
#include <iostream>
#include <set>


class AddRobot
{
public:
  // Constructor: recibe un puntero compartido al problem expert
  AddRobot(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
  : problem_expert_(problem_expert)
  {}

  // Método principal: añade predicados relacionados con el robot
  int add_robot(const std::string & robot_id, const std::string & initial_location, const std::string & robot_zone)
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
    result &= problem_expert_->addPredicate(plansys2::Predicate("(robot_at " + robot_id + " " + initial_location + ")"));
    result &= problem_expert_->addPredicate(plansys2::Predicate("(robot_zone " + robot_id + " " + robot_zone + ")"));
    result &= problem_expert_->addPredicate(plansys2::Predicate("(idle_robot " + robot_id + ")"));
    result &= problem_expert_->addFunction(plansys2::Function("(= (robot_capacity " + robot_id + ") " + robot_capacity + ")"));
    result &= problem_expert_->addFunction(plansys2::Function("(= (current_robot_load " + robot_id + ") 0)"));

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

    return result ? 0 : -1;
  }


  std::array<std::string, 2> get_robot_info(const std::string & robot_id)
  {
    std::array<std::string, 2> robot_info;
    

    // Separar usando el guion bajo como delimitador
    std::stringstream ss(robot_id);
    std::string token;
    std::vector<std::string> parts;

    while (std::getline(ss, token, '_')) {
        parts.push_back(token);
    }

    if (parts.size() >= 3) {
        robot_info[0] = parts[0];               // "small"
        robot_info[1] = parts[2];                // "1"
    } else {
        std::cerr << "Formato incorrecto." << std::endl;
        return {"", ""};
    }

    std::cout << "Primera parte: " << robot_info[0] << std::endl;
    std::cout << "Última parte: " << robot_info[1] << std::endl;

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

