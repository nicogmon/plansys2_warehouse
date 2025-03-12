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

#include "plansys2_warehouse/Move.hpp"


using namespace std::chrono_literals;

namespace plansys2_warehouse
{
Move::Move()
: plansys2::ActionExecutorClient("move", 1s)
{

    declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>());
    get_parameter_or("specialized_arguments", specialized_arguments_, std::vector<std::string>({""}));

  
    RCLCPP_INFO(get_logger(), "Move created");
    auto esp_size = specialized_arguments_.size();
    RCLCPP_INFO(get_logger(), "Specialized arguments size: %ld", esp_size);
    RCLCPP_INFO(get_logger(), "Specialized argument: %s", specialized_arguments_[0].c_str());
    
    std::vector<std::string> wp_names;
    get_parameter_or("waypoints", wp_names, {});
    
    for (const auto & wp : wp_names) {
      // std::cout << "Waypoint: " << wp << std::endl;
      declare_parameter<std::vector<double>>("waypoint_coords." + wp);
      std::vector<double> coords;
      
      if (get_parameter_or("waypoint_coords." + wp, coords, {})) {
        if (coords.size() == 3) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.header.stamp = now();
          pose.pose.position.x = coords[0];
          pose.pose.position.y = coords[1];
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          
          waypoints_[wp] = pose;
        } else {
          RCLCPP_ERROR(get_logger(), "Waypoint [%s] has incorrect coordinate size!", wp.c_str());
        }
      } else {
        RCLCPP_WARN(get_logger(), "No coordinate configured for waypoint [%s]", wp.c_str());
      }
    }

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&Move::current_pos_callback, this, _1));

    cancel_sub_ = create_subscription<std_msgs::msg::String>(
      "/cancel", 10, std::bind(&Move::cancel_callback, this, _1));
  }


void Move::current_pos_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  current_pos_ = msg->pose.pose;
};

void Move::cancel_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Cancel received");
  if (future_navigation_goal_handle_.valid()) {
    // Obtener el goal_handle de manera segura
    auto goal_handle = future_navigation_goal_handle_.get();
    
    if (goal_handle) {
        // Verificar si el estado de la acción es válido antes de cancelarla
        auto status = goal_handle->get_status();
        
        // Comprobar que la acción está en ejecución antes de cancelarla
        if (status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
            RCLCPP_INFO(get_logger(), "Canceling goal...");
            navigation_action_client_->async_cancel_goal(goal_handle);
        }
    }
  }
  
};

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Move::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    std::string robot = get_arguments()[0];
    RCLCPP_INFO(get_logger(), "Robot: %s", robot.c_str());
    // specialized_arguments_ = get_arguments();
    // RCLCPP_INFO(get_logger(), "Specialized arguments: %s", robot.c_str());
    std::string service_name = robot + "/navigate_to_pose";
    RCLCPP_INFO(get_logger(), "Service: %s", service_name.c_str());

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      service_name);//añadir el namspace del robot  que realiza la accion robot + "/navigate_to_pose" 
                            //nav2_sim_node solo acepta en el generico opcion de hacer tres o que ese nodo lanze 3 serviciosç
                            //investigar specialized arguments para que el nodo sepa que robot es el que realiza la accion
                            //si quito barra y meto namespace ya es automatico
    
 
    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");
    for (auto argument : get_arguments()) {
      RCLCPP_INFO(get_logger(), "Argument: %s", argument.c_str());
    }

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
        
      };

    send_goal_options.result_callback = [this](auto result) {
       // Verifica si el resultado es válido
        std::cout << "Goal result received" << std::endl;
        std::cout << "Status: " << static_cast<int>(result.code) << std::endl;  // Imprime el estado de la acción
        //poner running a false si ha terminado bien
        //me llega si ha terminado bien o mal 
        //comprobar aqui si llega o que, si no llegara arriba
        //donde llegue mandar async_cancel_goal
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          std::cout << "Navigation succeeded!" << std::endl;
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          std::cout << "Navigation was aborted!" << std::endl;
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          std::cout << "Navigation was canceled!" << std::endl;

      } else {
          std::cout << "Unknown result code!" << std::endl;
      }
        //wait o get para comprobar confirmacion de cancel 
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
      //poner running a true
    return ActionExecutorClient::on_activate(previous_state);
  };

//meter on deactivate para cancelar la accion de navegacion 
// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//   Move::on_deactivate(const rclcpp_lifecycle::State & previous_state)
//   {
//     //comprobar si ha terminado mal con el flag running 
//     //y enviar el cancel a navegacion
//     return ActionExecutorClient::on_deactivate(previous_state);
//   };

double Move::getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }


void Move::do_work()
{
  
  // if (ActionExecutorClient::should_cancel_goal()) {
  //   RCLCPP_WARN(get_logger(), "Move action was canceled!");
    
  //   // if (future_navigation_goal_handle_.valid()) {
  //   //     auto goal_handle = future_navigation_goal_handle_.get();
  //   //     if (goal_handle) {
  //   //         navigation_action_client_->async_cancel_goal(goal_handle);
  //   //     }
  //   // }

  //   finish(false, 0.0, "Move action canceled");
  //   return;

}
}
// namespace plansys2_house_problem
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2_warehouse::Move>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
 