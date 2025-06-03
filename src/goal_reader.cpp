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
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Goal_reader
  : public rclcpp::Node
{
public:
  Goal_reader()
  : Node("goal_reader"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/goal", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Goal_reader::timer_callback, this));
  }

  void timer_callback()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("plansys2_warehouse");
    std::ifstream file(package_share_directory + "/config/Goals.txt");

    if (!file.is_open()) {
      std::cerr << "Error: No se pudo abrir el archivo Goals.txt" << std::endl;
      return;
    } else if (file.peek() == std::ifstream::traits_type::eof()) {
      std::cout << "El fichero está vacío.\n";
      std_msgs::msg::String msg;
      msg.data = "";
      publisher_->publish(msg);
      return;
    }
    std::ostringstream p_goal;
    p_goal << "(and ";

    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty()) {
        p_goal << "(" << line << ")";
      }
    }
    p_goal << ")";
    std::string f_goal = p_goal.str();

    std_msgs::msg::String msg;
    msg.data = f_goal;
    publisher_->publish(msg);

    file.close();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Goal_reader>());
  rclcpp::shutdown();
  return 0;
}
