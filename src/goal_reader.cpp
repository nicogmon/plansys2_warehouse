#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class Goal_reader : public rclcpp::Node {
 public:
  Goal_reader() : Node("goal_reader"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/goal", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Goal_reader::timer_callback, this));
  }

  void timer_callback() {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("plansys2_warehouse");
    // std::cout << "package_share_directory: " << package_share_directory << std::endl;
    std::ifstream file(package_share_directory + "/config/Goals.txt");

    if (!file.is_open()) {
      std::cerr << "Error: No se pudo abrir el archivo Goals.txt" << std::endl;
      return;
    }
    std::ostringstream p_goal;
    p_goal << "(and ";  // Inicio del AND

    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty()) {  // Evitar líneas vacías
        p_goal << "(" << line << ")";
      }
    }
    p_goal << ")";  // Cierre del AND
    std::string f_goal = p_goal.str();
    // RCLCPP_INFO(get_logger(), "Goal: %s", f_goal.c_str());
    std_msgs::msg::String msg;
    msg.data = f_goal;
    publisher_->publish(msg);

    // cerrar fichero
    file.close();
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Goal_reader>());
  rclcpp::shutdown();
  return 0;
}
