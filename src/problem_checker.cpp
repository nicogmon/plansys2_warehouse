#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <random>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std::chrono_literals;

class ProblemChecker{
public:
    ProblemChecker(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert){
        problem_expert_ = problem_expert;
    }
        

    void check_problem()
    {
        // Abre el archivo de salida en modo escritura (si no existe, se crea)
        std::ofstream outFile("predicates.txt");

        if (!outFile.is_open()) {
            std::cerr << "Error al abrir el archivo para escribir." << std::endl;
            return;
        }
        predicates_ = problem_expert_->getPredicates();
        outFile << "Predicates:" << std::endl;  // Escribe en el archivo

        for (const auto &predicate : predicates_) {
            if(predicate.name == "connected" || predicate.name == "robot_zone" || predicate.name == "waypoint_from_zone"){
                continue;
            }
            for (int i = 0; i < predicate.parameters.size(); i++) {
                if (i == 0) {
                    outFile << "(" << predicate.name << " ";  // Escribe en el archivo
                }
                outFile << predicate.parameters[i].name;  // Escribe en el archivo

                if (i < predicate.parameters.size() - 1) {
                    outFile << " ";  // Espacio entre parámetros
                }

                if (i == predicate.parameters.size() - 1) {
                    outFile << ")" << std::endl;  // Cierra el paréntesis y escribe en el archivo
                }
            }
        }

        // Cierra el archivo
        outFile.close();
        // std::cout << "Predicates:" << std::endl;
        // for (const auto &predicate : predicates_) {
        //     for (int i = 0; i < predicate.parameters.size(); i++) {
        //         if (i == 0) {
        //             std::cout << "(" << predicate.name << " ";
        //         }
        //         std::cout << predicate.parameters[i].name;
        //         if (i < predicate.parameters.size() - 1) {
        //             std::cout << " " ;
        //         }
        //         if (i == predicate.parameters.size() - 1) {
        //             std::cout << ")" << std::endl;
        //         }
        //     }
        // }
    }
private:
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::vector<plansys2::Predicate> predicates_;
};


