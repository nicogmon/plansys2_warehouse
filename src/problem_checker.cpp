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
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

using namespace std::chrono_literals;

class ProblemChecker{
public:
    ProblemChecker(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert){
        problem_expert_ = problem_expert;
    }
    void get_necesary_predicates()
    {
        std::ifstream file("/tmp/problem.pddl");
        std::ofstream outFile("neccesary_predicates.txt");
        if (!file.is_open()) {
            std::cerr << "Error: No se pudo abrir el archivo problem.pddl" << std::endl;
            return;
        }
        if (!outFile.is_open()) {
            std::cerr << "Error al abrir el archivo para escribir." << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.find("( robot_at ") != std::string::npos ||
                line.find("( box_at ") != std::string::npos ||
                line.find("( loaded_box)") != std::string::npos ||
                line.find("( idle_robot ") != std::string::npos ||
                line.find("(= ( current_robot_load ") != std::string::npos){

                std::istringstream iss(line);
                std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, 
                                                std::istream_iterator<std::string>{}};
                
                if (tokens.size() >= 3) {
                    
                    if (tokens[1] == "box_at" || tokens[1] == "loaded_box") {
                        // for (size_t i = 1; i < tokens.size(); ++i) {
                        //     if (tokens[i].find("_box") != std::string::npos) {
                        //         outFile << tokens[i] << std::endl;  // Escribe el predicado con el parámetro que contiene "box"

                        //         break;  // Solo imprimimos el primer parámetro que contiene "box"
                        //     }
                        // }
                        outFile << tokens[2] << std::endl; // Imprime $2
                    
                    }
                    else{
                        outFile << tokens[1] << " " << tokens[2] << std::endl; // Imprime $2 y $3
                    }
                }
            }
            else if (line.find("( :goal") != std::string::npos) {
                return; 
            }

        }
        return;
    }
        

    bool check_problem()
    {
        // Abre el archivo de salida en modo escritura (si no existe, se crea)
        std::ofstream outFile("predicates.txt");

        if (!outFile.is_open()) {
            std::cerr << "Error al abrir el archivo para escribir." << std::endl;
            return false;
        }
        predicates_ = problem_expert_->getPredicates();
        functions_ = problem_expert_->getFunctions();
        outFile << "Functions:" << std::endl;  
        for (const auto &function : functions_) {
            for (int i = 0; i < function.parameters.size(); i++) {
                if (i == 0) {
                    outFile << "(" << function.name << " ";  
                }
                outFile << function.parameters[i].name;  

                if (i < function.parameters.size() - 1) {
                    outFile << " ";  // Espacio entre parámetros
                }

                if (i == function.parameters.size() - 1) {
                    outFile <<" " <<  function.value << " )" << std::endl;  // Cierra el paréntesis y escribe en el archivo
                   
                }
            }

        }
        outFile << std::endl;  
        outFile << "Predicates:" << std::endl;  

        for (const auto &predicate : predicates_) {
            if(predicate.name == "connected" || predicate.name == "robot_zone" || predicate.name == "waypoint_from_zone"){
                continue;
            }
            for (int i = 0; i < predicate.parameters.size(); i++) {
                if (i == 0) {
                    outFile << "(" << predicate.name << " ";  
                }
                outFile << predicate.parameters[i].name;  

                if (i < predicate.parameters.size() - 1) {
                    outFile << " ";  // Espacio entre parámetros
                }

                if (i == predicate.parameters.size() - 1) {
                    outFile << ")" << std::endl;  // Cierra el paréntesis y escribe en el archivo
                }
            }
        }
        // tenemos la misma info en /tmp/problem pddl con goal y el resto de predicados.
        // Cierra el archivo
        outFile.close();

        // Compara los predicados necesarios con los del archivo problem.pddl
        return check_necessary_predicates("neccesary_predicates.txt", "predicates.txt");


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

    void restore_action(plansys2_msgs::msg::ActionExecutionInfo& action){
        std::cout << "Action: " << action.action << std::endl;
        for (const auto &arg : action.arguments) {
            std::cout << "Arg: " << arg << std::endl;
        }
        if (action.action == "load_box"){
            std::cout << "Restoring load_box" << std::endl;
            std::cout << "restoring predicates" << std::endl;
            std::cout << "(iddle_robot " + action.arguments[0] + ")" << std::endl;
            std::cout << "(box_at " + action.arguments[1] + " " + action.arguments[2] + ")" << std::endl;
            problem_expert_->addPredicate(plansys2::Predicate("(idle_robot " + action.arguments[0] + ")"));
            
            // problem_expert_->addPredicate(plansys2::Predicate("(robot_at " + action.arguments[0] + " " + action.arguments[1] + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(box_at " + action.arguments[1] + " " + action.arguments[2] + ")"));
            std::cout << "restoring functions" << std::endl;
        }
        if (action.action == "move"){
            std::cout << "Restoring move" << std::endl;
            std::cout << "restoring predicates" << std::endl;
            std::cout << "(robot_at " + action.arguments[0] + " " + action.arguments[1] + ")" << std::endl;
            problem_expert_->addPredicate(plansys2::Predicate("(robot_at " + action.arguments[0] + " " + action.arguments[1] + ")"));
            std::cout << "restoring functions" << std::endl;
        }
        if (action.action == "unload_box"){
            std::cout << "Restoring unload_box" << std::endl;
            std::cout << "restoring predicates" << std::endl;
            std::cout << "(iddle_robot " + action.arguments[0] + ")" << std::endl;
            std::cout << "(box_at " + action.arguments[1] + " " + action.arguments[2] + ")" << std::endl;
            problem_expert_->addPredicate(plansys2::Predicate("(idle_robot " + action.arguments[0] + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(box_at " + action.arguments[1] + " " + action.arguments[2] + ")"));
            std::cout << "restoring functions" << std::endl;
        }



    }
private:
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::vector<plansys2::Predicate> predicates_;
    std::vector<plansys2::Function> functions_;

    bool check_necessary_predicates(std::string input_file1, std::string input_file2){
        std::ifstream file1(input_file1);  // Primer archivo con "robot_at small_robot"
        std::ifstream file2(input_file2);  // Segundo archivo con "Predicates: (idle_robot small_robot)"
        
        if (!file1 || !file2) {
            std::cerr << "Error abriendo los archivos" << std::endl;
            return false;
        }

        std::vector<std::string> lines1, lines2;
        std::string line;

        // Leer todas las líneas del primer archivo en un vector
        while (std::getline(file1, line)) {
            lines1.push_back(line);
        }

        // Leer todas las líneas del segundo archivo en otro vector
        while (std::getline(file2, line)) {
            lines2.push_back(line);
        }

        // Comprobar si cada línea del primer archivo está contenida en alguna del segundo archivo
        for (const std::string& str1 : lines1) {
            bool found = false;
            for (const std::string& str2 : lines2) {
                if (str2.find(str1) != std::string::npos) {  // str1 está contenido en str2
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::cout << "No encontrado: " << str1 << std::endl;
                return false;
            }
        }

        return true;
    }
    
};


