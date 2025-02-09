cmake_minimum_required(VERSION 3.5)
project(plansys2_warehouse)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)





# add_library(move_node SHARED src/behavior_tree_nodes/Move.cpp)
# list(APPEND plugin_libs move_node)

# add_library(load_node SHARED src/Load_box_action_node.cpp)
# list(APPEND plugin_libs load_node)

# add_library(unload_node SHARED src/behavior_tree_nodes/Unload_box.cpp)
# list(APPEND plugin_libs unload_node)

add_library(problem_checker SHARED src/problem_checker.cpp)
ament_target_dependencies(problem_checker ${dependencies})



foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()#revisar si hace falta behavior trees

add_executable(move_node src/Move_action_node.cpp)
ament_target_dependencies(move_node ${dependencies})

add_executable(load_node src/Load_box_action_node.cpp)
ament_target_dependencies(load_node ${dependencies})

add_executable(unload_node src/Unload_box_action_node.cpp)
ament_target_dependencies(unload_node ${dependencies})

add_executable(warehouse_controller_node src/warehouse_controller_node.cpp)
ament_target_dependencies(warehouse_controller_node ${dependencies})
target_link_libraries(warehouse_controller_node problem_checker)

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

add_executable(goal_reader_node src/goal_reader.cpp)
ament_target_dependencies(goal_reader_node ${dependencies})



install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_node
  load_node
  unload_node
  
  warehouse_controller_node
  nav2_sim_node
  goal_reader_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
