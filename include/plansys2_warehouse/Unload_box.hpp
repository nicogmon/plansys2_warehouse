#ifndef PLANSYS2_WAREHOUSE__UNLOAD_BOX_HPP_
#define PLANSYS2_WAREHOUSE__UNLOAD_BOX_HPP_



#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace plansys2_warehouse
{

class Unload_box : public plansys2::ActionExecutorClient
{
public:
  Unload_box();

private:

  void do_work() override;
  int counter_;
};

}  // namespace plansys2_warehouse
#endif  // PLANSYS2_WAREHOUSE__BEHAVIOR_TREE_NODES__UNLOAD_BOX_HPP_