#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(rclcpp::Node * parent_node, const std::string & target_node_name);
  void init();
  unsigned int get_state(std::chrono::seconds time_out=std::chrono::seconds(3));
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out=std::chrono::seconds(5));

private:
  rclcpp::Node * parent_node_;
  std::string target_node_name_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

};