#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"
#include "ros_sec_test/runner/lifecycle_service_client.hpp"

class Runner : public rclcpp::Node
{
public:
  Runner(const std::string &, std::shared_ptr<std::vector<std::string>>);

  Runner(const Runner &) = delete;
  Runner & operator=(const Runner &) = delete;

  void spin();
  virtual void initialize_client_vector();

private:
  std::shared_ptr<std::vector<std::string>> nodes_;
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes_;
  std::vector<std::shared_ptr<LifecycleServiceClient>> lc_clients_;
};
