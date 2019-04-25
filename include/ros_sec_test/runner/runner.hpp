#include <memory>

#include "ros_sec_test/attacks/noop/component.hpp"
//#include "ros_sec_test/attacks/resources/cpu/component.hpp"
//#include "ros_sec_test/attacks/resources/disk/component.hpp"
//#include "ros_sec_test/attacks/resources/memory/component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"
#include "ros_sec_test/runner/lifecycle_service_client.hpp"

class Runner: public rclcpp::Node
{
public:
    Runner(const std::string &);
    void spin();
    virtual void initialize_client_vector();
protected:
    //virtual void initialize_node_vector();
private:
    //rclcpp::NodeOptions options_;
    std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes_;
    std::vector<std::shared_ptr<LifecycleServiceClient>> lc_clients_;
};  