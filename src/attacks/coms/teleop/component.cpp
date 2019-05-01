#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcutils/logging_macros.h"

#include "geometry_msgs/msg/twist.hpp"

#include "ros_sec_test/attacks/coms/teleop/component.hpp"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

namespace ros_sec_test {
namespace attacks {
namespace coms {
namespace teleop{

Component::Component()
    : rclcpp_lifecycle::LifecycleNode("teleop", "", true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 10;
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", custom_qos_profile);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_activate(const rclcpp_lifecycle::State &) {
  // Log something
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    pub_->on_activate();
    char key(' ');
    std::shared_ptr<geometry_msgs::msg::Twist> twist = std::make_shared<geometry_msgs::msg::Twist>();
    std::map<char, std::vector<float>> speedBindings
    {
      {'w', {1, 0}},
      {'a', {0, 1}},
      {'s', {-1, 0}},
      {'d', {0, -1}},
    };
    int y = 0;
    int th = 0;
    printf("Enter commands\n");
    while(true) {
        //printf("Getting command\n");
        key = get_char_();
        //std::cout << "Received key " << key << "\n";
        if (key == '\x03') {
            printf("Exiting\n");
            break;
        } else {
            if (speedBindings.count(key) == 1) {
                y = speedBindings[key][0];
                th = speedBindings[key][1];
            } else {
                //printf("Command is invalid\n");
            }
            twist->linear.y = y;
            twist->angular.z = th;
            pub_->publish(twist);
        }

    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_deactivate(const rclcpp_lifecycle::State &) {
  // Log something
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_cleanup(const rclcpp_lifecycle::State &) {
  // Log something
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_shutdown(const rclcpp_lifecycle::State & state) {
  // Log something
    RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
int Component::get_char_(){
    //copied from https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    //Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}
}  //namespace teleop
}  // namespace noop
}  // namespace attacks
}  // namespace ros_sec_test


#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::coms::teleop::Component, rclcpp_lifecycle::LifecycleNode)