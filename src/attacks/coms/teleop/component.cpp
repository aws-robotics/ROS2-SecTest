// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "ros_sec_test/attacks/coms/teleop/component.hpp"

#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"
#include "geometry_msgs/msg/twist.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros_sec_test
{
namespace attacks
{
namespace coms
{
namespace teleop
{

Component::Component()
: rclcpp_lifecycle::LifecycleNode("teleop", "", rclcpp::NodeOptions().use_intra_process_comms(true))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  const int queue_size = 10;
  pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(queue_size)));
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  pub_->on_activate();
  thread_ = std::thread([this] {run_();});
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  pub_.reset();
  thread_.join();
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  thread_.join();
  return CallbackReturn::SUCCESS;
}

void Component::run_()
{
  int key = ' ';
  std::unique_ptr<geometry_msgs::msg::Twist> twist = std::make_unique<geometry_msgs::msg::Twist>();
  std::map<int, std::vector<float>> speedBindings
  {
    {'w', {1, 0}},
    {'a', {0, 1}},
    {'s', {-1, 0}},
    {'d', {0, -1}},
  };
  float y = 0;
  float th = 0;
  printf("Enter commands\n");
  while (true) {
    printf("Getting command\n");
    key = get_char_();
    printf("Received key %c\n", static_cast<char>(key));
    if (key == '\x03') {
      printf("Exiting\n");
      break;
    } else {
      if (speedBindings.count(key) == 1) {
        y = speedBindings[key][0];
        th = speedBindings[key][1];
      } else {
        printf("Command is invalid\n");
      }
      twist->linear.y = y;
      twist->angular.z = th;
      pub_->publish(std::move(twist));
    }
  }
}

int Component::get_char_()
{
  // Copied from https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
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

}  // namespace teleop
}  // namespace coms
}  // namespace attacks
}  // namespace ros_sec_test

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::coms::teleop::Component,
  rclcpp_lifecycle::LifecycleNode)
