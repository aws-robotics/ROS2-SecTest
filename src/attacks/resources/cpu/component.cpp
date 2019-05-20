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
#include "ros_sec_test/attacks/resources/cpu/component.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace cpu 
{

Component::Component()
: rclcpp_lifecycle::LifecycleNode(
    "cpu_attacker", "", rclcpp::NodeOptions().use_intra_process_comms(
      true)), threads_(){}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  timer_ = this->create_wall_timer(
    1s, [this] {run_periodic_attack();});
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  for (auto &thread: threads_) {
      thread.join();
  }
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  timer_.reset();
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  //Join threads in case cleanup wasn't called before
  for (auto &thread: threads_) {
      thread.join();
  }
  return CallbackReturn::SUCCESS;
}

void Component::run_periodic_attack_()
{
  threads_.emplace_back(std::thread([this]{infinite_sum_loop();}));  
}

void Component::infinite_sum_loop() const
{
  int i, i_sum = 0;
  for (i=0; ; i++) {
    if (!rclcpp::ok()) {
      return;
    }
    //It's fine if this overflows
    i_sum += i;
    //Prevent optimization
    asm("nop");
  }
}

}  // namespace cpu 
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::resources::cpu::Component,
  rclcpp_lifecycle::LifecycleNode)
