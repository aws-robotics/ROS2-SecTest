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
#include "ros_sec_test/attacks/periodic_attack_component.hpp"

#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

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

PeriodicAttackComponent::PeriodicAttackComponent(std::string node_name)
: rclcpp_lifecycle::LifecycleNode(
    node_name, "", rclcpp::NodeOptions().use_intra_process_comms(
      true)),
  mutex_(),
  timer_() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeriodicAttackComponent::on_configure(const rclcpp_lifecycle::State & /* state */)
RCPPUTILS_TSA_REQUIRES(!mutex_)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  auto callback = [this] {run_periodic_attack();};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timer_ = this->create_wall_timer(1s, std::move(callback));
  }
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeriodicAttackComponent::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeriodicAttackComponent::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeriodicAttackComponent::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  terminate_attack_and_cleanup_resources();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeriodicAttackComponent::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  terminate_attack_and_cleanup_resources();
  return CallbackReturn::SUCCESS;
}

void
PeriodicAttackComponent::terminate_attack_and_cleanup_resources()
{
  timer_.reset();
}

void
PeriodicAttackComponent::run_periodic_attack()
{}
}  // namespace attacks
}  // namespace ros_sec_test
