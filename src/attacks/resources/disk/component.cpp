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
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "ros_sec_test/attacks/resources/disk/component.hpp"

using namespace std::chrono_literals;

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace disk
{

Component::Component()
: rclcpp_lifecycle::LifecycleNode(
    "disk_attacker", "", rclcpp::NodeOptions().use_intra_process_comms(
    true)) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  std::cout << "on_configure called\n";
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  timer_ = this->create_wall_timer(
    1s, [this] {run_periodic_attack();});
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  fd_ = open("attack.dat", O_WRONLY | O_CREAT);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  timer_.reset();
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
Component::run_periodic_attack() const
{
  // Get file size
  struct stat stat_buf;
  int rc = fstat(fd_, &stat_buf);
  RCLCPP_INFO(get_logger(), "Current disk size %d", stat_buf.st_size);
  if (rc == 0) {
    // Allocate an additonal 100MB
    posix_fallocate(fd_, stat_buf.st_size, 100 * 1024 * 1024);
  }
  // we should stop the attack if the call fails.
}

}  // namespace disk
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::resources::disk::Component,
  rclcpp_lifecycle::LifecycleNode)
