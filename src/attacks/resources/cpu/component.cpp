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
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "rcpputils/thread_safety_annotations.hpp"

#include "ros_sec_test/attacks/periodic_attack_component.hpp"

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using PeriodicAttackComponent = ros_sec_test::attacks::PeriodicAttackComponent;

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace cpu
{
// By default start with twice the maximum supported number of threads. Hardware concurrency
// isn't guaranteed to be accurate so the maximum number of threads is doubled to avoid
// the attack failing.
Component::Component()
: Component(2 * std::thread::hardware_concurrency()) {}

Component::Component(std::size_t max_num_threads)
: PeriodicAttackComponent("resources_cpu"),
  max_num_threads_(max_num_threads),
  threads_() {}

void Component::terminate_attack_and_cleanup_resources() RCPPUTILS_TSA_REQUIRES(!mutex_)
{
  PeriodicAttackComponent::terminate_attack_and_cleanup_resources();
  std::vector<std::thread> threads;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    threads = std::move(threads_);
  }
  for (auto & thread : threads) {
    thread.join();
  }
  threads.clear();
}

void Component::run_periodic_attack() RCPPUTILS_TSA_REQUIRES(!mutex_)
{
  auto thread = std::thread([this] {consume_cpu_resources();});
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (max_num_threads_ > threads_.size()) {
      threads_.push_back(std::move(thread));
    }
  }
}


void Component::consume_cpu_resources()
{
  int i_sum = 0;
  for (int i = 0;; i++) {
    if (!rclcpp::ok()) {
      return;
    }
    // It's fine if this overflows
    i_sum += i;
    // Prevent optimization
    asm ("nop");
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
