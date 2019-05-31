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
#include "ros_sec_test/attacks/resources/memory/component.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

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

// Amount of additional memory in bytes to allocate each time the attack loop is allocated.
static constexpr std::size_t kBytesAllocatedPerAttack = 100 * 1024 * 1024;

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace memory
{

Component::Component()
: Component(SIZE_MAX) {}

Component::Component(std::size_t max_num_bytes_allocated)
: PeriodicAttackComponent("resources_memory"),
  num_bytes_allocated_(0),
  max_num_bytes_allocated_(max_num_bytes_allocated),
  memory_block_() {}

Component::~Component()
{
  free(memory_block_);
}

void Component::terminate_attack_and_cleanup_resources() RCPPUTILS_TSA_REQUIRES(!mutex_)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timer_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    munlock(memory_block_, num_bytes_allocated_);
    free(memory_block_);
  }
}

void Component::run_periodic_attack() RCPPUTILS_TSA_REQUIRES(!mutex_)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (num_bytes_allocated_ < max_num_bytes_allocated_) {
    size_t memory_to_allocate = std::min(num_bytes_allocated_ + kBytesAllocatedPerAttack,
        max_num_bytes_allocated_);
    void * new_memory_block = realloc(memory_block_, memory_to_allocate);
    if (new_memory_block == NULL) {
      RCLCPP_ERROR(get_logger(), "Failed to allocate memory.");
    } else {
      memory_block_ = new_memory_block;
      num_bytes_allocated_ = memory_to_allocate;
      int result = mlock(memory_block_, num_bytes_allocated_);
      if (0 != result) {
        RCLCPP_ERROR(get_logger(), "Error while trying to lock memory: %s ", std::strerror(errno));
      }
    }
  }
}


}  // namespace memory
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::resources::memory::Component,
  rclcpp_lifecycle::LifecycleNode)
