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
#ifndef ROS_SEC_TEST__ATTACKS__RESOURCES__MEMORY__COMPONENT_HPP_
#define ROS_SEC_TEST__ATTACKS__RESOURCES__MEMORY__COMPONENT_HPP_
#include <mutex>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcpputils/thread_safety_annotations.hpp"

#include "ros_sec_test/attacks/periodic_attack_component.hpp"

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace memory
{

/// This attack tries to allocate as much physical memory as possible.
/**
 * WARNING: this attack will try to fill your RAM.
 *
 * The attack will slowly allocate memory until
 *  the RAM is full.
 *
 * When the node is killed it will attempt to free all
 * allocated memory
 */
class Component : public ros_sec_test::attacks::PeriodicAttackComponent
{
public:
  Component();
  explicit Component(std::size_t max_memory);
  ~Component();
  Component(const Component &) = delete;
  Component & operator=(const Component &) = delete;

private:
  /// Allocate 10MiB of memory.
  void run_periodic_attack() final;

  /// Stop allocaing new memory and free already allocated memory
  void terminate_attack_and_cleanup_resources() final;

  /// Amount of memory currently allocated in bytes
  size_t num_bytes_allocated_;

  /// Maximum amount of memory to allocate in bytes
  const std::size_t max_num_bytes_allocated_;

  /// Allocated memory block locked to physical memory
  void * memory_block_;
};

}  // namespace memory
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__RESOURCES__MEMORY__COMPONENT_HPP_
