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


namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace memory
{

/// This attack tries to allocate all available memory.
/**
 * WARNING: this attack will try to fill your RAM.
 *
 * The attack will slowly allocate memory until
 *  the RAM is full.
 *
 * When the node is killed it will attempt to free all
 * allocated memory
 */
class Component : public rclcpp_lifecycle::LifecycleNode
{
public:
  Component();
  explicit Component(std::size_t max_memory);
  Component(const Component &) = delete;
  Component & operator=(const Component &) = delete;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /* state */) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /* state */) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /* state */) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & /* state */) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & /* state */ state) final;

private:
  /// Allocate 10MiB of memory.
  void run_periodic_attack();

  /// Stop allocaing new memory and free aady allocated memory
  void terminate_attack_and_clear_resources();

  /// Maximum amount of memory to allocate
  const std::size_t max_memory_;

  // Manages thread safety for vec_
  mutable std::mutex mutex_;

  /// Timer controlling how often we allocate more memory.
  rclcpp::TimerBase::SharedPtr timer_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// Keep track of allocatedmemory
  std::vector<std::vector<int>> vec_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace memory
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__RESOURCES__MEMORY__COMPONENT_HPP_
