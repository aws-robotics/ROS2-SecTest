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
#ifndef ROS_SEC_TEST__ATTACKS__RESOURCES__CPU__COMPONENT_HPP_
#define ROS_SEC_TEST__ATTACKS__RESOURCES__CPU__COMPONENT_HPP_
#include <mutex>
#include <thread>
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
namespace cpu
{

/// This attack tries to use all available CPU resources.
/**
 * WARNING: this attack will try to use all of your CPU resources!
 *
 * The attack will spawn threads doing arbitrary work.
 *
 * When the node shutdowns, the node will attempt to kill
 * the threads it is using.
 */
class Component : public rclcpp_lifecycle::LifecycleNode
{
public:
  Component()
  : Component(32) {}
  explicit Component(std::size_t max_num_threads);
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
  /// Spawn another thread.
  void run_periodic_attack();

  /// Run an infinite loop of arbitrary work
  static void consume_cpu_resources();

  /// Join threads, clear vector, reset timer
  void terminate_attack_and_cleanup_resources();

  /// Maximum number of threads to spawn
  std::size_t max_num_threads_;

  /// Manages thread safety for threads_
  mutable std::mutex mutex_;

  /// List of threads in use
  std::vector<std::thread> threads_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// Timer controlling how often we spawn another thread.
  rclcpp::TimerBase::SharedPtr timer_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace cpu
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__RESOURCES__CPU__COMPONENT_HPP_
