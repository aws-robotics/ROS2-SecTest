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
#include <vector>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  Component();
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
  void infinite_sum_loop() const;

  /// Timer controlling how often we spawn another thread.
  rclcpp::TimerBase::SharedPtr timer_;

  /// List of threads in use
  std::vector<std::thread> threads_;
};

}  // namespace cpu 
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__RESOURCES__CPU__COMPONENT_HPP_
