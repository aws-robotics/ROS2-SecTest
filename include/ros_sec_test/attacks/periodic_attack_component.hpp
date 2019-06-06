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
#ifndef ROS_SEC_TEST__ATTACKS__PERIODIC_ATTACK_COMPONENT_HPP_
#define ROS_SEC_TEST__ATTACKS__PERIODIC_ATTACK_COMPONENT_HPP_
#include <mutex>
#include <string>
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

/**
 * Base blass for attacks that follow a pattern of executing
 * an attack periodicially base on a timer
 */
class PeriodicAttackComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  PeriodicAttackComponent();
  explicit PeriodicAttackComponent(std::string node_name);
  PeriodicAttackComponent(const PeriodicAttackComponent &) = delete;
  PeriodicAttackComponent & operator=(const PeriodicAttackComponent &) = delete;
  virtual ~PeriodicAttackComponent() = default;

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /* state */);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /* state */);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /* state */);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & /* state */);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & /* state */ state);

protected:
  /// Execute the attack.
  virtual void run_periodic_attack();

  /// Stop the attack timer and free any resources.
  virtual void terminate_attack_and_cleanup_resources();

  // Manages thread safety for attacks.
  mutable std::mutex mutex_;

  /// Timer controlling how often the attack is run.
  rclcpp::TimerBase::SharedPtr timer_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__PERIODIC_ATTACK_COMPONENT_HPP_
