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
#ifndef ROS_SEC_TEST__ATTACKS__RESOURCES__DISK__COMPONENT_HPP_
#define ROS_SEC_TEST__ATTACKS__RESOURCES__DISK__COMPONENT_HPP_
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros_sec_test/attacks/periodic_attack_component.hpp"

namespace ros_sec_test
{
namespace attacks
{
namespace resources
{
namespace disk
{

/// This attack tries to fill the local disk with data.
/**
 * WARNING: this attack will try to fill your disk completely!
 *
 * The attack will slowly grow a single file by chunk of 100 MiB until
 * the disk is full.
 *
 * When the node shutdowns, the node will attempt to delete the created
 * file to restore the system to its initial state.
 */
class Component : public ros_sec_test::attacks::PeriodicAttackComponent
{
public:
  Component();
  Component(const Component &) = delete;
  Component & operator=(const Component &) = delete;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /* state */) final;

private:
  /// Grow the file size by 100MiB.
  void run_periodic_attack() override;

  /// File descriptor to the large file this attack tries to allocate.
  int fd_;
};

}  // namespace disk
}  // namespace resources
}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__RESOURCES__DISK__COMPONENT_HPP_
