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
#include "ros_sec_test/attacks/noop/component.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros_sec_test
{
namespace attacks
{
namespace noop
{

Component::Component()
: rclcpp_lifecycle::LifecycleNode("noop", "", rclcpp::NodeOptions().use_intra_process_comms(true))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Component::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  return CallbackReturn::SUCCESS;
}

}  // namespace noop
}  // namespace attacks
}  // namespace ros_sec_test

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros_sec_test::attacks::noop::Component, rclcpp_lifecycle::LifecycleNode)
