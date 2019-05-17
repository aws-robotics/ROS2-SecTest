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
#ifndef ROS_SEC_TEST__ATTACKS__FACTORY_UTILS_HPP_
#define ROS_SEC_TEST__ATTACKS__FACTORY_UTILS_HPP_
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros_sec_test
{
namespace attacks
{

/// Instantiates a lifecycle attack node based on its name.
rclcpp_lifecycle::LifecycleNode::SharedPtr build_attack_node_from_name(
  const std::string & attack_node_name);

}  // namespace attacks
}  // namespace ros_sec_test

#endif  // ROS_SEC_TEST__ATTACKS__FACTORY_UTILS_HPP_
