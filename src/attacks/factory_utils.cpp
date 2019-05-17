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
#include "ros_sec_test/attacks/factory_utils.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "ros_sec_test/attacks/noop/component.hpp"
#include "ros_sec_test/attacks/resources/disk/component.hpp"

using LifecycleNodeShPtr = std::shared_ptr<rclcpp_lifecycle::LifecycleNode>;
using LifecycleNodeConstructorCallback = std::function<LifecycleNodeShPtr()>;

using NoopNode = ros_sec_test::attacks::noop::Component;
using DiskNode = ros_sec_test::attacks::resources::disk::Component;

static const std::map<std::string, LifecycleNodeConstructorCallback>
kNodeNameToFactoryCallback = {
  {"noop", []() -> LifecycleNodeShPtr {return std::make_shared<NoopNode>();}},
  {"resources/disk", []() -> LifecycleNodeShPtr {return std::make_shared<DiskNode>();}},
};

namespace ros_sec_test
{
namespace attacks
{

rclcpp_lifecycle::LifecycleNode::SharedPtr build_attack_node_from_name(
  const std::string & attack_node_name)
{
  const auto it = kNodeNameToFactoryCallback.find(attack_node_name);
  if (it != kNodeNameToFactoryCallback.end()) {
    return it->second();
  }
  return rclcpp_lifecycle::LifecycleNode::SharedPtr();
}

}  // namespace attacks
}  // namespace ros_sec_test
