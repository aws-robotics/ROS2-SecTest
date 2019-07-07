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
#ifndef TEST_UTILITIES__PERIODIC_ATTACK_COMPONENT_MOCK_HPP_
#define TEST_UTILITIES__PERIODIC_ATTACK_COMPONENT_MOCK_HPP_
#include <string>

#include "gmock/gmock.h"

#include "ros_sec_test/attacks/periodic_attack_component.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


using PeriodicAttackComponent = ros_sec_test::attacks::PeriodicAttackComponent;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MockPeriodicAttackComponent : public PeriodicAttackComponent
{
public:
  explicit MockPeriodicAttackComponent(std::string node_name)
  : PeriodicAttackComponent(node_name) {}
  MOCK_METHOD1(on_configure, CallbackReturn(const rclcpp_lifecycle::State &));
  MOCK_METHOD1(on_activate, CallbackReturn(const rclcpp_lifecycle::State &));
  MOCK_METHOD1(on_deactivate, CallbackReturn(const rclcpp_lifecycle::State &));
  MOCK_METHOD1(on_cleanup, CallbackReturn(const rclcpp_lifecycle::State &));
  MOCK_METHOD1(on_shutdown, CallbackReturn(const rclcpp_lifecycle::State &));
};

#endif  // TEST_UTILITIES__PERIODIC_ATTACK_COMPONENT_MOCK_HPP_
