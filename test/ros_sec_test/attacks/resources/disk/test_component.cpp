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
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "ros_sec_test/attacks/resources/disk/component.hpp"
#include "test_utilities/utility_fixtures.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

using DiskNode = ros_sec_test::attacks::resources::disk::Component;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using ros_sec_test::test::test_utilities::NodeConfigurationFixture;

TEST_F(NodeConfigurationFixture, check_configuring_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
}

TEST_F(NodeConfigurationFixture, check_activating_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
}

TEST_F(NodeConfigurationFixture, check_deactivating_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
}

TEST_F(NodeConfigurationFixture, check_cleaningup_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
}

TEST_F(NodeConfigurationFixture, check_unconfigured_shuttingdown_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP));
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_FINALIZED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
}

TEST_F(NodeConfigurationFixture, check_active_shuttingdown_transition) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_FINALIZED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVE_SHUTDOWN)).id());
}

TEST_F(NodeConfigurationFixture, check_full_node_lifecycle) {
  auto attack_node = std::make_shared<DiskNode>();
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  ASSERT_EQ(State::PRIMARY_STATE_FINALIZED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
