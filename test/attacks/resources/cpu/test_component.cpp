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

#include "ros_sec_test/attacks/resources/cpu/component.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

using CPUNode = ros_sec_test::attacks::resources::cpu::Component;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class ROSTestingFixture : public ::testing::Test
{
public:
  ROSTestingFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~ROSTestingFixture()
  {
    rclcpp::shutdown();
  }
};


class NodeConfigurationFixture : public ROSTestingFixture
{
protected:
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  std::promise<void> thread_promise_;
  std::shared_future<void> future_;

public:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_node");
    executor_.add_node(node_);
    future_ = thread_promise_.get_future();
  }

  void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    executor_.add_node(node);
  }

  void spin_executor_until(std::shared_future<void> & future)
  {
    using namespace std::chrono_literals;
    std::thread thread_spin([this, &future]() {
        RCLCPP_INFO(this->node_->get_logger(), "Spin thread started.");
        this->executor_.spin_until_future_complete(future, 1000ms);
        RCLCPP_INFO(this->node_->get_logger(), "Spin thread ended.");
      });
    thread_spin.join();
  }
  void start_executor()
  {
    spin_executor_until(future_);
  }
  void stop_executor()
  {
    thread_promise_.set_value();
  }
  void TearDown() override
  {
    stop_executor();
  }
};

TEST_F(NodeConfigurationFixture, check_configuring_transition) {
  auto attack_node = std::make_shared<CPUNode>(0);
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
}

TEST_F(NodeConfigurationFixture, check_activating_transition) {
  auto attack_node = std::make_shared<CPUNode>(0);
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
}

TEST_F(NodeConfigurationFixture, check_deactivating_transition) {
  auto attack_node = std::make_shared<CPUNode>(0);
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
  auto attack_node = std::make_shared<CPUNode>(0);
  add_node_to_executor(attack_node->get_node_base_interface());
  start_executor();
  attack_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, attack_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, attack_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
}

TEST_F(NodeConfigurationFixture, check_unconfigured_shuttingdown_transition) {
  auto attack_node = std::make_shared<CPUNode>(0);
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
  auto attack_node = std::make_shared<CPUNode>(0);
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
  auto attack_node = std::make_shared<CPUNode>(0);
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
