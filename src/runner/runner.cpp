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
#include "runner/runner.hpp"

#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros_sec_test/attacks/factory_utils.hpp"

using rclcpp::ParameterValue;
using LifecycleServiceClient = ros_sec_test::utilities::LifecycleServiceClient;
using ros_sec_test::attacks::build_attack_node_from_name;

static const char * const kAttackNodeNamesParameter = "attack_nodes";

// By default, do not run any attack and print a message explaining to the user how
// to use the node.
static const std::vector<std::string> kDefaultAttackNodeNames = {};

namespace ros_sec_test
{
namespace runner
{

Runner::Runner()
: node_(rclcpp::Node::make_shared("attacker_node", "",
    rclcpp::NodeOptions().use_intra_process_comms(true))),
  attack_nodes_(),
  executor_(),
  logger_(rclcpp::get_logger("Runner"))
{
  RCLCPP_INFO(logger_, "Initializing Runner");
  node_->declare_parameter(kAttackNodeNamesParameter, ParameterValue(kDefaultAttackNodeNames));
  executor_.add_node(node_);
  const auto node_names = retrieve_attack_nodes_names();
  if (node_names.empty()) {
    warn_user_no_attack_nodes_passed();
  } else {
    initialize_attack_nodes(node_names);
  }
}

Runner::Runner(const std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> & attack_nodes)
: node_(rclcpp::Node::make_shared("attacker_node", "",
    rclcpp::NodeOptions().use_intra_process_comms(true))),
  attack_nodes_(),
  executor_(),
  logger_(rclcpp::get_logger("Runner"))
{
  RCLCPP_INFO(logger_, "Initializing Runner");
  executor_.add_node(node_);
  if (attack_nodes.empty()) {
    warn_user_no_attack_nodes_passed();
  } else {
    for (const auto & attack_node : attack_nodes) {
      AttackNodeData node_data = {
        attack_node,
        std::make_shared<LifecycleServiceClient>(node_.get(), attack_node->get_name())
      };
      attack_nodes_.emplace_back(std::move(node_data));
      executor_.add_node(attack_node->get_node_base_interface());
      RCLCPP_INFO(logger_, "Adding attack node '%s'", attack_node->get_name());
    }
  }
}

std::future<void> Runner::execute_all_attacks_async()
{
  return std::async(std::launch::async,
           [this]() {start_and_stop_all_nodes();});
}

void Runner::initialize_attack_nodes(const std::vector<std::string> & node_names)
{
  for (const auto & node_name : node_names) {
    auto attack_node = build_attack_node_from_name(node_name);
    if (attack_node) {
      AttackNodeData node_data = {
        attack_node,
        std::make_shared<LifecycleServiceClient>(node_.get(), node_name)
      };
      attack_nodes_.emplace_back(std::move(node_data));
      executor_.add_node(attack_node->get_node_base_interface());
      RCLCPP_INFO(logger_, "Adding attack node '%s'", node_name.c_str());
    }
  }
}

std::vector<std::string> Runner::retrieve_attack_nodes_names()
{
  return node_->get_parameter(kAttackNodeNamesParameter).as_string_array();
}

void Runner::spin()
{
  RCLCPP_INFO(logger_, "Spinning started");
  std::shared_future<void> attack_result_future = execute_all_attacks_async();
  executor_.spin_until_future_complete(attack_result_future);
  RCLCPP_INFO(logger_, "Spinning finished");
}

void Runner::start_and_stop_all_nodes()
{
  for (const auto & node_data : attack_nodes_) {
    RCLCPP_INFO(logger_, "Configuring attack node '%s'", node_data.node->get_name());
    auto & client = node_data.lifecycle_client;
    if (!client->configure() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING)
    {
      return;
    }
  }
  for (const auto & node_data : attack_nodes_) {
    RCLCPP_INFO(logger_, "Enabling attack node '%s'", node_data.node->get_name());
    auto & client = node_data.lifecycle_client;
    if (!client->activate() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
    {
      return;
    }
  }
  for (const auto & node_data : attack_nodes_) {
    RCLCPP_INFO(logger_, "Shutting-down attack node '%s'", node_data.node->get_name());
    auto & client = node_data.lifecycle_client;
    if (!client->shutdown() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
    {
      return;
    }
  }
}

void Runner::warn_user_no_attack_nodes_passed()
{
  RCLCPP_WARN(logger_,
    "%s",
    "No attack specified. This node will not no anything.\n"
    "Please re-start this node and specify the list of attacks to execute:\n"
    "$ ros2 run ros_sec_test runner __params:=params.yaml\n"
    "\n"
    "params.yaml:\n"
    "attacker_node:\n"
    "  ros__parameters:\n"
    "    attack_nodes:\n"
    "      - 'noop'\n"
    "\n"
    "The available attacks are 'noop' and 'resources/disk'.\n"
  );
}

}  // namespace runner
}  // namespace ros_sec_test
