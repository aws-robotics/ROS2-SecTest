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
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros_sec_test/attacks/factory_utils.hpp"

using LifecycleServiceClient = ros_sec_test::utilities::LifecycleServiceClient;
using ros_sec_test::attacks::build_attack_node_from_name;

namespace ros_sec_test
{
namespace runner
{

Runner::Runner(const std::vector<std::string> & node_names)
: node_(std::make_shared<rclcpp::Node>("attacker_node", "",
    rclcpp::NodeOptions().use_intra_process_comms(true))),
  attack_nodes_(),
  lifecycle_clients_(),
  executor_()
{
  executor_.add_node(node_);
  for (const auto & node_name : node_names) {
    auto attack_node = build_attack_node_from_name(node_name);
    if (attack_node) {
      attack_nodes_.emplace_back(attack_node);
      executor_.add_node(attack_node->get_node_base_interface());
      lifecycle_clients_.push_back(
        std::make_shared<LifecycleServiceClient>(node_.get(), node_name));
    }
  }
}

std::future<void> Runner::execute_all_attacks_async()
{
  return std::async(std::launch::async,
           [this]() {start_and_stop_all_nodes();});
}

void Runner::spin()
{
  std::shared_future<void> attack_result_future = execute_all_attacks_async();
  executor_.spin_until_future_complete(attack_result_future);
}

void Runner::start_and_stop_all_nodes()
{
  for (auto & client : lifecycle_clients_) {
    if (!client->configure() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING)
    {
      return;
    }
  }
  for (auto & client : lifecycle_clients_) {
    if (!client->activate() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
    {
      return;
    }
  }
  for (auto & client : lifecycle_clients_) {
    if (!client->shutdown() ||
      client->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
    {
      return;
    }
  }
}

}  // namespace runner
}  // namespace ros_sec_test
