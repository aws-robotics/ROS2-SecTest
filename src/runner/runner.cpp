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

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"

using LifecycleServiceClient = ros_sec_test::utilities::LifecycleServiceClient;

namespace ros_sec_test
{
namespace runner
{

Runner::Runner(const std::vector<std::string> & node_names)
: node_(std::make_shared<rclcpp::Node>("attacker_node", "",
    rclcpp::NodeOptions().use_intra_process_comms(true))),
  nodes_(node_names),
  executor_()
{
}

void Runner::spin()
{
  initialize_client_vector();
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

void Runner::initialize_client_vector()
{
  for (const auto & node_name : nodes_) {
    lifecycle_clients_.push_back(
      std::make_shared<LifecycleServiceClient>(node_.get(), node_name));
  }
}

}  // namespace runner
}  // namespace ros_sec_test
