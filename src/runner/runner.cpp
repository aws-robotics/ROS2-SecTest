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

using LifecycleServiceClient = ros_sec_test::utilities::LifecycleServiceClient;

namespace ros_sec_test
{
namespace runner
{

Runner::Runner(const std::string & node_name, std::shared_ptr<std::vector<std::string>> nodes)
: Node(node_name, "", rclcpp::NodeOptions().use_intra_process_comms(true)), nodes_(nodes)
{
}

void Runner::spin()
{
  initialize_client_vector();
  for (auto & client : lc_clients_) {
    if (!client->configure() || !client->get_state()) {
      return;
    }
  }
  for (auto & client : lc_clients_) {
    if (!client->activate() || !client->get_state()) {
      return;
    }
  }
  for (auto & client : lc_clients_) {
    if (!client->shutdown() || !client->get_state()) {
      return;
    }
  }
}

void Runner::initialize_client_vector()
{
  for (auto & node_name : *nodes_) {
    lc_clients_.push_back(
      std::make_shared<LifecycleServiceClient>(this, node_name));
  }
}

}  // namespace runner
}  // namespace ros_sec_test
