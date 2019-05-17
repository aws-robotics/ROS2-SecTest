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
#ifndef RUNNER__RUNNER_HPP_
#define RUNNER__RUNNER_HPP_
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"

#include "utilities/lifecycle_service_client.hpp"

namespace ros_sec_test
{
namespace runner
{

class Runner
{
public:
  explicit Runner(const std::vector<std::string> & node_names);

  Runner(const Runner &) = delete;
  Runner & operator=(const Runner &) = delete;

  void spin();

private:
  struct AttackNodeData
  {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    std::shared_ptr<utilities::LifecycleServiceClient> lifecycle_client;
  };
  std::future<void> execute_all_attacks_async();
  void start_and_stop_all_nodes();

  /// Create a separate node to send lifecycle requests.
  rclcpp::Node::SharedPtr node_;
  std::vector<AttackNodeData> attack_nodes_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

}  // namespace runner
}  // namespace ros_sec_test

#endif  // RUNNER__RUNNER_HPP_
