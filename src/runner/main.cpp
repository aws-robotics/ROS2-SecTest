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
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "ros_sec_test/attacks/factory_utils.hpp"

#include "runner/runner.hpp"

using ros_sec_test::attacks::build_attack_node_from_name;
using ros_sec_test::runner::Runner;
using ros_sec_test::utilities::LifecycleServiceClient;

void run_script(Runner & runner);

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  // Get set of attack to start
  std::unordered_set<std::string> node_names;
  for (int i = 1; i < argc; i++) {
    node_names.insert(std::string(argv[i]));
  }

  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes;
  std::vector<std::string> initialized_nodes;
  for (const auto & node_name : node_names) {
    auto attack_node = build_attack_node_from_name(node_name);
    if (attack_node) {
      attack_nodes.emplace_back(attack_node);
      initialized_nodes.push_back(node_name);
    }
  }
  std::cout << "Starting runner\n";
  rclcpp::executors::SingleThreadedExecutor exec;
  for (const auto node : attack_nodes) {
    exec.add_node(node->get_node_base_interface());
  }
  std::cout << "Nodes added to executor\n";
  Runner runner(initialized_nodes);
  exec.add_node(runner.get_internal_node());
  std::shared_future<void> script = std::async(std::launch::async,
      [&runner]() {runner.spin();});

  exec.spin_until_future_complete(script);
  rclcpp::shutdown();
  return 0;
}
