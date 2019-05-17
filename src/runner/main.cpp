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
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "runner/runner.hpp"

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

  std::vector<std::string> initialized_nodes;
  for (const auto & node_name : node_names) {
    initialized_nodes.push_back(node_name);
  }
  Runner runner(initialized_nodes);
  rclcpp::executors::SingleThreadedExecutor & exec = runner.get_internal_executor();
  std::shared_future<void> script = std::async(std::launch::async,
      [&runner]() {runner.spin();});
  exec.spin_until_future_complete(script);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
