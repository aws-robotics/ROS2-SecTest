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

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "ros_sec_test/attacks/coms/teleop/component.hpp"
#include "ros_sec_test/attacks/noop/component.hpp"
#include "ros_sec_test/attacks/resources/disk/component.hpp"

#include "runner/runner.hpp"

void run_script(std::shared_ptr<Runner> runner);

void run_script(std::shared_ptr<Runner> runner)
{
  runner->spin();
}


int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  //Get set of attack to start
  std::unordered_set<std::string> node_names;
  for (int i = 1; i < argc; i++) {
    node_names.insert(std::string(argv[i]));
  }

  std::unordered_map<std::string,
    std::function<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>()>> attack_node_list = {
    {"noop", std::make_shared<ros_sec_test::attacks::noop::Component>},
    {"teleop", std::make_shared<ros_sec_test::attacks::coms::teleop::Component>},
    {"disk", std::make_shared<ros_sec_test::attacks::resources::disk::Component>}};

  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes;
  std::shared_ptr<std::vector<std::string>> initialized_nodes =
    std::make_shared<std::vector<std::string>>();
  for (const auto & node_name: node_names) {
    if (attack_node_list.count(node_name) == 1) {
      attack_nodes.push_back(attack_node_list.at(node_name)());
      initialized_nodes->push_back(node_name);
    }
  }
  std::cout << "Starting runner\n";
  rclcpp::executors::SingleThreadedExecutor exec;
  for (const auto node: attack_nodes) {
    exec.add_node(node->get_node_base_interface());
  }
  std::cout << "Nodes added to executor\n";
  std::shared_ptr<Runner> runner = std::make_shared<Runner>("attack_runner", initialized_nodes);
  exec.add_node(runner);
  std::shared_future<void> script = std::async(std::launch::async,
      std::bind(run_script, runner));

  exec.spin_until_future_complete(script);
  rclcpp::shutdown();
  return 0;
}
