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
#include "utilities/lifecycle_service_client.hpp"
#include "ros_sec_test/attacks/factory_utils.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

using LifecycleServiceClient = ros_sec_test::utilities::LifecycleServiceClient;
using ros_sec_test::attacks::build_attack_node_from_name;
using CPUNode = ros_sec_test::attacks::resources::cpu::Component;

TEST(attacks_resources_cpu_component, state_transition) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  const std::string node_name = "resources_cpu";
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = rclcpp::Node::make_shared("test_node");
    auto attack_node = std::make_shared<CPUNode>(1);
    LifecycleServiceClient lifecycle_client(node.get(), node_name);
    executor.add_node(node);
    executor.add_node(attack_node->get_node_base_interface());
    std::promise<void> thread_promise;
    std::shared_future<void> future = thread_promise.get_future();
    RCLCPP_INFO(node->get_logger(), "Starting thread");

    std::thread thread_spin([&executor, &future, &node]() {
        RCLCPP_INFO(node->get_logger(), "Spin thread started.");
        executor.spin_until_future_complete(future, 1000ms);
        RCLCPP_INFO(node->get_logger(), "Spin thread ended.");
      });

    EXPECT_TRUE(lifecycle_client.configure());

    EXPECT_TRUE(lifecycle_client.activate());

    EXPECT_TRUE(lifecycle_client.shutdown());

    RCLCPP_INFO(node->get_logger(), "Thread promise set.");
    thread_promise.set_value();
    thread_spin.join();
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
