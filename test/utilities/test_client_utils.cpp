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
#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "utilities/client_utils.hpp"

using rcl_interfaces::srv::ListParameters;
using rclcpp::executors::SingleThreadedExecutor;
using ros_sec_test::utilities::invoke_service_once_ready;

TEST(invoke_service_once_ready, check_wait_logic) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  {
    SingleThreadedExecutor executor;
    auto node = rclcpp::Node::make_shared("test_node");
    executor.add_node(node);
    auto callback =
      [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
      };
    auto srv = node->create_service<ListParameters>("test_service", callback);

    std::promise<void> thread_promise;
    std::shared_future<void> future = thread_promise.get_future();

    std::thread thread_spin([&executor, &future, &node]() {
        RCLCPP_INFO(node->get_logger(), "Spin thread started.");
        executor.spin_until_future_complete(future, 10ms);
        RCLCPP_INFO(node->get_logger(), "Spin thread ended.");
      });
    std::thread thread_client([&node]() {
        RCLCPP_INFO(node->get_logger(), "Client thread started.");
        auto client = node->create_client<ListParameters>("test_service");
        auto request = std::make_shared<ListParameters::Request>();
        invoke_service_once_ready(node.get(), client.get(), request, 10s);
        RCLCPP_INFO(node->get_logger(), "Client thread ended.");
      });

    thread_client.join();
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
