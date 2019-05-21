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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "utilities/client_utils.hpp"

using rcl_interfaces::srv::ListParameters;
using rclcpp::executors::SingleThreadedExecutor;
using ros_sec_test::utilities::invoke_service_once_ready;
using ros_sec_test::utilities::wait_for_result;

TEST(invoke_service_once_ready, service_not_available) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  {
    auto node = rclcpp::Node::make_shared("test_node");
    auto client = node->create_client<ListParameters>("test_service");
    auto request = std::make_shared<ListParameters::Request>();
    auto response = invoke_service_once_ready(node.get(), client.get(), request, 100ms);
    RCLCPP_INFO(node->get_logger(), "The following reported error is expected.");
    EXPECT_EQ(response.get(), nullptr);
  }
  rclcpp::shutdown();
}

TEST(invoke_service_once_ready, invoke_service_successfully) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  {
    SingleThreadedExecutor executor;
    auto node = rclcpp::Node::make_shared("test_node");
    executor.add_node(node);

    std::promise<void> thread_promise;
    std::shared_future<void> future = thread_promise.get_future();
    std::atomic_bool service_has_been_invoked(false);

    auto callback =
      [&service_has_been_invoked](const ListParameters::Request::SharedPtr,
        ListParameters::Response::SharedPtr) {
        service_has_been_invoked.store(true);
      };
    auto srv = node->create_service<ListParameters>("test_service", callback);

    std::thread thread_spin([&executor, &future, &node]() {
        RCLCPP_INFO(node->get_logger(), "Spin thread started.");
        executor.spin_until_future_complete(future, 10ms);
        RCLCPP_INFO(node->get_logger(), "Spin thread ended.");
      });

    auto client = node->create_client<ListParameters>("test_service");
    auto request = std::make_shared<ListParameters::Request>();
    auto response = invoke_service_once_ready(node.get(), client.get(), request, 10s);
    EXPECT_NE(response.get(), nullptr);
    EXPECT_TRUE(service_has_been_invoked.load());

    RCLCPP_INFO(node->get_logger(), "Thread promise set.");
    thread_promise.set_value();
    thread_spin.join();
  }
  rclcpp::shutdown();
}

TEST(wait_for_result, check_timeout) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  {
    std::promise<void> promise;
    std::future<void> future = promise.get_future();
    EXPECT_EQ(wait_for_result(future, 1ms), std::future_status::timeout);
  }
  rclcpp::shutdown();
}

TEST(wait_for_result, check_ready) {
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  {
    std::promise<void> promise;
    std::future<void> future = promise.get_future();
    promise.set_value();
    EXPECT_EQ(wait_for_result(future, 1ms), std::future_status::ready);
  }
  rclcpp::shutdown();
}

// Unlike std::future::wait_for(), wait_for_result should never block if ROS is not
// initialized or already shut down.
TEST(wait_for_result, check_rclcpp_not_ok) {
  using namespace std::chrono_literals;
  EXPECT_FALSE(rclcpp::ok());
  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  EXPECT_EQ(wait_for_result(future, 10s), std::future_status::timeout);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
