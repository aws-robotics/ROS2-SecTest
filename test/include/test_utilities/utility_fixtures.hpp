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

#ifndef TEST_UTILITIES__UTILITY_FIXTURES_HPP_
#define TEST_UTILITIES__UTILITY_FIXTURES_HPP_
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

namespace ros_sec_test
{
namespace test
{
namespace test_utilities
{

class ROSTestingFixture : public ::testing::Test
{
public:
  ROSTestingFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~ROSTestingFixture()
  {
    rclcpp::shutdown();
  }
};


class NodeConfigurationFixture : public ROSTestingFixture
{
protected:
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  std::promise<void> thread_promise_;
  std::shared_future<void> future_;

public:
  void SetUp() override;
  void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);
  void spin_executor_until(std::shared_future<void> & future);
  void start_executor();
  void stop_executor();
  void TearDown() override;
};
}  // namespace test_utilities
}  // namespace test
}  // namespace ros_sec_test
#endif  // TEST_UTILITIES__UTILITY_FIXTURES_HPP_
