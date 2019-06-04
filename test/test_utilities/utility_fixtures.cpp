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
#include "test_utilities/utility_fixtures.hpp"

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

void NodeConfigurationFixture::SetUp()
{
  node_ = rclcpp::Node::make_shared("test_node");
  executor_.add_node(node_);
  future_ = thread_promise_.get_future();
}

void NodeConfigurationFixture::add_node_to_executor(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  executor_.add_node(node);
}

void NodeConfigurationFixture::spin_executor_until(std::shared_future<void> & future)
{
  using namespace std::chrono_literals;
  std::thread thread_spin([this, &future]() {
      RCLCPP_INFO(this->node_->get_logger(), "Spin thread started.");
      this->executor_.spin_until_future_complete(future, 100ms);
      RCLCPP_INFO(this->node_->get_logger(), "Spin thread ended.");
    });
  thread_spin.join();
}
void NodeConfigurationFixture::start_executor()
{
  spin_executor_until(future_);
}
void NodeConfigurationFixture::stop_executor()
{
  thread_promise_.set_value();
}

void NodeConfigurationFixture::TearDown()
{
  stop_executor();
}
}  // namespace test_utilities
}  // namespace test
}  // namespace ros_sec_test
