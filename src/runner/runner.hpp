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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"

#include "utilities/lifecycle_service_client.hpp"

namespace ros_sec_test
{
namespace runner
{

class Runner : public rclcpp::Node
{
public:
  Runner(const std::string &, const std::vector<std::string> & node_names);

  Runner(const Runner &) = delete;
  Runner & operator=(const Runner &) = delete;

  void spin();
  virtual void initialize_client_vector();

private:
  std::vector<std::string> nodes_;
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes_;
  std::vector<std::shared_ptr<utilities::LifecycleServiceClient>> lifecycle_clients_;
};

}  // namespace runner
}  // namespace ros_sec_test

#endif  // RUNNER__RUNNER_HPP_
