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
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(rclcpp::Node * parent_node, const std::string & target_node_name);

  LifecycleServiceClient(const LifecycleServiceClient &) = delete;
  LifecycleServiceClient & operator=(const LifecycleServiceClient &) = delete;

  void init();
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));
  bool change_state(
    std::uint8_t transition,
    std::chrono::seconds time_out = std::chrono::seconds(5));

private:
  rclcpp::Node * parent_node_;
  std::string target_node_name_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};
