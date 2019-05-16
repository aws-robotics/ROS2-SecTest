// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include "utilities/lifecycle_service_client.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "utilities/client_utils.hpp"
#include "utilities/service_utils.hpp"

using namespace std::chrono_literals;

using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
using GetStateSrv = lifecycle_msgs::srv::GetState;

namespace ros_sec_test
{
namespace utilities
{

LifecycleServiceClient::LifecycleServiceClient(
  rclcpp::Node * parent_node,
  const std::string & target_node_name)
: target_node_name_(target_node_name),
  parent_node_(parent_node),
  client_change_state_(parent_node->create_client<ChangeStateSrv>(build_change_state_service_name(
      target_node_name))),
  client_get_state_(parent_node->create_client<GetStateSrv>(build_get_state_service_name(
      target_node_name)))
{}

unsigned
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<GetStateSrv::Request>();
  auto result = invoke_service_once_ready(parent_node_, client_get_state_.get(), request, time_out);
  return result->current_state.id;
}

bool
LifecycleServiceClient::change_state(
  rclcpp_lifecycle::Transition transition,
  std::chrono::seconds time_out)
{
  auto request = std::make_shared<ChangeStateSrv::Request>();
  request->transition.id = transition.id();
  auto result = invoke_service_once_ready(parent_node_,
      client_change_state_.get(), request, time_out);
  return !!result;
}

bool LifecycleServiceClient::activate()
{
  return change_state(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::
           TRANSITION_CONFIGURE));
}

bool LifecycleServiceClient::configure()
{
  return change_state(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::
           TRANSITION_CONFIGURE));
}

bool LifecycleServiceClient::shutdown()
{
  return change_state(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::
           TRANSITION_CONFIGURE));
}

}  // namespace utilities
}  // namespace ros_sec_test
