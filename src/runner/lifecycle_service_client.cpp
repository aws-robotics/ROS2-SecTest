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

#include "ros_sec_test/runner/lifecycle_service_client.hpp"

using namespace std::chrono_literals;

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

LifecycleServiceClient::LifecycleServiceClient(rclcpp::Node * parent_node, const std::string & target_node_name):
parent_node_(parent_node), target_node_name_(target_node_name)
{}

void
LifecycleServiceClient::init()
{

  // Every lifecycle node spawns automatically a couple
  // of services which allow an external interaction with
  // these nodes.
  // The two main important ones are GetState and ChangeState.
  std::string node_get_state_topic = "/" + target_node_name_ + "/get_state";
  std::string node_change_state_topic = "/" + target_node_name_ + "/change_state";

  client_get_state_ = this->parent_node_->create_client<lifecycle_msgs::srv::GetState>(
    node_get_state_topic);
  client_change_state_ = this->parent_node_->create_client<lifecycle_msgs::srv::ChangeState>(
    node_change_state_topic);
}

/// Requests the current state of the node
/**
 * In this function, we send a service request
 * asking for the current state of the node
 * lc_talker.
 * If it does return within the given time_out,
 * we return the current state of the node, if
 * not, we return an unknown state.
 * \param time_out Duration in seconds specifying
 * how long we wait for a response before returning
 * unknown state
 */
unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      parent_node_->get_logger(),
      "Service %s is not available.",
      client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // We send the service request for asking the current
  // state of the lc_talker node.
  auto future_result = client_get_state_->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      parent_node_->get_logger(), "Server time out while getting current state for node %s", target_node_name_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // We have an succesful answer. So let's print the current state.
  if (future_result.get()) {
    RCLCPP_INFO(parent_node_->get_logger(), "Node %s has current state %s.",
      target_node_name_.c_str(), future_result.get()->current_state.label.c_str());
    return future_result.get()->current_state.id;
  } else {
    RCLCPP_ERROR(
      parent_node_->get_logger(), "Failed to get current state for node %s", target_node_name_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

/// Invokes a transition
/**
 * We send a Service request and indicate
 * that we want to invoke transition with
 * the id "transition".
 * By default, these transitions are
 * - configure
 * - activate
 * - cleanup
 * - shutdown
 * \param transition id specifying which
 * transition to invoke
 * \param time_out Duration in seconds specifying
 * how long we wait for a response before returning
 * unknown state
 */
bool
LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      parent_node_->get_logger(),
      "Service %s is not available.",
      client_change_state_->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      parent_node_->get_logger(), "Server time out while changing current state for node %s", target_node_name_.c_str());
    return false;
  }

  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      parent_node_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(
      parent_node_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}
