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
#ifndef UTILITIES__LIFECYCLE_SERVICE_CLIENT_HPP_
#define UTILITIES__LIFECYCLE_SERVICE_CLIENT_HPP_
#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/transition.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ros_sec_test
{
namespace utilities
{

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(rclcpp::Node * parent_node, const std::string & target_node_name);

  LifecycleServiceClient(const LifecycleServiceClient &) = delete;
  LifecycleServiceClient & operator=(const LifecycleServiceClient &) = delete;

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
  rclcpp_lifecycle::State get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  bool activate();
  bool configure();
  bool shutdown();

private:
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
  bool change_state(
    const rclcpp_lifecycle::Transition transition,
    const std::chrono::seconds time_out = std::chrono::seconds(5));

  const std::string target_node_name_;
  rclcpp::Node * parent_node_;
  const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
  const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
};

}  // namespace utilities
}  // namespace ros_sec_test

#endif  // UTILITIES__LIFECYCLE_SERVICE_CLIENT_HPP_
