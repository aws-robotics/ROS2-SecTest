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
#ifndef UTILITIES__CLIENT_UTILS_HPP_
#define UTILITIES__CLIENT_UTILS_HPP_

#include <chrono>
#include <future>

#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"

namespace ros_sec_test
{
namespace utilities
{

/// Wait for a service to exist and invoke it.
/**
 * \param[in] node Node invoking the service.
 * \param[in] client Client invoking the service.
 * \param[in] request Request object passed during service invocation.
 * \param[in] time_out maximum duration this method will block.
 *
 * \tparam Request Service request type
 * \tparam Rep Timeout argument Rep type (see STL documentation)
 * \tparam Period Timeout argument Period type (see STL documentation)
 */
template<typename Request, class Rep, class Period>
typename rclcpp::Client<Request>::SharedResponse invoke_service_once_ready(
  rclcpp::Node * node,
  rclcpp::Client<Request> * client,
  typename rclcpp::Client<Request>::SharedRequest request,
  const std::chrono::duration<Rep, Period> & time_out);

/// Identical to std::future::wait_for except but waiting will be interrupting if ROS 2 shutdowns.
/*
 * Using std::future::wait_for has one inconvenient: if ROS 2 is shutting down, the future
 * result may never be delivered and wait_for will hang until the timeout is reached.
 * There is no way in the STL C++11 to wait on ROS 2 shutdown and a future result simultaneously.
 *
 * This method simulates this behavior by waking up the thread regularly to check on ROS 2 status.
 *
 * \param[in] future Future to wait on.
 * \param[in] time_out maximum duration this method will block.
 * \return Future status (same behavior than std::future::wait_for).
 */
template<typename FutureT, typename Rep, typename Period>
std::future_status
wait_for_result(
  FutureT & future, const std::chrono::duration<Rep,
  Period> & timeout_duration);

}  // namespace utilities
}  // namespace ros_sec_test

#include "utilities/client_utils.hxx"
#endif  // UTILITIES__CLIENT_UTILS_HPP_
