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

template<typename Request, class Rep, class Period>
typename rclcpp::Client<Request>::SharedResponse invoke_service_once_ready(
  rclcpp::Node * node,
  rclcpp::Client<Request> * client,
  typename rclcpp::Client<Request>::SharedRequest request,
  const std::chrono::duration<Rep, Period> & time_out);

template<typename FutureT, typename Rep, typename Period>
std::future_status
wait_for_result(
  FutureT & future, const std::chrono::duration<Rep,
  Period> & timeout_duration);

#include "utilities/client_utils.hxx"
#endif  // UTILITIES__CLIENT_UTILS_HPP_
