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
#ifndef ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HXX_
#define ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HXX_

#include <chrono>
#include <future>

#include "rclcpp/utilities.hpp"

template<typename FutureT, typename Rep, typename Period>
std::future_status
wait_for_result(
  FutureT & future,
  const std::chrono::duration<Rep, Period> & timeout_duration)
{
  const auto end = std::chrono::steady_clock::now() + timeout_duration;
  const std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    const auto now = std::chrono::steady_clock::now();
    const auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      return status;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

#endif  //! ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HXX_
