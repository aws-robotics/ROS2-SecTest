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
#ifndef ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HPP_
#define ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HPP_

#include <chrono>
#include <future>

template<typename FutureT, typename Rep, typename Period>
std::future_status
wait_for_result(
  FutureT & future, const std::chrono::duration<Rep,
  Period> & timeout_duration);

#include "utilities/client_utils.hxx"
#endif  //! ROS_SEC_TEST__UTILITIES_CLIENT_UTILS_HPP_
