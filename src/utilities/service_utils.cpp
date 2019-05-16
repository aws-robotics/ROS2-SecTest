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
#include <sstream>
#include <string>

#include "utilities/service_utils.hpp"

static std::string build_service_name(
  const std::string & target_node_name,
  const std::string & topic_name)
{
  std::ostringstream ss;
  ss << "/";
  ss << target_node_name;
  ss << "/";
  ss << topic_name;
  return ss.str();
}

namespace ros_sec_test
{
namespace utilities
{

std::string build_change_state_service_name(const std::string & target_node_name)
{
  return build_service_name(target_node_name, "change_state");
}

std::string build_get_state_service_name(const std::string & target_node_name)
{
  return build_service_name(target_node_name, "get_state");
}

}  // namespace utilities
}  // namespace ros_sec_test
