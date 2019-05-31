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
#include <gtest/gtest.h>

#include <string>

#include "utilities/service_utils.hpp"

using ros_sec_test::utilities::build_change_state_service_name;
using ros_sec_test::utilities::build_get_state_service_name;

TEST(build_change_state_service_name, non_empty) {
  EXPECT_NE(build_change_state_service_name("node"), "");
}

TEST(build_get_state_service_name, non_empty) {
  EXPECT_NE(build_get_state_service_name("node"), "");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
