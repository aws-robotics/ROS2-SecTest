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
#include <cstdio>
#include <cstdlib>

#include "rclcpp/utilities.hpp"
#include "runner/runner.hpp"

using ros_sec_test::runner::Runner;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  Runner runner;
  runner.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
