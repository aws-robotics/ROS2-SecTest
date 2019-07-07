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
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>


#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "runner/runner.hpp"
#include "test_utilities/utility_fixtures.hpp"
#include "test_utilities/periodic_attack_component_mock.hpp"

using LifecycleNodeShPtr = std::shared_ptr<rclcpp_lifecycle::LifecycleNode>;
using ros_sec_test::test::test_utilities::ROSTestingFixture;
using ros_sec_test::runner::Runner;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ::testing::Return;
using ::testing::_;

TEST_F(ROSTestingFixture, test_multiple_attack_lifecycle) {
  std::vector<LifecycleNodeShPtr> nodes;
  auto node1 = std::make_shared<MockPeriodicAttackComponent>("first_attack_node");
  auto node2 = std::make_shared<MockPeriodicAttackComponent>("second_attack_node");
  nodes.push_back(node1);
  nodes.push_back(node2);
  Runner runner(nodes);

  EXPECT_CALL(*node1, on_configure(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));
  EXPECT_CALL(*node1, on_activate(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));
  EXPECT_CALL(*node1, on_shutdown(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));


  EXPECT_CALL(*node2, on_configure(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));
  EXPECT_CALL(*node2, on_activate(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));
  EXPECT_CALL(*node2, on_shutdown(_))
  .Times(1)
  .WillOnce(Return(CallbackReturn::SUCCESS));

  runner.spin();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
