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
#ifndef RUNNER__RUNNER_HPP_
#define RUNNER__RUNNER_HPP_
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"

#include "utilities/lifecycle_service_client.hpp"

namespace ros_sec_test
{
namespace runner
{

/// Main class containing the whole application logic, instantiated in main().
/**
 * Runner holds a ROS 2 node and one additional lifecycle_node per enabled attack node.
 *
 * The additional ROS 2 node (node_) is used to keep isolated from attack nodes the
 * calls to the services managing the lifecycle nodes.
 *
 * Attack nodes are set a instantiation time and can *never* be changed.
 *
 * The only public method exposed by this class is spin() which starts, execute the attacks
 * and stop all the nodes when needed.
 */
class Runner
{
public:
  /// Instantiate all nodes from parameter without activating them.
  /**
   * CAVEAT: rclcpp::init() *must* be called before instantiating this object.
   */
  Runner();

  /// Use a vector of already instantiated nodes.
  /**
   * CAVEAT: rclcpp::init() *must* be called before instantiating this object.
   */
  explicit Runner(const std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> & attack_nodes);

  Runner(const Runner &) = delete;
  Runner & operator=(const Runner &) = delete;

  /// Starts, executes all attacks and shutdown all nodes.
  void spin();

private:
  /// Holds a shared pointer to a node and the associated lifecycle client.
  struct AttackNodeData
  {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

    /// Wraps lifecycle service calls in a conveninent interface.
    std::shared_ptr<utilities::LifecycleServiceClient> lifecycle_client;
  };

  /// Execute all attacks synchronously.
  /**
   * Running attacks in a different thread allow us to spin from the main thread.
   */
  std::future<void> execute_all_attacks_async();

  /// Initialize the attack_nodes_ attributes. Invoked from the class constructor.
  /**
   * \param[in] node_names All attack nodes to be instantiated.
   */
  void initialize_attack_nodes(const std::vector<std::string> & node_names);

  /// Read the list of attacks to be run from the node parameter.
  /**
   * \return Names of all enabled attacks.
   */
  std::vector<std::string> retrieve_attack_nodes_names();

  /// Relies on lifecycle client to start and stop all attacks.
  void start_and_stop_all_nodes();

  /// Log a message explaining that no attack have been specified and the runner will do nothing.
  void warn_user_no_attack_nodes_passed();

  /// Create a separate node to send lifecycle requests.
  rclcpp::Node::SharedPtr node_;

  /// Contain all attack nodes information.
  std::vector<AttackNodeData> attack_nodes_;

  /// Executor shared by all nodes.
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Logger logger_;
};

}  // namespace runner
}  // namespace ros_sec_test

#endif  // RUNNER__RUNNER_HPP_
