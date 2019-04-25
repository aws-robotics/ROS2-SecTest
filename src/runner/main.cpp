#include <memory>
#include <thread>
#include "ros_sec_test/runner/runner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"



void run_script(std::shared_ptr<Runner> runner) {
    runner->spin();
}



int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> attack_nodes;
  attack_nodes.push_back(
      std::make_shared<ros_sec_test::attacks::noop::Component>());

  std::cout<<"Starting runner\n";
  rclcpp::executors::SingleThreadedExecutor exec;
  for (const auto node: attack_nodes) {

    exec.add_node(node->get_node_base_interface());
  }
  std::cout<<"Nodes added to executor\n";
  std::shared_ptr<Runner> runner = std::make_shared<Runner>("attack_runner");
  exec.add_node(runner);
  std::shared_future<void> script = std::async(std::launch::async,
      std::bind(run_script, runner));

  exec.spin_until_future_complete(script);
  rclcpp::shutdown();
  return 0;
}