#include "ros_sec_test/runner/runner.hpp"

Runner::Runner(const std::string & node_name, std::shared_ptr<std::vector<std::string>> nodes)
: Node(node_name, "", rclcpp::NodeOptions().use_intra_process_comms(true)), nodes_(nodes)
{
  //initialize_node_vector();
}

void Runner::spin()
{
  std::cout << "Running\n";
  initialize_client_vector();
  std::cout << "Client vector initialized\n";
  //Configure attacks

  for (auto client: lc_clients_) {
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!client->get_state()) {
      return;
    }
  }
  std::cout << "Attacks configured\n";
  //Activate attacks
  for (auto client: lc_clients_) {
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!client->get_state()) {
      return;
    }
  }
  std::cout << "Attacks Activated\n";
  //Activate attacks
  for (auto client: lc_clients_) {
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN)) {
      return;
    }
    if (!client->get_state()) {
      return;
    }
  }
  std::cout << "Attacks Shutdown\n";
}

void Runner::initialize_client_vector()
{
  for (auto node_name: *nodes_) {
    lc_clients_.push_back(
      std::make_shared<LifecycleServiceClient>(this, node_name));
  }
  //lc_clients_.push_back(
  //  std::make_shared<LifecycleServiceClient>(this, std::string("teleop")));
  //lc_clients_.push_back(
  //  std::make_shared<LifecycleServiceClient>(this, std::string("disk_attacker")));

  for (auto client: lc_clients_) {
    client->init();
  }
}
