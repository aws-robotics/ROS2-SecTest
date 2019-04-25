#include "ros_sec_test/runner/runner.hpp"

Runner::Runner(const std::string & node_name): Node(node_name, "", true){
  //initialize_node_vector();
}

void Runner::spin() {
  std::cout<<"Running\n";
/*
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(this);
  for (const auto node: attack_nodes_) {

    exec.add_node(node->get_node_base_interface());
  }
  std::cout<<"Nodes added to executor\n";
*/
  initialize_client_vector();
  std::cout<<"Client vector initialized\n";
  //Configure attacks

  for (auto client: lc_clients_){
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!client->get_state()) {
      return;
    }
  }
  std::cout<<"Attacks configured\n";
  //Activate attacks
  for (auto client: lc_clients_){
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!client->get_state()) {
      return;
    }
  }
}

void Runner::initialize_client_vector() {
  lc_clients_.push_back(
    std::make_shared<LifecycleServiceClient>(this, std::string("noop")));

  for (auto client: lc_clients_) {
    client->init();
  }
}

/*
void Runner::initialize_node_vector() {
  attack_nodes_.push_back(
      std::make_shared<ros_sec_test::attacks::noop::Component>());
/*
  attack_nodes_.push_back(
      std::make_shared<ros_sec_test::attacks::resources::cpu::Component>(
          options_));
  attack_nodes_.push_back(
      std::make_shared<ros_sec_test::attacks::resources::disk::Component>(
          options_));
  attack_nodes_.push_back(
      std::make_shared<ros_sec_test::attacks::resources::memory::Component>(
          options_));
*/
//}

