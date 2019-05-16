#include "runner/runner.hpp"

Runner::Runner(const std::string & node_name, std::shared_ptr<std::vector<std::string>> nodes)
: Node(node_name, "", rclcpp::NodeOptions().use_intra_process_comms(true)), nodes_(nodes)
{
}

void Runner::spin()
{
  initialize_client_vector();
  for (auto & client: lc_clients_) {
    if (!client->configure() || !client->get_state()) {
      return;
    }
  }
  for (auto & client: lc_clients_) {
    if (!client->activate() || !client->get_state()) {
      return;
    }
  }
  for (auto & client: lc_clients_) {
    if (!client->shutdown() || !client->get_state()) {
      return;
    }
  }
}

void Runner::initialize_client_vector()
{
  for (auto & node_name: *nodes_) {
    lc_clients_.push_back(
      std::make_shared<LifecycleServiceClient>(this, node_name));
  }
}
