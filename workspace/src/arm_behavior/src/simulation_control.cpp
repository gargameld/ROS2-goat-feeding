#include "arm_behavior/simulation_control.hpp"

#include <chrono>
#include <future>
#include <stdexcept>
#include <string>

#include "std_srvs/srv/trigger.hpp"

namespace
{

constexpr char kPauseSimulationService[] = "/physics_sync_node/pause_simulation";
constexpr char kResumeSimulationService[] = "/physics_sync_node/resume_simulation";

void call_simulation_service(
  const rclcpp::Node::SharedPtr & node,
  const std::string & service_name,
  std::chrono::seconds timeout)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(timeout)) {
    throw std::runtime_error(
            "Simulation service '" + service_name + "' was not available after " +
            std::to_string(timeout.count()) + " seconds");
  }

  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    throw std::runtime_error(
            "Simulation service '" + service_name + "' timed out after " +
            std::to_string(timeout.count()) + " seconds");
  }

  const auto response = future.get();
  if (!response->success) {
    throw std::runtime_error(
            response->message.empty() ?
            "Simulation service '" + service_name + "' rejected the request" :
            response->message);
  }
}

}  // namespace

namespace arm
{

void pause_simulation(const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  call_simulation_service(node, kPauseSimulationService, timeout);
}

void resume_simulation(const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  call_simulation_service(node, kResumeSimulationService, timeout);
}

}  // namespace arm
