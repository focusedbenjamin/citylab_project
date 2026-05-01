#include "rclcpp/client.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

// Aliases for the GetDirection service and LaserScan message
using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

using namespace std::chrono_literals;
using namespace std::placeholders;

class DirectionClient : public rclcpp::Node {
public:
  // Constructor: Initializes the node and sets up the client and subscription
  DirectionClient() : rclcpp::Node::Node("direction_service") {
    // Create a client that will call the /direction_service
    client_ = this->create_client<GetDirection>("/direction_service");

    // Create a subscription to the /scan topic to receive LaserScan messages
    subscription_ = this->create_subscription<LaserScan>(
        "/scan", 1, std::bind(&DirectionClient::scan_callback, this, _1));

    // Wait for the service to be available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service.");
    }

    // Log a message indicating the client has started
    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
  }

private:
  // Attributes
  rclcpp::Client<GetDirection>::SharedPtr
      client_; // Client for the GetDirection service
  rclcpp::Subscription<LaserScan>::SharedPtr
      subscription_; // Subscription to the LaserScan topic

  // Method called when a LaserScan message is received
  void scan_callback(const LaserScan::SharedPtr msg) {
    // Create a request for the GetDirection service
    auto request = std::make_shared<GetDirection::Request>();

    // Fill the request with the laser scan data
    request->laser_data = *msg;

    // Asynchronously send the request to the service and bind the response
    // handler
    auto future_result = client_->async_send_request(
        request, std::bind(&DirectionClient::handle_response, this, _1));
    RCLCPP_INFO(this->get_logger(), "Service Requested");
  }

  // Method to handle the service response
  void
  handle_response(const rclcpp::Client<GetDirection>::SharedFuture future) {
    // Get the service response
    auto response = future.get();

    // Log the direction returned by the service
    RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                response->direction.c_str());
    RCLCPP_INFO(this->get_logger(), "Service Completed");
  }
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create an instance of the DirectionClient node
  auto node = std::make_shared<DirectionClient>();

  // Spin the node to keep processing callbacks
  rclcpp::spin(node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
