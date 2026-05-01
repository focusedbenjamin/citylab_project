#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string.h>

// Alias for the GetDirection service from the robot_patrol package
using GetDirection = robot_patrol::srv::GetDirection;

using namespace std::chrono_literals;
using namespace std::placeholders;

class Patrol : public rclcpp::Node {
public:
  // Constructor: Initializes the node and sets up communication interfaces
  Patrol() : rclcpp::Node("patrol") {
    // Log message indicating the creation of a Patrol instance
    RCLCPP_INFO(this->get_logger(), "Creating instance object of Patrol class");

    // Create a subscription to the /scan topic to receive LaserScan messages
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));

    // Create a publisher to send velocity commands to the /cmd_vel topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a client to interact with the /direction_service
    client_ = this->create_client<GetDirection>("/direction_service");

    // Wait for the direction service to become available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service.");
    }

    // Log message indicating the client has started
    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
  }

private:
  // Callback method triggered when a LaserScan message is received
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (scan_init_done_) {
      // Set the front index value from laser scan ranges
      front_idx_ = (ranges_size_ / 2);
      // Set the front right index value from laser scans
      front_right_idx_ = (ranges_size_ / 2) - (segment_size_ / 2);
      // Set the front left index value from laser scans
      front_left_idx_ = (ranges_size_ / 2) + (segment_size_ / 2);

      // Create a request to send to the GetDirection service
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *msg;

      // Store the front laser reading based on the pre-defined index
      front_reading_ = msg->ranges[front_idx_];
      // Store the front right laser reading based on the pre-defined index
      front_right_reading_ = msg->ranges[front_right_idx_];
      // Store the front left laser reading based on the pre-defined index
      front_left_reading_ = msg->ranges[front_left_idx_];

      // Asynchronously send the request and handle the response in publishVel
      auto future_result = client_->async_send_request(
          request, std::bind(&Patrol::publishVel, this, _1));
    } else {
      // Get the length of laser scan ranges
      ranges_size_ = msg->ranges.size();
      // Get right and left laser scan index
      right_index_ = static_cast<int>(0.25 * ranges_size_);
      left_index_ = static_cast<int>(0.75 * ranges_size_);
      // Divide front scan ranges into 3 segments - left, front and right
      segment_size_ = static_cast<int>((left_index_ - right_index_) / 3.0);
      // Set laser scan initialization flag to true
      scan_init_done_ = true;
    }
  }

  // Method to process the service response and publish velocity commands
  void publishVel(const rclcpp::Client<GetDirection>::SharedFuture future) {
    // Get the response from the service
    auto response = future.get();

    // Create a Twist message to store velocity commands
    geometry_msgs::msg::Twist vel_message;

    // Check if the front reading exceeds front threshold
    if ((front_right_reading_ > side_threshold_) &&
        (front_reading_ > front_threshold_) &&
        (front_left_reading_ > side_threshold_)) {
      // Move forward if no object is detected within front threshold
      RCLCPP_INFO(this->get_logger(),
                  "Front Object Detected at %+0.3f m, Moving Forward.",
                  front_reading_);
      vel_message.linear.x = 0.1;
      vel_message.angular.z = 0.0;
    } else {
      // Use the service direction to determine movement
      RCLCPP_INFO(this->get_logger(), "Using Direction Service");
      // Log the received direction
      RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                  response->direction.c_str());
      if (response->direction == "forward") {
        vel_message.linear.x = 0.1;
        vel_message.angular.z = 0.0;
      } else if (response->direction == "left") {
        vel_message.linear.x = 0.1;
        // vel_message.angular.z = (3.14159 / 8.0); // sim
        vel_message.angular.z = (3.14159 / 8.0); // real
      } else if (response->direction == "right") {
        vel_message.linear.x = 0.1;
        // vel_message.angular.z = -(3.14159 / 8.0); // sim
        vel_message.angular.z = -(3.14159 / 2.0); // real
      } else {
        vel_message.linear.x = 0.0;
        vel_message.angular.z = 0.0;
      }
    }

    // Publish the velocity command to the /cmd_vel topic
    publisher_->publish(vel_message);
  }

  // Private attributes
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_; // Subscription to LaserScan messages
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_; // Publisher for velocity commands
  rclcpp::Client<GetDirection>::SharedPtr
      client_; // Client for the GetDirection service

  // Process variables
  bool scan_init_done_ = false; // Flag to indicate laser scan initialization
  int ranges_size_;             // Ranges in the laser scanner
  int right_index_;             // Index of the right laser reading
  int left_index_;              // Index of the left laser reading
  int segment_size_;            // Size of laser scan segments
  int front_idx_;               // Index of the front laser reading
  int front_right_idx_;         // Index of the front right laser reading
  int front_left_idx_;          // Index of the front left laser reading
  float front_reading_;         // Value of the front laser reading
  float front_right_reading_;   // Value of the front right laser reading
  float front_left_reading_;    // Value of the front left laser reading
  const float front_threshold_ = 0.35;
  const float side_threshold_ = 0.25;
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create an instance of the Patrol node
  auto node = std::make_shared<Patrol>();

  // Use a multi-threaded executor to spin the node and process callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
