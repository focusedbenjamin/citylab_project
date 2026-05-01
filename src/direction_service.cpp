#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

// Alias for the GetDirection service from the robot_patrol package
using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
  // Constructor: Initializes the node and creates the service
  DirectionService() : rclcpp::Node::Node("direction_service") {
    using namespace std::placeholders;

    // Create a service named "/direction_service"
    // The handle_service method will be called when a service request is
    // received
    service_ = this->create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handle_service, this, _1, _2));

    // Log a message indicating the service server has started
    RCLCPP_INFO(this->get_logger(), "Service Server Ready");
  }

private:
  // Attributes
  // Shared pointer to the service
  rclcpp::Service<GetDirection>::SharedPtr service_;

  // Method that handles incoming service requests
  void handle_service(const std::shared_ptr<GetDirection::Request> request,
                      std::shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service Requested");

    // Get length of scan data
    int ranges_size = request->laser_data.ranges.size();
    // Get right and left laser scan index
    int right_index = static_cast<int>(0.250 * ranges_size);
    int left_index = static_cast<int>(0.750 * ranges_size);
    // Divide front scan ranges into 3 segments - left, front and right
    int segment_size = ((left_index - right_index) / 3);

    // Variables to accumulate distances for each section
    // (Should be reset to zeros before every loop)
    float total_dist_sec_right = 0.0;
    float total_dist_sec_front = 0.0;
    float total_dist_sec_left = 0.0;

    // Process laser scan data and extract range values for different sections
    // of the robot's surroundings
    for (int i = 0; i < segment_size; i++) {
      // get the right, front and left ranges
      float right_range =
          request->laser_data.ranges[right_index + (0 * segment_size) + i];
      float front_range =
          request->laser_data.ranges[right_index + (1 * segment_size) + i];
      float left_range =
          request->laser_data.ranges[right_index + (2 * segment_size) + i];

      // Add right range to total if not inifinity
      if (!std::isinf(right_range)) {
        total_dist_sec_right += right_range;
      }

      // Add front range to total if not inifinity
      if (!std::isinf(front_range)) {
        total_dist_sec_front += front_range;
      }

      // Add left range to total if not inifinity
      if (!std::isinf(left_range)) {
        total_dist_sec_left += left_range;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "total_dist_sec_front: %+0.3f",
    //             total_dist_sec_front);
    // RCLCPP_INFO(this->get_logger(), "total_dist_sec_left: %+0.3f",
    //             total_dist_sec_left);
    // RCLCPP_INFO(this->get_logger(), "total_dist_sec_right: %+0.3f",
    //             total_dist_sec_right);

    // Determine which section has the greatest total distance and set the
    // direction accordingly
    if ((total_dist_sec_front > total_dist_sec_right) &&
        (total_dist_sec_front > total_dist_sec_left)) {
      response->direction = "forward";
    } else if ((total_dist_sec_right > total_dist_sec_front) &&
               (total_dist_sec_right > total_dist_sec_left)) {
      response->direction = "right";
    } else if ((total_dist_sec_left > total_dist_sec_front) &&
               (total_dist_sec_left > total_dist_sec_right)) {
      response->direction = "left";
    } else {
      // otherwise do nothing
    }
    RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                response->direction.c_str());
    RCLCPP_INFO(this->get_logger(), "Service Completed");
  }
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create an instance of the DirectionService node
  auto node = std::make_shared<DirectionService>();

  // Spin the node to process service requests
  rclcpp::spin(node);

  // Shutdown the ROS 2 system when done
  rclcpp::shutdown();
  return 0;
}
