#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

// Type aliases for ease of use
using Pose2D = geometry_msgs::msg::Pose2D;
using Twist = geometry_msgs::msg::Twist;
using GoToPose = robot_patrol::action::GoToPose;
using Odometry = nav_msgs::msg::Odometry;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using namespace std::placeholders;

class GoToPoseServer : public rclcpp::Node {
public:
  // Constructor: Initializes the node and sets up subscriptions, publishers,
  // and action servers
  GoToPoseServer() : rclcpp::Node::Node("go_to_pose_action") {
    // Subscription to the Odometry topic
    this->subscription_ = this->create_subscription<Odometry>(
        "/odom", 10, std::bind(&GoToPoseServer::odom_callback, this, _1));

    // Create an action server for the "go_to_pose" action
    this->action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "/go_to_pose",
        std::bind(&GoToPoseServer::handle_goal, this, _1, _2),
        std::bind(&GoToPoseServer::handle_cancel, this, _1),
        std::bind(&GoToPoseServer::handle_accepted, this, _1));

    // Publisher for velocity commands
    this->publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);

    // Log the launch of the action server
    RCLCPP_INFO(this->get_logger(), "Launched /go_to_pose action server.");
  }

private:
  // Attributes for storing the desired and current positions of the robot
  Pose2D desired_pos_;
  Pose2D current_pos_;

  // Shared pointers for the action server, subscription, and publisher
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  // Method to handle incoming goal requests
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal with x: %.2f y: %.2f theta: %.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;                                             // Unused variable
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal
  }

  // Method to handle cancellation requests
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;                            // Unused variable
    return rclcpp_action::CancelResponse::ACCEPT; // Accept the cancellation
  }

  // Method called when a goal is accepted
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    const auto goal = goal_handle->get_goal();
    desired_pos_.x = goal->goal_pos.x;
    desired_pos_.y = goal->goal_pos.y;
    desired_pos_.theta = goal->goal_pos.theta;

    // Start the execution in a separate thread
    std::thread{std::bind(&GoToPoseServer::execute, this, _1), goal_handle}
        .detach();
  }

  // Main execution loop for processing the goal
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(),
                "Executing goal: x: %.2f, y: %.2f, theta: %.2f", desired_pos_.x,
                desired_pos_.y, desired_pos_.theta);

    // Hardcoded rates
    rclcpp::Rate control_rate(2.0); // control loop at 30 Hz
    rclcpp::Duration feedback_period =
        rclcpp::Duration::from_seconds(0.2); // feedback at 5 Hz

    // Feedback
    auto feedback = std::make_shared<GoToPose::Feedback>();
    rclcpp::Time last_feedback_time = this->now();

    auto maybe_publish_feedback = [&]() {
      rclcpp::Time now = this->now();
      if (now - last_feedback_time >= feedback_period) {
        feedback->current_pos.x = current_pos_.x;
        feedback->current_pos.y = current_pos_.y;
        feedback->current_pos.theta = current_pos_.theta;
        goal_handle->publish_feedback(feedback);
        last_feedback_time = now;
      }
    };

    auto vel_msg = Twist();

    // --- Phase 1: rotate to face goal ---
    float angle_delta = INFINITY;
    float goal_direction = computeVectorAngle(desired_pos_.x - current_pos_.x,
                                              desired_pos_.y - current_pos_.y);

    while (std::abs(angle_delta) > M_PI / 90) {
      goal_direction = computeVectorAngle(desired_pos_.x - current_pos_.x,
                                          desired_pos_.y - current_pos_.y);
      angle_delta = goal_direction - current_pos_.theta;

      vel_msg.linear.x = 0.0f;
      // vel_msg.angular.z = (angle_delta / 8.0); // sim
      vel_msg.angular.z = (angle_delta / 8.0); // real
      publisher_->publish(vel_msg);

      maybe_publish_feedback();
      control_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Orientation to goal completed.");

    // --- Phase 2: translate towards goal ---
    float distance_delta = INFINITY;
    while (distance_delta > 0.01f) {
      float dx = desired_pos_.x - current_pos_.x;
      float dy = desired_pos_.y - current_pos_.y;
      goal_direction = computeVectorAngle(dx, dy);
      angle_delta = goal_direction - current_pos_.theta;
      distance_delta = std::sqrt(dx * dx + dy * dy);

      vel_msg.linear.x = std::min(0.2f, distance_delta);
      vel_msg.angular.z = (angle_delta / 2.0);
      publisher_->publish(vel_msg);

      maybe_publish_feedback();
      control_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Translation to goal completed.");

    // --- Phase 3: final yaw alignment ---
    angle_delta = INFINITY;
    vel_msg.linear.x = 0.0f;
    while (std::abs(angle_delta) > M_PI / 360) {
      angle_delta = desired_pos_.theta * M_PI / 180 - current_pos_.theta;
      vel_msg.angular.z = (angle_delta / 8.0); // sim
      vel_msg.angular.z = (angle_delta / 2.0); // real
      publisher_->publish(vel_msg);

      maybe_publish_feedback();
      control_rate.sleep();
    }
    vel_msg.angular.z = 0.0f;
    publisher_->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "Final goal pose completed.");

    // Send result
    auto result = std::make_shared<GoToPose::Result>();
    result->status = true;
    goal_handle->succeed(result);
  }

  // Callback method to handle incoming odometry data
  void odom_callback(const Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = computeRotationAngleZ(msg->pose.pose.orientation.z,
                                               msg->pose.pose.orientation.w);
  }

  // Utility method to compute the rotation angle (theta) from quaternion
  float computeRotationAngleZ(const float z, const float w) {
    float thetaRadians = 2 * atan2(z, w);
    return thetaRadians;
  }

  // Utility method to compute the angle of a vector (x, y) in radians
  float computeVectorAngle(const float x, const float y) {
    float thetaRadians = atan2(y, x);
    return thetaRadians;
  }
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a node for the GoToPoseServer
  auto node = std::make_shared<GoToPoseServer>();

  // Use a multi-threaded executor to run the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor to handle callbacks
  executor.spin();

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
