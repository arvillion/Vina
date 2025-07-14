#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "woosh_robot_msgs/srv/twist.hpp"
#include "geometry_msgs/msg/twist.hpp" // Include Twist message header

class TwistRedirector : public rclcpp::Node {
public:
  TwistRedirector()
      : Node("twist_redirector") {
    // Create a client for the Twist service
    twist_cli_ = this->create_client<woosh_robot_msgs::srv::Twist>(
        "woosh_robot/robot/Twist");

    // Wait for the service to be available
    while (!twist_cli_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Service unavailable!");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    // Create a subscriber to the /cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&TwistRedirector::cmdVelCallback, this, std::placeholders::_1));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Create a service request
    auto request = std::make_shared<woosh_robot_msgs::srv::Twist::Request>();
    request->arg.linear = msg->linear.x;  // Map linear.x to service request
    request->arg.angular = msg->angular.z; // Map angular.z to service request

    // Send the service request asynchronously
    auto request_future = twist_cli_->async_send_request(
      request, std::bind(&TwistRedirector::handleServiceResponse, this, std::placeholders::_1));
  }

  void handleServiceResponse(rclcpp::Client<woosh_robot_msgs::srv::Twist>::SharedFuture future) {
    auto response = future.get();
    if (response->ok) {
      RCLCPP_INFO(this->get_logger(), "Twist command succeeded: %s",
                  response->msg.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Twist command failed: %s",
                  response->msg.c_str());
    }
  }

  rclcpp::Client<woosh_robot_msgs::srv::Twist>::SharedPtr twist_cli_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistRedirector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}