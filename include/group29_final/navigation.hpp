#pragma once

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <geometry_msgs/msg/pose_array.hpp>

namespace navigation {
class NavigationDemo : public rclcpp::Node {
 public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  NavigationDemo(std::string node_name) : Node(node_name) {
    // initialize the client
    client_ =
        rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    // initialize the publisher
    initial_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

    pose_array_subscriber_  = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "pose_array_topic", 10,
            std::bind(&NavigationDemo::poseArrayCallback, this, std::placeholders::_1));

    // set the initial pose for navigation
 

    

            // batttery1_subsrciber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            // "mage/camera5/image", rclcpp::SensorDataQoS(), std::bind(&NavigationDemo::logicalCameraCb5, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief Publisher to the topic /initialpose
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
  /**
   * @brief Action client for the action server follow_waypoints
   *
   */
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  //   rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Response from the server after sending the goal
   */
  void goal_response_callback(
      std::shared_future<GoalHandleNavigation::SharedPtr> future);
  /**
   * @brief Feedback received while the robot is driving towards the goal
   *
   * @param feedback
   */
  void feedback_callback(
      GoalHandleNavigation::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  /**
   * @brief Result after the action has completed
   *
   * @param result
   */
  void result_callback(const GoalHandleNavigation::WrappedResult& result);
  /**
   * @brief Method to build and send a goal using the client
   *
   */
  void send_goal();
  void set_initial_pose();
void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

geometry_msgs::msg::PoseArray pose_array_data ;




rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_subscriber_ ;
  
};  // class NavigationDemo
}  // namespace navigation