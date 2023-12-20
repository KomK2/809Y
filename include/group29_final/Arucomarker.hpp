
#pragma once

// Include necessary libraries
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <unordered_map>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>

using namespace std::chrono_literals;

class ArucoMarker : public rclcpp::Node
{
public:

    ArucoMarker(std::string node_name) : Node(node_name)
    { 
        subscriber_aruco_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::arucoCallback, this, std::placeholders::_1));

        logical_camera_1 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera1/image", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::logicalCameraCb1, this, std::placeholders::_1));

        logical_camera_2 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera2/image", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::logicalCameraCb2, this, std::placeholders::_1));

        logical_camera_3 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera3/image", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::logicalCameraCb3, this, std::placeholders::_1));

        logical_camera_4 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera4/image", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::logicalCameraCb4, this, std::placeholders::_1));

        logical_camera_5 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera5/image", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::logicalCameraCb5, this, std::placeholders::_1));

        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "clock", rclcpp::SensorDataQoS(), std::bind(&ArucoMarker::clock_callback, this, std::placeholders::_1));


        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        broadcaster_timer_ = this->create_wall_timer(100ms, std::bind(&ArucoMarker::broadcaster_timer, this));

        publishs_data_timer_ = this->create_wall_timer(100ms, std::bind(&ArucoMarker::sort_and_publish, this));

        batttery1_publisher_  = this->create_publisher<geometry_msgs::msg::Pose>("/battery_position_1", 10);
        batttery2_publisher_  = this->create_publisher<geometry_msgs::msg::Pose>("/battery_position_2", 10);
        batttery3_publisher_  = this->create_publisher<geometry_msgs::msg::Pose>("/battery_position_3", 10);
        batttery4_publisher_  = this->create_publisher<geometry_msgs::msg::Pose>("/battery_position_4", 10);
        batttery5_publisher_  = this->create_publisher<geometry_msgs::msg::Pose>("/battery_position_5", 10);

        pose_array_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pose_array_topic", 10);

    }

private:
    ros2_aruco_interfaces::msg::ArucoMarkers aruco_data ;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscriber_aruco_;
    void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    std::unordered_map<std::string, mage_msgs::msg::AdvancedLogicalCameraImage> logical_cam_dict;

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_1 ;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_2 ;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_3 ;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_4 ;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_5 ;

    void logicalCameraCb1(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void logicalCameraCb2(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void logicalCameraCb3(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void logicalCameraCb4(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void logicalCameraCb5(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    rosgraph_msgs::msg::Clock clock_time;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;

    void broadcast_cb_(mage_msgs::msg::AdvancedLogicalCameraImage data, const std::string &source_frame, const std::string &target_frame, const std::string &part_frame);
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    void listen_to_timer_callback(mage_msgs::msg::AdvancedLogicalCameraImage data,const std::string &source_frame,  const std::string &part_frame);
    void listen_transform(mage_msgs::msg::AdvancedLogicalCameraImage data,const std::string &source_frame, const std::string &target_frame );

    rclcpp::TimerBase::SharedPtr broadcaster_timer_;
    rclcpp::TimerBase::SharedPtr publishs_data_timer_;
    void broadcaster_timer();
    rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr batttery1_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr batttery2_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr batttery3_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr batttery4_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr batttery5_publisher_;

    void publish_data(const std::string &target_frame, geometry_msgs::msg::Pose battery_pose_actual);

    std::vector<int> list_of_colors;
    std::unordered_map<int ,geometry_msgs::msg::Pose> battery_poses; 

    void sort_and_publish();  

    int color_to_number_mapp(std::string color); 
    std::vector<int> sorted_list_of_colors;
    std::string toUpperCase(const std::string& input) ;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;


protected:
    int  numberOfWayPoints(int aruco_index);

    
};
