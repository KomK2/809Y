#include <rclcpp/rclcpp.hpp>
#include <Arucomarker.hpp>
#include<thread>
#include <chrono>
#include <string>
#include <cctype>


using namespace std::chrono_literals;


void ArucoMarker::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{   
    aruco_data = *msg;
    if(aruco_data.marker_ids.size()){
        // shutting down the aruco node 
        subscriber_aruco_.reset();

        auto aruco_index = aruco_data.marker_ids.at(0) ;
        
        for (int j = 1; j <= 5; ++j) {  
                std::string base_param_path = "aruco_" + std::to_string(aruco_index) + ".wp" + std::to_string(j);

                this->declare_parameter(base_param_path + ".type", "battery");
                this->declare_parameter(base_param_path + ".color", "green");

                std::string type, color;
                this->get_parameter(base_param_path + ".type", type);
                this->get_parameter(base_param_path + ".color", color);

                auto upper_case_colors = toUpperCase(color);
                int col =    color_to_number_mapp(upper_case_colors);
                RCLCPP_INFO(this->get_logger(),"The color %s is : %i" ,upper_case_colors.c_str() ,col);
                list_of_colors.push_back(col);
            }
    }
}


std::string ArucoMarker::toUpperCase(const std::string& input) {
    std::string result = input;
    for (char& c : result) {
        c = std::toupper(c);
    }
    return result;
}

int ArucoMarker:: color_to_number_mapp(std::string color){
    for (int i = 0; i <=5; i++)
    {
        auto green= int(mage_msgs::msg::Part::GREEN) ;
        auto blue= int(mage_msgs::msg::Part::BLUE) ;
        auto red= int(mage_msgs::msg::Part::RED) ;
        auto orange= int(mage_msgs::msg::Part::ORANGE) ;
        auto purple= int(mage_msgs::msg::Part::PURPLE) ;
        if (color == "GREEN"){
          return green ;
        }
        else if(color == "RED"){
            return red ; 
        }else if (color == "ORANGE")
        {
         return orange; 
        }else if (color == "PURPLE")
        {
            return purple ;
        
        }else if (color == "BLUE")
        {
           return blue;
        }
    }

    
}

int ArucoMarker::numberOfWayPoints(int aruco_index)
{
   int count = 0;
        while (true)
        {
            std::string wp_param_path = "aruco_" + std::to_string(aruco_index) + ".wp" + std::to_string(count + 1);
            if (!this->has_parameter(wp_param_path + ".type")) 
            {
                break;
            }
            count++;
        }
        return count;
}

void ArucoMarker::logicalCameraCb1(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
   logical_cam_dict["lc1"] = *msg;
   if (logical_cam_dict.count("lc1"))
   {
    logical_camera_1.reset();
   }
   
   
}

void ArucoMarker::logicalCameraCb2(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
     
    logical_cam_dict["lc2"] = *msg;
    if (logical_cam_dict.count("lc2"))
   {
    logical_camera_2.reset();
   }
}

void ArucoMarker::logicalCameraCb3(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    logical_cam_dict["lc3"] = *msg;
    if (logical_cam_dict.count("lc3"))
   {
    logical_camera_3.reset();
   }
}

void ArucoMarker::logicalCameraCb4(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    logical_cam_dict["lc4"] = *msg;
    if (logical_cam_dict.count("lc4"))
   {
    logical_camera_4.reset();
   }
}

void ArucoMarker::logicalCameraCb5(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    logical_cam_dict["lc5"] = *msg;
    if (logical_cam_dict.count("lc5"))
   {
    logical_camera_5.reset();
   }
}

void ArucoMarker::broadcast_cb_(mage_msgs::msg::AdvancedLogicalCameraImage data,
 const std::string &source_frame, const std::string &target_frame,
 const std::string &part_frame
 ){
    geometry_msgs::msg::TransformStamped geometry_message;

    double broadcast_time = clock_time.clock.sec;
    rclcpp::Time battery_time(broadcast_time, clock_time.clock.nanosec);

    geometry_msgs::msg::TransformStamped t_stamped_broadcast;
    geometry_message.header.stamp = battery_time;

    geometry_message.header.frame_id = source_frame;
    geometry_message.child_frame_id = target_frame ;

    geometry_message.transform.translation.x = data.sensor_pose.position.x ;
    geometry_message.transform.translation.y = data.sensor_pose.position.y;
    geometry_message.transform.translation.z = data.sensor_pose.position.z;

    geometry_message.transform.rotation.x = data.sensor_pose.orientation.x ;
    geometry_message.transform.rotation.y = data.sensor_pose.orientation.y;
    geometry_message.transform.rotation.z = data.sensor_pose.orientation.z;
    geometry_message.transform.rotation.w = data.sensor_pose.orientation.w;

    tf_broadcaster_->sendTransform(geometry_message);

    geometry_message.header.frame_id = target_frame;
    geometry_message.child_frame_id = part_frame;

    geometry_message.transform.translation.x = data.part_poses[0].pose.position.x;
    geometry_message.transform.translation.y = data.part_poses[0].pose.position.y;
    geometry_message.transform.translation.z = data.part_poses[0].pose.position.z;

    geometry_message.transform.rotation.x = data.part_poses[0].pose.orientation.x;
    geometry_message.transform.rotation.y = data.part_poses[0].pose.orientation.y;
    geometry_message.transform.rotation.z = data.part_poses[0].pose.orientation.z;
    geometry_message.transform.rotation.w = data.part_poses[0].pose.orientation.w;

    tf_broadcaster_->sendTransform(geometry_message);
    listen_to_timer_callback(data,source_frame,part_frame );
}



void ArucoMarker::listen_to_timer_callback(
    mage_msgs::msg::AdvancedLogicalCameraImage data,
 const std::string &source_frame,
 const std::string &part_frame)
{

  listen_transform(data,source_frame, part_frame);
}

void ArucoMarker::listen_transform(mage_msgs::msg::AdvancedLogicalCameraImage data,const std::string &source_frame, const std::string &target_frame)
{
  geometry_msgs::msg::TransformStamped t_stamped;
  geometry_msgs::msg::Pose battery_pose_actual;
  try
  {
    t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 100ms);
  }
  catch (const tf2::TransformException &ex)
  {
    return;
  }

  battery_pose_actual.position.x = t_stamped.transform.translation.x;
  battery_pose_actual.position.y = t_stamped.transform.translation.y;
  battery_pose_actual.position.z = t_stamped.transform.translation.z;
  battery_pose_actual.orientation = t_stamped.transform.rotation;
    

    // Adding color to battery_poses map
    auto color_int = data.part_poses.at(0).part.color ;
    // auto color_int_str = std::to_string(color_int);
    if(battery_poses.count(color_int) == 0){
        // RCLCPP_ERROR(this->get_logger(), "Colors in battery are: %i,", color_int);
        battery_poses[color_int] = battery_pose_actual ;
    }

    

    // publish_data(target_frame, battery_pose_actual);
    
    
}
void ArucoMarker::sort_and_publish(){
    auto length_of_map = battery_poses.size();

    geometry_msgs::msg::PoseArray pose_array;

    // for (const auto &pair : battery_poses) {
    //     RCLCPP_ERROR(this->get_logger(), "Colors int in battery are: %i,", pair.first);
    //         // std::cout << pair.first << '\n';  
    //     }

    for (int i = 0; i < list_of_colors.size(); i++)
    {
        auto col_int = list_of_colors[i];
        auto x_cordinate = battery_poses[col_int].position.x ;
        RCLCPP_ERROR(this->get_logger(), "publishing Colors :%i , x is: %f,", col_int, x_cordinate);


        pose_array.poses.push_back(battery_poses[col_int]);
    }
    
    for (const auto& pose : pose_array.poses) {
    // Access and process each pose
    // For example, access the position and orientation of the pose
    double x_cord = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;

    double orientation_x = pose.orientation.x;
    double orientation_y = pose.orientation.y;
    double orientation_z = pose.orientation.z;
    double orientation_w = pose.orientation.w;

    RCLCPP_ERROR(this->get_logger(), "subs Colors  , x is: %f,", x_cord);
}
    
    pose_array_publisher_->publish(pose_array);
   
// publishs_data_timer_.reset();
 
}



void ArucoMarker::publish_data(const std::string &target_frame, geometry_msgs::msg::Pose battery_pose_actual){
    if (target_frame =="part_1"){
        batttery1_publisher_->publish(battery_pose_actual);
    }else if (target_frame =="part_2"){
        batttery2_publisher_->publish(battery_pose_actual);
    }else if (target_frame =="part_3"){
        batttery3_publisher_->publish(battery_pose_actual);
    }else if (target_frame =="part_4"){
        // RCLCPP_INFO(this->get_logger(),"publishing to 4" );
        batttery4_publisher_->publish(battery_pose_actual);
    }else if (target_frame =="part_5"){
        // RCLCPP_INFO(this->get_logger(),"publishing to 5" );
        batttery5_publisher_->publish(battery_pose_actual);
    }
}

void ArucoMarker::broadcaster_timer()
{
    if(logical_cam_dict.count("lc1")){
         broadcast_cb_(logical_cam_dict["lc1"],"map","camera1_frame","part_1");

    }else
    {
        RCLCPP_INFO(this->get_logger(),"not found" );
    }
    
    if (logical_cam_dict.count("lc2"))
    {
        broadcast_cb_(logical_cam_dict["lc2"],"map","camera2_frame","part_2");
    }else
    {
        RCLCPP_INFO(this->get_logger(),"not found" );
    }
    if (logical_cam_dict.count("lc3"))
    {
            broadcast_cb_(logical_cam_dict["lc3"],"map","camera3_frame","part_3");
    }else
    {
        RCLCPP_INFO(this->get_logger(),"not found" );
    }
    
    if (logical_cam_dict.count("lc4"))
    { 
        broadcast_cb_(logical_cam_dict["lc4"],"map","camera4_frame","part_4");
    }else
    {
        RCLCPP_INFO(this->get_logger(),"not found" );
    }

     if (logical_cam_dict.count("lc5"))
    {
        
       broadcast_cb_(logical_cam_dict["lc5"],"map","camera5_frame","part_5");
    }else
    {
        RCLCPP_INFO(this->get_logger(),"not found" );
    }
    
}

void ArucoMarker::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    clock_time =*msg;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoMarker>("arucomarker");
    rclcpp::spin(node);
    rclcpp::shutdown();
}


