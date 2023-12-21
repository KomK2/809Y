#include "navigation.hpp"

//===============================================
void navigation::NavigationBloc::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

  message.header.frame_id = "map";
  // Set initial position
  message.pose.pose.position.x = 1.00;
  message.pose.pose.position.y = -1.59;
  message.pose.pose.position.z = 0.0;
  // Set initial orientation
  message.pose.pose.orientation.x = -0.0023328;
  message.pose.pose.orientation.y =  0.0023341;
  message.pose.pose.orientation.z = -0.7069158;
  message.pose.pose.orientation.w = 0.7072901;
  initial_pose_pub_->publish(message);
}
//===============================================
void navigation::NavigationBloc::send_goal() {
  using namespace std::placeholders;

  if (!this->client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses.resize(5);

  for (int i = 0; i < pose_array_data.poses.size() ; i++)
  {
  goal_msg.poses[i].header.frame_id = "map";
  goal_msg.poses[i].pose.position.x = pose_array_data.poses[i].position.x ;
  goal_msg.poses[i].pose.position.y = pose_array_data.poses[i].position.y;
  goal_msg.poses[i].pose.position.z = 0.0;
  goal_msg.poses[i].pose.orientation.x = pose_array_data.poses[i].orientation.x;
  goal_msg.poses[i].pose.orientation.y = pose_array_data.poses[i].orientation.y;
  goal_msg.poses[i].pose.orientation.z = pose_array_data.poses[i].orientation.z;
  goal_msg.poses[i].pose.orientation.w = pose_array_data.poses[i].orientation.w;
  }
 
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavigationBloc::goal_response_callback, this, _1);
  send_goal_options.result_callback =
      std::bind(&NavigationBloc::result_callback, this, _1);

  client_->async_send_goal(goal_msg, send_goal_options);
}

//===============================================
void navigation::NavigationBloc::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

//===============================================
void navigation::NavigationBloc::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
void navigation::NavigationBloc::result_callback(
    const GoalHandleNavigation::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

void  navigation::NavigationBloc::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {

  pose_array_data = *msg ;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received PoseArray message");

    auto pose_array = *msg;
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

    RCLCPP_ERROR(this->get_logger(), "subs Colors , x is: %f,", x_cord);
       RCLCPP_ERROR(this->get_logger(), "subs Colors , x is: %f,", y);
    
    
}

    set_initial_pose();
    // pause for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // send the goal
    send_goal();
    pose_array_subscriber_.reset();
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::NavigationBloc>("navigation_demo");
  rclcpp::spin(node);
  rclcpp::shutdown();
}