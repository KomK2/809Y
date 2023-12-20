cmake_minimum_required(VERSION 3.8)
project(group29_final)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder whish has all the header files
include_directories(include/group29_final)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  ros2_aruco_interfaces
  mage_msgs
  nav2_msgs
  rclcpp_action
  rclcpp_components 
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(arucomarker src/Arucomarker.cpp)
ament_target_dependencies(arucomarker ${FRAME_DEMO_INCLUDE_DEPENDS})

add_executable(navigation src/navigation.cpp)
ament_target_dependencies(navigation ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS arucomarker navigation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


#-----------------------------
# Installation
#-----------------------------


install(TARGETS
arucomarker
navigation
# marker_listener_sub
# battery_finder
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()