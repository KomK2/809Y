# Turtlebot Navigation 

## 📌 Overview
This project enables **Turtlebot** to autonomously navigate using **FollowWaypoint Navigation** in **ROS2**, guided by **ArUCo Markers**. The robot reads marker IDs, retrieves parameters from the `params.yaml` file, and sequentially navigates to predefined waypoints.

![Project Environment](https://github.com/KomK2/809Y.git)

---

## 🚀 Features
✅ **Reads ArUCo Marker IDs** to determine navigation sequence.  
✅ **Retrieves waypoint data** from `params.yaml`.  
✅ **Captures part positions** from logical cameras and transforms them relative to the map.  
✅ **Classifies parts** by color and type, then retrieves their positions.  
✅ **Initializes Turtlebot in RVIZ** and generates waypoints dynamically.  
✅ **Sequential navigation** through mapped waypoints.  

---

## 📂 ROS2 Package Details
This project consists of a **ROS2 package** that:
- Captures part positions using logical cameras.
- Stores and retrieves part locations based on their classification.
- Reads **ArUCo Marker IDs** to determine Turtlebot’s navigation path.
- Uses `FollowWaypoint Navigation` to move between predefined waypoints.
- Displays real-time visualization in **RVIZ**.

---

## 🚀 Running the Project
### 📌 Setting Up and Running
1. Clone the repository:
   ```bash
   git clone https://github.com/KomK2/809Y.git
   ```
2. Navigate to the workspace and build the package:
   ```bash
   cd 809Y
   colcon build
   source install/setup.bash
   ```
3. Launch the package:
   ```bash
   ros2 launch turtlebot_navigation navigation.launch.py
   ```
4. The robot will start in the **RVIZ** environment, reading **ArUCo Marker IDs** and following waypoints as per `params.yaml`.

---

## 📚 References
- **ROS2 Navigation Stack**
- **FollowWaypoint Navigation**
- **ArUCo Marker Tracking in ROS2**

For any queries, please reach out via GitHub issues or email. 🚀

