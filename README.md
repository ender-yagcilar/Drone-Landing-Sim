# ArUco Marker Landing Algorithm using ROS Noetic, OpenCV, MAVROS (C++)

![ArUco Marker]![MarkerData_50](https://github.com/ender-yagcilar/Drone-Marker-Landing/assets/136696120/87fc995d-9b5d-40d8-bf9b-7ff7662dbbe3)


## Requirements:
- OpenCV (I hope there won't be version conflicts, but I'm not sure)
- MAVROS

## Installation:
1. Build the project using `catkin_make`. The first `catkin_make` may result in an error as the service might not be detected initially. However, it will install the service during the first `catkin_make`. Subsequent `catkin_make` commands will install the `drone_pkg`.

## Running the Drone Node:
To run the drone node, use the following command:
```
rosrun drone_pkg vision_node
```
This node processes the images from the drone's camera and publishes the marker's position to the topic `/Vision/MarkerPosition`. For error analysis related to position calculations, refer to the method `marker_detection_error()` in the node. Note that the variable `marker_position_ground_truth[3]` should be updated with the actual position of the marker in Gazebo for accurate error analysis. The center of the marker in Gazebo is not the center of the marker but its bottom edge.

## Running the Drone Movement Node:
To run the drone movement node, use the following command:
```
rosrun drone_pkg drone_node
```
**Important:** Before running this node, ensure that you have seen the message "READY TO TAKEOFF" in the simulation. If not, wait until you see it; otherwise, you will encounter a "PREFLIGHT FAIL: EKF HORIZ POS ERROR" and the drone won't arm. This node contains general methods related to the drone's movement. To modify the behavior of the drone, make necessary uncomment/comment changes in the `Drone::work()` method.

## Running Both Nodes Together:
Both nodes can be launched together using the `ender_nodes.launch` file:
```
roslaunch drone_pkg ender_nodes.launch
```

Feel free to modify the nodes or experiment with the code as per your requirements. Happy coding!
