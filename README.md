# candlefinder
ROS candlefinder

Root folder should be a ROS workspace.


# Topics
## Candlefinder topics:
### drive_vector
Published by:
Subscribed to by: driveduino
Message type: geometry_msgs::Twist

__msg.angular.z__: The target angle for the robot to turn, in degrees. This is an absolute position relative to the base's start position.
__msg.linear.x__: The target speed for the robot, from -1 to 1.

### base_pose
Published by: driveduino
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp)
Message Type: geometry_msgs::Twist

__msg.angular.z__: The current angle of the robot's drive base, in degrees. This is an absolute position relative to the base's start position.
__msg.linear.x__: The current speed for the robot, from -1 to 1.

### chatter
Published by: driveduino, spinduino
Subscribed to by:
Message Type: std_msgs::String

__msg.data__: String of information to be printed. Good for debugging arduino code!

### target_head_angle
Published by: spinduino_test
Subscribed to by: spinduino
Message Type: geometry_msgs::Quaternion

__msg.z__: The target angle for the robot's head to turn, in degrees. This is an absolute position relative to the head's start position.

### current_head_angle
Published by: spinduino
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp)
Message Type: geometry_msgs::Quaternion

__msg.z__: The current angle of the robot's head, in degrees. This is an absolute position relative to the head's start position.

### camera_scan_map
Published by: camera_scan_map_node (camera_scan_map.cpp)
Subscribed to by:
Message Type: nav_msgs::OccupancyGrid

__msg.info__: A collection of useful information about the map, including its width, height, and resolution. See nav_msgs::MapMetaData.

__msg.data__: An array of integers representing the map surrounding the robot, indicating where the robot has already scanned with its camera.

- -1 = unexplored
- 1 = camera has seen here
- 100 = robot is here (currently gets overridden anyways)

### flame_coord
Published by: opencv_candlefinder (opencv_node.cpp)
Subscribed to by:
Message Type: geometry_msgs::Point

__msg.x__: The x-position of the flame in the camera's vision

__msg.y__: The y-position of the flame in the camera's vision

## Hector topics:
### map
### slam_out_pose
