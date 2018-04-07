# candlefinder
ROS candlefinder

Root folder should be a ROS workspace.


# Topics
## Candlefinder topics:

### start_bool
Published by: serial_node
Subscribed to by:
Message type: std_msgs::Bool

__msg.data__: Boolean value, "true" if the robot should start.

### drive_vector
Published by: control, teleop_robot (robot_teleop.cpp)
Subscribed to by: serial_node
Message type: geometry_msgs::Twist

__msg.angular.z__: The target angle for the robot to turn, in degrees. This is an absolute position relative to the base's start position.
__msg.linear.x__: The target speed for the robot, from 0 to 100.

### base_pose
Published by: serial_node
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp)
Message Type: geometry_msgs::Twist

__msg.angular.z__: The current angle of the robot's drive base, in degrees. This is an absolute position relative to the base's start position.
__msg.linear.x__: The current speed for the robot, from 0 to 100.

### target_head_angle
Published by: navigation_node (navigation.cpp)
Subscribed to by: serial_node
Message Type: geometry_msgs::Quaternion

__msg.z__: The target angle for the robot's head to turn, in degrees. This is an absolute position relative to the head's start position.

### current_head_angle
Published by: serial_node
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp)
Message Type: geometry_msgs::Quaternion

__msg.z__: The current angle of the robot's head, in degrees. This is an absolute position relative to the head's start position.

### camera_scan_map
Published by: camera_scan_map_node (camera_scan_map.cpp)
Subscribed to by: navigation_node (navigation.cpp)
Message Type: nav_msgs::OccupancyGrid

__msg.info__: A collection of useful information about the map, including its width, height, and resolution. See nav_msgs::MapMetaData.

__msg.data__: An array of integers representing the map surrounding the robot, indicating where the robot has already scanned with its camera.

- -1 = unexplored
- 1 = camera has seen here
- 100 = robot is here (currently gets overridden anyways)

### cost_map
Published by: cost_map_node (cost_map.cpp)
Subscribed to by: navigation_node (navigation.cpp)
Message Type: nav_msgs::OccupancyGrid

__msg.info__: A collection of useful information about the map, including its width, height, and resolution. See nav_msgs::MapMetaData.

__msg.data__: An array of integers representing the map surrounding the robot, except all wall obstacles have been inflated, indicating where the center of the robot should avoid in order to prevent collisions.

- -1 = unexplored
- 0 = mapped and unoccupied.
- 75 = buffer zone?
- 99 = inflated obstacle area
- 100 = obstacle from hector_map

### nav_map
Published by: navigation_node (navigation.cpp)
Subscribed to by: control
Message Type: nav_msgs::OccupancyGrid

__msg.info__: A collection of useful information about the map, including its width, height, and resolution. See nav_msgs::MapMetaData.

__msg.data__: An array of integers

- -1 = unexplored
- 0 = in current search window
- 50 = right-angle only path
- 100 = path to be followed

### navGoal
Published by:
Subscribed to by: navigation_node (navigation.cpp)
Message Type: geometry_msgs::Point

__msg.info__: Target for pathfinding algorithim. If set to -1,-1 it is ignored, otherwise, the algorithm trys to find a path to this specific point

### exploration_target_angle
Published by: navigation_node (navigation.cpp)
Subscribed to by: control
Message Type: std_msgs::Int16

__msg.data__: An integer representing the angle the robot should turn in order to drive to the nearest unexplored area. Range: 0 to 360, always positive.

### candle_loc
Published by: FLIR_node (FLIR_node.py)
Subscribed to by:
Message Type: geometry_msgs::Point

__msg.x__: The x-position of the flame in the camera's vision. -1 if none available.

__msg.y__: The y-position of the flame in the camera's vision. -1 if none available.

### extinguish
Published by:
Subscribed to by: serial_node
Message Type: std_msgs::Bool

__msg.data__: Boolean value, true if CO2 valve should be opened

## Hector topics:
### map
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp), cost_map_node (cost_map.cpp)
Message Type: nav_msgs::OccupancyGrid

### slam_out_pose
Subscribed to by: camera_scan_map_node (camera_scan_map.cpp), navigation_node (navigation.cpp)
Message Type: geometry_msgs::PoseStamped

## Serial Reference:

read and write to serial port:
  /dev/ttyUSB1 (or adjust as needed)
  9600 baud

## Read and publish:
  start_bool:
    - g* for go
  base_pose
    - a* for angle
    - s* for speed
  current_head_angle
    - h* for head angle

## Process and Write to serial:
  drive_vector
    - a* for angle
    - s* for speed
  target_head_angle
    - h* for head angle
  extinguish
    - e* for extinguish flag
