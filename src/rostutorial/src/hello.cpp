// ROS hello world program :)

#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize the ROS system.
  ros::init(argc, argv, "hello_ros");

  // Establish the program as a ROS node
  ros::NodeHandle nh;

  //Send some output as a log message
  ROS_INFO_STREAM("Hello, ROS");

}
