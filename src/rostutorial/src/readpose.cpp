#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>

//callback, each time a new pose message arrives
void poseMessageReceived(const turtlesim::Pose& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed 
    << "position=(" << msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);
}

int main(int argc, char **argv) {
  //init ros system
  ros::init(argc, argv, "subscribe_to_pose"); // name of node 
  ros::NodeHandle nh;

  //subscriber object
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);

  ros::spin(); //let ros run
}