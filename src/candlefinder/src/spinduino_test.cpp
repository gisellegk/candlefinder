#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

int counter = 0;
geometry_msgs::Quaternion q;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "spinduino_test_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("target_head_angle", 1000);

  ros::Rate rate(1); // 1hz
  ROS_INFO_STREAM("turn head");

  while(ros::ok()) {
    if(counter == 1) {
      q.z = 25;
      ROS_INFO_STREAM(q.z);
      pub.publish(q);
    } else if (counter == 5) {
      q.z = 90;
      ROS_INFO_STREAM(q.z);
      pub.publish(q);
    } else if (counter == 10) {
      q.z = 355;
      ROS_INFO_STREAM(q.z);
      pub.publish(q);
    } else if (counter == 15) {
      q.z = 470;
      ROS_INFO_STREAM(q.z);
      pub.publish(q);
    }
    counter++;
    ROS_INFO_STREAM(q.z);
    ros::spinOnce();
    rate.sleep();
  }
}
