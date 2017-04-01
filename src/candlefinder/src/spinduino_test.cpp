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
      pub.publish(q);
    } else if (counter == 2) {
      q.z = 90;
      pub.publish(q);
    } else if (counter == 4) {
      q.z = 355;
      pub.publish(q);
    } else if (counter == 5) {
      q.z = 470;
      pub.publish(q);
    }
    counter++;
    ros::spinOnce();
    rate.sleep();
  }
}
