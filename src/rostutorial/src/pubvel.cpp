// random velocity for random turtles

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

  srand(time(0));

  ros::Rate rate(2); // 2Hz

  while(ros::ok()) {
    //create and build message
    geometry_msgs::Twist msg;
    msg.linear.x = double(rand())/double(RAND_MAX);
    msg.angular.z = 2*double(rand())/double(RAND_MAX) -1;

    //publish message
    pub.publish(msg);

  //print info
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

  rate.sleep();
  }
}