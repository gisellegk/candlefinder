#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include "camera_scan_map.h"

std::vector<int8_t> map;
nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int robot_row = 0;
int robot_col=0;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "camera_scan_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("camera_scan_map", 1000);
  ros::Subscriber mapSub = nh.subscribe("map", 1000, &saveMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("heres the camera scan map or something");

  info.width = 1;
  info.height = 1;

  while(ros::ok()) {
    std::vector<int8_t> m = map;
    std::vector<int8_t>  cam_scan(info.width*info.height, -1);
    cam_scan[info.width*robot_row+robot_col] = 1;

    nav_msgs::OccupancyGrid c;
    c.data = cam_scan;
    c.info = info;
    pub.publish(c);
    ros::spinOnce();
    rate.sleep();
  }

}

void saveMap(const nav_msgs::OccupancyGrid& msg){
  map = msg.data;
  info = msg.info;
}

void savePose(const geometry_msgs::PoseStamped& msg){
  pose = msg.pose;
  double angle = atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z);

  float x = (pose.position.y / info.resolution) + (info.height/2.0);
  float y = ((pose.position.x+1) / info.resolution) + (info.width/2.0);
  robot_row = (int)(sqrt(x^2+y^2)*cos(atan(y/x)+angle));
  robot_col = (int)(sqrt(x^2+y^2)*sin(atan(y/x)+angle));
}
