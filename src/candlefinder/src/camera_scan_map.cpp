#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include "camera_scan_map.h"

int8[] map;
MapMetaData info;
PoseStamped pose;

int robot_row;
int robot_col;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "camera_scan_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("camera_scan_map", 1000);
  ros::Subscriber<nav_msgs::OccupancyGrid> sub("map", saveMap );
  ros::Subscriber<geometry_msgs::PoseStamped> sub("slam_out_pose", savePose );

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("heres the camera scan map or something");

  while(ros::ok()) {
    int8[] m = map;
    int8[] cam_scan = {-1};
    cam_scan[info.width*robot_row+robot_col] = 1;

    OccupancyGrid c;
    c.data = cam_scan;
    c.info = info;
    pub.publish(c);

  }

}

void saveMap(const nav_msgs::OccupancyGrid& msg){
  map = msg.data;
  info = msg.info;
}

void savePose(const geometry_msgs::PoseStamped& msg){
  pose = msg.pose;
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}
