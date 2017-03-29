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
    int row = robot_row;
    int col = robot_col;
    int robotIndex = info.width*robot_row+robot_col;
    std::vector<int8_t> m = map;
    std::vector<int8_t>  cam_scan(info.width*info.height, -1);
    cam_scan[robotIndex] = 100; //robot is here

    /*
    for x number of rays
      create the linear function that models the ray
      "walk" along that line, checking the value in map until you hit 100
      if the value of map is 0, add a 1 to the cam_scan map
      if the value of the map is 100 or -1 stop!!

    */
    int numRays = 4;
    for(int ray = 0; ray < numRays; ray++){
      int angle = (360 / numRays)* ray;
      // 0 = 0, 1 = 90, 2=180, 3=270

      float rise = sin(angle);
      float run = cos(angle); // i think?
      int currentPixel = robotIndex;
      //double check that the robot's position is considered "unoccupied"

      //FOR EACH PIXEL IN THIS LINE
      int maxRange = 10; //idk
      for(int i = 1; m[currentPixel] == 0 || i < maxRange; i++) {
        cam_scan[currentPixel] = 1; // 1 = scanned. i guess. idk.
        currentPixel = robotIndex + (int)(i * (info.width*run + rise));
        //idk if you can just add this shit together or what okay
      }
      //now move on to the next ray :)
    }

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
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}
