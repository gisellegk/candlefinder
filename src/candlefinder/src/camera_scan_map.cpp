#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include "camera_scan_map.h"

std::vector<int8_t> hector_map;
std::vector<int8_t> cam_scan(1,-1);

nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int robot_row = 0;
int robot_col=0;

int headAngle = 0;
int robotAngle= 0;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "camera_scan_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("camera_scan_map", 1000);
  ros::Subscriber mapSub = nh.subscribe("map", 1000, &saveMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);
  ros::Subscriber headAngleSub = nh.subscribe("current_head_angle", 1000, &saveHeadAngle);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("heres the camera scan map or something");

  info.width = 1;
  info.height = 1;


  while(ros::ok()) {
    int robotIndex = info.width*robot_row+robot_col;
    ROS_INFO_STREAM("robotIndex " << robotIndex);
    std::vector<int8_t> m = hector_map;
    cam_scan[robotIndex] = 100; //robot is here
    int angle = headAngle + robotAngle;

    if(robotIndex != 0) {
      int numRays = 100;
      for(int ray = 0 - (numRays/2); ray < numRays/2; ray++){
        int a = angle + (60.0 / numRays)* ray;
        float rise = sin(a/57.6);
        float run = cos(a/57.6); // hah radians

        int currentPixel = robotIndex;

        //FOR EACH PIXEL IN THIS LINE
        int maxRange = 25; //idk
        for(int i = 1; m[currentPixel] != 100 && m[currentPixel] >= 0 && i < maxRange; i++) {
          cam_scan[currentPixel] = 1; // 1 = scanned. i guess. idk.
          currentPixel = robotIndex + (int)((info.width*round(i * run) + round(i*rise)));
        }
        //now move on to the next ray :)
      }
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
  hector_map = msg.data;
  info = msg.info;
  if(cam_scan.size() == 1) { //if and only if this is the first msg. replace with a reeeaal map. or something idk.
    std::vector<int8_t> cam_scan_new(msg.info.width*msg.info.height, -1);
    cam_scan = cam_scan_new; //jankasaurus
  }
}

void savePose(const geometry_msgs::PoseStamped& msg){
  pose = msg.pose;
  robotAngle = pose.orientation.z;
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}


void saveHeadAngle(const geometry_msgs::Quaternion& msg){
  headAngle = msg.z;
}
