#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/UInt16MultiArray.h>
#include <queue>
#include "navigation.h"

std::vector<int8_t> cost_map;
std::vector<int8_t> cam_map(1,-1);
std::vector<int8_t> nav_map(1,-1);

nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int robot_row = 0;
int robot_col=0;

int headAngle = 0;
int robotAngle= 0;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle nh;

  ros::Publisher nav_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("nav_map", 1000);
  ros::Publisher path_plan_pub = nh.advertise<std_msgs::UInt16MultiArray>("path_plan", 1000);
  ros::Subscriber mapSub = nh.subscribe("cost_map", 1000, &saveMap);
  ros::Subscriber camSub = nh.subscribe("camera_scan_map", 1000, &saveCamMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("heres the nav map or something");

  info.width = 1;
  info.height = 1;


  while(ros::ok()) {
    std::vector<int8_t> m(info.width*info.height,-1);// = nav_map;
    std::vector<uint16_t> pathPoints;
    int robotPos = info.width*robot_row+robot_col;
    if(nav_map.size() != 1 && cam_map.size() != 1 &&  robotPos > 0) {
      int finalTarget = -1;
      std::queue<int> frontier;
      frontier.push(robotPos);
      std::vector<int> came_from(info.width*info.height, -1);
      came_from[robotPos] = robotPos;

      while(frontier.size() > 0 && finalTarget == -1) {
        int currentPixel =  frontier.front();
        frontier.pop();
        ROS_INFO_STREAM("camvalhere " << (cam_map[currentPixel] != 1));
        if(cam_map[currentPixel] != 1) {
          finalTarget = currentPixel;
        } else {
          m[currentPixel] = 0;
          int currentPixel_X = currentPixel/info.width;
          int currentPixel_Y = currentPixel%info.width;
          //brace yourself...
          for(int i = 0; i < 4; i++) {
            int a = 90*i;
            int rise = round(sin(a/57.6));
            int run = round(cos(a/57.6)); // hah radians
            int next = XYtoCords(currentPixel_X + run, currentPixel_Y + rise);
            if(came_from[next] == -1) {
              if(cost_map[next] <= 50 && cost_map[next] >= 0) {
                frontier.push(next);
              }
              came_from[next] = currentPixel;
            }
          }
        }
      }
      if(finalTarget!=-1 ) {
        int linePos = finalTarget;
        do{
          pathPoints.insert(pathPoints.begin(),linePos);
          m[linePos] = 100;
          linePos = came_from[linePos];
        } while(linePos != robotPos);
      }
    }

    nav_msgs::OccupancyGrid c;
    c.data = m;
    c.info = info;
    nav_map_pub.publish(c);
    std_msgs::UInt16MultiArray p;
    p.data = pathPoints;
    path_plan_pub.publish(p);
    ros::spinOnce();
    rate.sleep();
}

}

void saveMap(const nav_msgs::OccupancyGrid& msg){
  cost_map = msg.data;
  info = msg.info;
  if(nav_map.size() == 1) { //if and only if this is the first msg. replace with a reeeaal map. or something idk.
    std::vector<int8_t> nav_map_new(msg.info.width*msg.info.height, -1);
    nav_map = nav_map_new; //jankasaurus
  }
}

void saveCamMap(const nav_msgs::OccupancyGrid& msg){
  cam_map = msg.data;
  if(cam_map.size() == 1) { //if and only if this is the first msg. replace with a reeeaal map. or something idk.
    std::vector<int8_t> cam_map_new(msg.info.width*msg.info.height, -1);
    cam_map = cam_map_new; //jankasaurus
  }
}

void savePose(const geometry_msgs::PoseStamped& msg){
  pose = msg.pose;
  robotAngle = round(atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1-2*(pose.orientation.z*pose.orientation.z))*57.3);
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}

int XYtoCords(int x, int y) {
  return x*info.width + y;
}
