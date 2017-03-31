#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>

void saveMap(const nav_msgs::OccupancyGrid&);
void savePose(const geometry_msgs::PoseStamped&);

std::vector<int8_t> hector_map;
std::vector<int8_t> robot_outline(1,-1);

nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int robot_row = 0;
int robot_col=0;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "robot_outline_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_outline", 1000);
  ros::Subscriber mapSub = nh.subscribe("map", 1000, &saveMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("trying to draw a circle around dat robot");

  info.width = 1;
  info.height = 1;

  while(ros::ok()) {
    int robotIndex = info.width*robot_row+robot_col;
    std::vector<int8_t> m = hector_map;
    robot_outline[robotIndex] = 100; //robot is here
    int x = 5;
    int y = 0;
    int err = 0;

    while (x >= y)
    {
      robot_outline[robotIndex + info.width*x + y]
      robot_outline[robotIndex + info.width*y + x]
      robot_outline[robotIndex - info.width*x + y]
      robot_outline[robotIndex - info.width*y + x]
      robot_outline[robotIndex - info.width*x - y]
      robot_outline[robotIndex - info.width*y - x]
      robot_outline[robotIndex + info.width*y - x]
      robot_outline[robotIndex - info.width*x - y]

      if (err <= 0)
      {
        y += 1;
        err += 2*y + 1;
      }
      if (err > 0)
      {
        x -= 1;
        err -= 2*x + 1;
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
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}
