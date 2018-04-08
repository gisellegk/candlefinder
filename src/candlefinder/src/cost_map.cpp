#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>

void saveMap(const nav_msgs::OccupancyGrid&);
void changePixel(int, int);

std::vector<int8_t> hector_map;
std::vector<int8_t> cost_map(1,-1);

int cost_radius = 20;

nav_msgs::MapMetaData info;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "cost_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("cost_map", 1000);
  ros::Subscriber mapSub = nh.subscribe("map", 1000, &saveMap);

  ros::Rate rate(30); //idk
  ROS_INFO_STREAM("map.inflate();");

  info.width = 1;
  info.height = 1;

  int cost_radiusSQ = cost_radius * cost_radius;
  while(ros::ok()) {
    cost_map = hector_map;

    for(int i = 0; i < cost_map.size(); i++){
      //are you ready!!!!
      //for each pixel in the map
      if(hector_map[i] > 50){
        int i_x = floor(i / info.width);
        int i_y = i % info.width;

        for(int x = 0; x < cost_radius; x++) {
          for(int y = 1; y < cost_radius ; y++) {
            if(x * x + y * y < cost_radiusSQ){
              changePixel(i_x+x, i_y+y);//q1
              changePixel(i_x-y, i_y+x);//q2
              changePixel(i_x-x, i_y-y);//q3
              changePixel(i_x+y, i_y-x);//q4
            }
          }
        }
      }

    }

    nav_msgs::OccupancyGrid c;
    c.data = cost_map;
    c.info = info;
    pub.publish(c);
    ros::spinOnce();
    rate.sleep();
  }
}

void changePixel(int x, int y){
  int coord = info.width*x+y;
  if(x > 0 && y > 0 && coord < info.width*info.height){
    if(hector_map[coord] > 50) {
      cost_map[coord] = 100;
    } else {
      cost_map[coord] = 99;
    }
  }
}

void saveMap(const nav_msgs::OccupancyGrid& msg){
  hector_map = msg.data;
  info = msg.info;
  if(cost_map.size() == 1) {
    std::vector<int8_t> cost_map_new(msg.info.width*msg.info.height, -1);
    cost_map = cost_map_new; //jankasaurus
  }
}
