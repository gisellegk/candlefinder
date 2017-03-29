#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>

void saveMap(const nav_msgs::OccupancyGrid&);
void savePose(const geometry_msgs::PoseStamped&);
