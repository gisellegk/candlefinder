#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Int32MultiArray.h>

void saveMap(const nav_msgs::OccupancyGrid&);
void savePose(const geometry_msgs::PoseStamped&);
void saveHeadAngle(const geometry_msgs::Quaternion&);
void saveHeatArray(const std_msgs::Int32MultiArray&);
int scale(int32_t value, int32_t originalMin, int32_t originaMax, int newMin, int newMax);
