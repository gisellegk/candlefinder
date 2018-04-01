#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/MapMetaData.h>
#include <queue>

void saveMap(const nav_msgs::OccupancyGrid&);
void saveCamMap(const nav_msgs::OccupancyGrid&);
void savePose(const geometry_msgs::PoseStamped&);
int XYtoCords(int, int);
geometry_msgs::Quaternion toQuaternion(double, double, double);
