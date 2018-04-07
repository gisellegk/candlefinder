#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Int16.h>
#include <queue>
#include "navigation.h"
#include <math.h>

std::vector<int8_t> cost_map;
std::vector<int8_t> cam_map(1,-1);
std::vector<int8_t> nav_map(1,-1);

nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int OFFEST = 2;
int ANGULAR_OFFSET = 10;

int robot_row = 0;
int robot_col=0;

int headAngle = 0;
int robotAngle= 0;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle nh;

  ros::Publisher nav_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("nav_map", 1000);
  ros::Publisher exploration_target_angle_pub = nh.advertise<std_msgs::Int16>("exploration_target_angle", 1000);
  ros::Subscriber mapSub = nh.subscribe("cost_map", 1000, &saveMap);
  ros::Subscriber camSub = nh.subscribe("camera_scan_map", 1000, &saveCamMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("heres the nav map or something");

  info.width = 1;
  info.height = 1;


  while(ros::ok()) {
    std::vector<int8_t> m(info.width*info.height,-1);// = nav_map;
    std::vector<int> pathPoints;
    std::vector<geometry_msgs::Pose> vectors;
    int robotPos = info.width*robot_row+robot_col;
    if(nav_map.size() != 1 && cam_map.size() != 1 &&  robotPos > 0) {
      bool stuckInCostMap = (cost_map[robotPos] == 99);
      //ROS_INFO_STREAM("Stuck in costmap: " << stuckInCostMap);
      int finalTarget = -1;
      std::queue<int> frontier;
      frontier.push(robotPos);
      std::vector<int> came_from(info.width*info.height, -1);
      came_from[robotPos] = robotPos;
      while(frontier.size() > 0 && finalTarget == -1) {
        int currentPixel =  frontier.front();
        frontier.pop();
        //ROS_INFO_STREAM("camvalhere " << (cam_map[currentPixel] != 1));
        //ROS_INFO_STREAM("cm: " << (float)cost_map[currentPixel]);
        if((cam_map[currentPixel] != 1 && !stuckInCostMap) || (stuckInCostMap && cost_map[currentPixel] == 0)) {
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
              //ROS_INFO_STREAM("cm: " << (float)cost_map[next]);
              if(cost_map[next] <= 50 && cost_map[next] >= 0 || (stuckInCostMap)) {
                frontier.push(next);
              }
              came_from[next] = currentPixel;
            }
          }
        }
      }
      if(finalTarget!=-1 ) {
        int linePos = finalTarget;
        do {
          pathPoints.insert(pathPoints.begin(),linePos);
          m[linePos] = 50;
          linePos = came_from[linePos];
          int x = linePos/info.width;
          int y = linePos%info.width;
        } while(linePos != robotPos);
      }

      int numberOfPathPoints = pathPoints.size();
      if(numberOfPathPoints > 1) {
        std::vector<uint16_t> newPathPoints;
        int s = 0;

        while(s < numberOfPathPoints - 1) {
          //ROS_INFO_STREAM("s: " << s);
          for(int e = numberOfPathPoints - 1; e > s; e--) {
            bool lineofsight = false;
            float start_X = pathPoints[s]/info.width;
            float start_Y = pathPoints[s]%info.width;
            //ROS_INFO_STREAM("PS x: " << start_X << " y: " << start_Y << " p: " << XYtoCords(start_X, start_Y));
            float end_X = pathPoints[e]/info.width;
            float end_Y = pathPoints[e]%info.width;

            float diff_X = end_X - start_X;
            float diff_Y = end_Y - start_Y;

            float angle = atan(diff_Y/diff_X);

            if(diff_X > 0 && diff_Y > 0) {
              //first quadrant
            } else if(diff_X < 0 && diff_Y > 0) {
              //second quadrant
              angle+=M_PI;
            } else if(diff_X < 0 && diff_Y <= 0) {
              //third quadrant
              angle+=M_PI;
            } else if(diff_X > 0 && diff_Y <= 0) {
              //fourth quadrant
              angle+=2*M_PI;
            }

            int distance = ceil(sqrt(pow(diff_Y,2)+pow(diff_X,2)));
            for(int dd = 0; dd < distance+1; dd++) {
              int x = round(start_X + dd * cos(angle));
              int y = round(start_Y + dd * sin(angle));
              //ROS_INFO_STREAM("x: " << x << " y: " << y << " d: " << distance << " p: " << XYtoCords(x,y));
              //ROS_INFO_STREAM("cm: " << (float)cost_map[XYtoCords(x,y)]);
              if(x < 0 ||  x > info.width-1 || y < 0 || y> info.height-1) continue;
              if(cost_map[x*info.width + y] != 99 || stuckInCostMap) {
                if(x == end_X && y == end_Y)
                  lineofsight = true;
              } else {
                break;
              }
            }
            if(lineofsight) {
              geometry_msgs::Pose newVector;
              //ROS_INFO_STREAM("posX: " << (float)start_X << " posY: " << (float)start_Y);
              //ROS_INFO_STREAM("tarX: " << (float)(robotPos/info.width) << " tarY: " <<  (float)(robotPos%info.width));
              //newVector.position.x = (start_X - (info.width/2.0)) * info.resolution;
              //newVector.position.y = (start_Y - (info.height/2.0)) * info.resolution;
              newVector.position = pose.position;
              newVector.orientation = toQuaternion(0, 0, angle);
              if(vectors.size() == 0){
                int length_L = 0;
                int START_X_L = round(start_X + OFFEST * cos(angle-M_PI/2));
                int START_Y_L = round(start_Y + OFFEST * sin(angle-M_PI/2));
                for(length_L = 0; length_L < 100; length_L++){
                  int x = round(START_X_L + length_L * cos(angle-(ANGULAR_OFFSET/57.2958)));
                  int y = round(START_Y_L + length_L * sin(angle-(ANGULAR_OFFSET/57.2958)));
                  if(x > 0 && y > 0 && x < info.width && y < info.height) {
                    if(cost_map[x*info.width+y] == 99) break;
                  } else break;
                }

                int length_R = 0;
                int START_X_L = round(start_X + OFFEST * cos(angle+M_PI/2));
                int START_Y_L = round(start_Y + OFFEST * sin(angle+M_PI/2));
                for(length_L = 0; length_L < 100; length_L++){
                  int x = round(START_X_L + length_L * cos(angle+(ANGULAR_OFFSET/57.2958)));
                  int y = round(START_Y_L + length_L * sin(angle+(ANGULAR_OFFSET/57.2958)));
                  if(x > 0 && y > 0 && x < info.width && y < info.height) {
                    if(cost_map[x*info.width+y] == 99) break;
                  } else break;
                }

                std_msgs::Int16 msg;
                int globalTargetAngle = ((int)(-(angle*57.295779-90)+360)%360);
                int targetAngle = (globalTargetAngle - robotAngle + 360) % 360;


                if(length_L > length_R + 2 && length_L > 2) {
                  //go a little to the left
                  targetAngle -= ANGULAR_OFFSET;
                } else if(length_R > length_L + 2 && length_R > 2) {
                  //go a little to the right
                  targetAngle += ANGULAR_OFFSET;
                }


                msg.data = targetAngle;
                exploration_target_angle_pub.publish(msg);
                //ROS_INFO_STREAM("robot angle: " << robotAngle << "deg, global target angle: " << globalTargetAngle << "deg");
                ROS_INFO_STREAM("target angle " << targetAngle);
              }
              vectors.push_back(newVector);
              for(int dd = 0; dd < distance; dd++){
               int x = round(start_X + dd * cos(angle));
               int y = round(start_Y + dd * sin(angle));
               m[x*info.width+y] = 100;
               newPathPoints.push_back(x*info.width+y);
             }
             s = e;
            }
          }
        }
      } else if (numberOfPathPoints == 1) {
        float start_X = robotPos/info.width;
        float start_Y = robotPos%info.width;
        //ROS_INFO_STREAM("PS x: " << start_X << " y: " << start_Y << " p: " << XYtoCords(start_X, start_Y));
        float end_X = pathPoints[0]/info.width;
        float end_Y = pathPoints[0]%info.width;

        float diff_X = end_X - start_X;
        float diff_Y = end_Y - start_Y;

        float angle = atan(diff_Y/diff_X);

        if(diff_X > 0 && diff_Y > 0) {
          //first quadrant
        } else if(diff_X < 0 && diff_Y > 0) {
          //second quadrant
          angle+=M_PI;
        } else if(diff_X < 0 && diff_Y <= 0) {
          //third quadrant
          angle+=M_PI;
        } else if(diff_X > 0 && diff_Y <= 0) {
          //fourth quadrant
          angle+=2*M_PI;
        }

        std_msgs::Int16 msg;
        int globalTargetAngle = ((int)(-(angle*57.295779-90)+360)%360);
        int targetAngle = (globalTargetAngle - robotAngle + 360) % 360;
        msg.data = targetAngle;
        exploration_target_angle_pub.publish(msg);
        ROS_INFO_STREAM("SINGLE POINT: target angle " << targetAngle);

      }
    }
    nav_msgs::OccupancyGrid c;
    c.data = m;
    c.info = info;
    nav_map_pub.publish(c);
    //std_msgs::UInt16MultiArray p;
    //p.data = pathPoints;
    //path_plan_pub.publish(p);
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
  robotAngle = (int)(360 + round(atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1-2*(pose.orientation.z*pose.orientation.z))*57.3))%360;
  robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
  robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
}

int XYtoCords(int x, int y) {
  return x*info.width + y;
}

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw) {
	geometry_msgs::Quaternion q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}
