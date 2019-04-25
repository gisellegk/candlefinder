#include "heat_map.h"

#define MAX_RANGE 300 // max camera range in pixels
#define HFOV 38 // 38 degree horizontal field of view

std::vector<int8_t> hector_map;
std::vector<int8_t> heat_scan(1,-1);
std::vector<int32_t> heatData;

nav_msgs::MapMetaData info;
geometry_msgs::Pose pose;

int robot_row = 0;
int robot_col = 0;

int headAngle = 0;
int robotAngle = 0;

bool first = true;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "heat_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("heat_map", 1000);
  ros::Subscriber mapSub = nh.subscribe("map", 1000, &saveMap);
  ros::Subscriber poseSub = nh.subscribe("slam_out_pose", 1000, &savePose);
  ros::Subscriber headAngleSub = nh.subscribe("current_head_angle", 1000, &saveHeadAngle);
  ros::Subscriber heatArray = nh.subscribe("heat_array", 1000, &saveHeatArray);

    ros::Rate rate(10); //idk
    ROS_INFO_STREAM("heres the camera scan map or something");

    info.width = 1;
    info.height = 1;


    while(ros::ok()) {
      int robotIndex = info.width*robot_row+robot_col;
      //ROS_INFO_STREAM("robotIndex " << robotIndex);
      std::vector<int8_t> m = hector_map;
      //heat_scan[robotIndex] = 100; //robot is here
      int angle = (360+360-(robotAngle + headAngle))%360;

      if(robotIndex != 0) {

        //ROS_INFO_STREAM("ROBOT ANGLE: " << robotAngle << " head angle: " << headAngle << " cam angle: " << angle);
        /*if(first) {
          for(int a = 0; a < 360; a++){
            float rise = sin(a/57.6);
            float run = cos(a/57.6); // hah radians

            int currentPixel = 0;

            //FOR EACH PIXEL IN THIS LINE
            for(int i = 1; m[currentPixel] != 100 && m[currentPixel] >= 0 && i < 22; i++) {
              heat_scan[currentPixel] = 1; // 1 = scanned. i guess. idk.
              currentPixel = robotIndex + (int)((info.width*round(i * run) + round(i*rise)));
            }
            //now move on to the next ray :)
          }
          first = false;
        }*/


        int numRays = 300;
        for(int ray = 0 - (numRays/2); ray < numRays/2; ray++){
          int a = angle + (HFOV / (double) numRays)* ray;
          float rise = sin(a/57.6);
          float run = cos(a/57.6); // hah radians

          int currentPixel = robotIndex;


          int heatValue = scale(heatData[floor((ray+numRays/2)/5)], 3000, 4500, 1, 255);

          //FOR EACH PIXEL IN THIS LINE
          for(int i = 1; m[currentPixel] != 100 && m[currentPixel] >= 0 && i < MAX_RANGE; i++) {
            currentPixel = robotIndex + (int)((info.width*round(i * run) + round(i*rise)));
            if(m[currentPixel] == 100) heat_scan[currentPixel] = (heatValue+heat_scan[currentPixel])/2;
          }
          //now move on to the next ray :)
        }


      }
      nav_msgs::OccupancyGrid c;
      c.data = heat_scan;
      c.info = info;
      pub.publish(c);
      ros::spinOnce();
      rate.sleep();
    }

  }


  void saveMap(const nav_msgs::OccupancyGrid& msg){
    hector_map = msg.data;
    info = msg.info;
    if(heat_scan.size() == 1) { //if and only if this is the first msg. replace with a reeeaal map. or something idk.
      std::vector<int8_t> heat_scan_new(msg.info.width*msg.info.height, -1);
      heat_scan = heat_scan_new; //jankasaurus
    }
  }

  void saveHeatArray(const std_msgs::Int32MultiArray& msg){
    if(sizeof(msg.layout.dim) == 0) return;
    std::vector<int32_t> newHeat(msg.layout.dim[0].size);
    for(int i = 0; i < msg.layout.dim[0].size; i++) {
      newHeat[i] = msg.data[i];
    }
    heatData = newHeat;
  }

  void savePose(const geometry_msgs::PoseStamped& msg){
    pose = msg.pose;
    robotAngle = (int)(round(atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1-2*(pose.orientation.z*pose.orientation.z))*57.3)+270)%360;
    robot_row = (int)((pose.position.y / info.resolution) + (info.height/2.0));
    robot_col = (int)((pose.position.x / info.resolution) + (info.width/2.0));
  }


  void saveHeadAngle(const geometry_msgs::Quaternion& msg){
    headAngle = msg.z;
  }


  int scale(int32_t value, int32_t originalMin, int32_t originalMax, int newMin, int newMax) {
    double percetage = (value-originalMin)/(double)(originalMin-originalMax);
    return percetage * (newMin-newMax)+newMin;
  }
