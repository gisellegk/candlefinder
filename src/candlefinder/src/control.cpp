#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

bool start = false;
int flame_x = -1;
int head_angle = 0;
int base_angle = 0;

enum STATE {
  CAMSPIN = 0,
  EXPLORE = 1,
  FLAME = 2
};

STATE state;

void saveStartBool(const std_msgs::Bool& msg) {
  if(!start && msg.data) {
    ROS_INFO_STREAM("ALARM DETECTED!!! WEEEEOOOOOOOWEEEEOOOO");
    //it's 2am pls send help
  }
  start = msg.data;
}

void saveFlameCoord(const geometry_msgs::Point& msg){
  flame_x = msg.x;
}

void saveCurrentHeadAngle(const geometry_msgs::Quaternion& msg){
  head_angle = msg.z;
}

void saveBasePose(const geometry_msgs::Twist& msg){
  base_angle = msg.angular.z;
  if(msg.linear.x < 0) {
    base_angle += 180;
  }
  base_angle = base_angle % 360;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "cost_map_node");
  ros::NodeHandle nh;

  ros::Publisher headAnglePub = nh.advertise<geometry_msgs::Quaternion>("target_head_angle", 1000);

  ros::Subscriber currentHeadAngleSub = nh.subscribe("current_head_angle", 1000, &saveCurrentHeadAngle);
  ros::Subscriber basePoseSub = nh.subscribe("base_pose", 1000, &saveBasePose);

  ros::Subscriber fftSub = nh.subscribe("start_bool", 1000, &saveStartBool);
  ros::Subscriber flameSub = nh.subscribe("flame_coord", 1000, &saveFlameCoords);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("let's do this!!!");
  state = CAMSPIN;

  while(ros::ok()) {
    if(start){
      switch(state){
      //Search & extinguish sequence goes here
      /*
      state CAMSPIN:
        Spin 360 to camscan surrounding area
      */
      case CAMSPIN:
        geometry_msgs::Quaternion q;
        q.z = 359;
        headAnglePub.publish(q);
        if(head_angle >=350) state = EXPLORE; //probs good enough i guess.
        break;
      /*
      state EXPLORE:
        Begin line following, camscanning in the direction you are moving
      */
      case EXPLORE:
      //this bit noodles the head around in the direction the base is pointing. hopefully.
        if(flame_x >= 0) {
          state = FLAME;
          break;
        }
        int angleDiff = abs(head_angle - base_angle);
        if(angleDiff > 180) 360 - angleDiff;
        if(angleDiff >= 60){
          geometry_msgs::Quaternion q;
          if( base_angle + 60 < head_angle){
            q.z = base_angle - 60;
          }
          else {
            q.z = base_angle + 60;
          }
          headAnglePub.publish(q);
        }
        break;
      /*
      state FLAME:
        Center camera on flame. Get camera's angle & direct drive base to that angle. Adjust this angle as you drive forward, keeping the flame horizontally centered in the camera.

        Stop driving forward when costmap wall is hit.

        Move to EXTINGUISH if heat sensor is positive.

        Wiggle the head a little bit for like 5 seconds maybe if it's negative. Move back to EXPLORE if heat sensor is still negative.
      */
      case FLAME:
        if(flame_x < 0) {
          state = EXPLORE;
          break;
        }
        
        break;
      /*
      state EXTINGUISH:
        CO2 that shit, Open Servo and move head back and forth a few degrees

      */
      case EXTINGUISH:
        break;
    }

    }
    ros::spinOnce();
    rate.sleep();
  }
}
