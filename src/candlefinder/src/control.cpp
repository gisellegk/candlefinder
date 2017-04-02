#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

bool start = false;
int flame_x = -1;
int flame_y = -1;
int head_angle = 0;
int base_angle = 0;
int slam_angle = 0;
std::vector<uint16_t> pathPoints;

enum STATE {
  CAMSPIN,
  EXPLORE,
  FLAME,
  EXTINGUISH
};

STATE state;

void saveStartBool(const std_msgs::Bool& msg) {
  if(!start && msg.data) {
    ROS_INFO_STREAM("ALARM DETECTED!!! WEEEEOOOOOOOWEEEEOOOO");
    start = true;
    //it's 2am pls send help
  }
}

void savePathPlan(const std_msgs::UInt16MultiArray& msg) {
  pathPoints = msg.data;
}

void saveFlameCoord(const geometry_msgs::Point& msg){
  flame_x = msg.x;
  flame_y = msg.y;
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

void saveSlamPose(const geometry_msgs::PoseStamped& msg) {
  geometry_msgs::Pose pose = msg.pose;
  slam_angle = ((int)round(atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1-2*(pose.orientation.z*pose.orientation.z))*57.3)+360)%360;

}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh;

  ros::Publisher headAnglePub = nh.advertise<geometry_msgs::Quaternion>("target_head_angle", 1000);
  ros::Publisher driveVectorPub = nh.advertise<geometry_msgs::Twist>("drive_vector", 1000);

  ros::Subscriber currentHeadAngleSub = nh.subscribe("current_head_angle", 1000, &saveCurrentHeadAngle);
  ros::Subscriber basePoseSub = nh.subscribe("base_pose", 1000, &saveBasePose);
  ros::Subscriber slamPoseSub = nh.subscribe("slam_out_pose", 1000, &saveSlamPose);
  ros::Subscriber pathPlanSub = nh.subscribe("path_plan", 1000, &savePathPlan);

  ros::Subscriber fftSub = nh.subscribe("start_bool", 1000, &saveStartBool);
  ros::Subscriber flameSub = nh.subscribe("flame_coord", 1000, &saveFlameCoord);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("let's do this!!!");
  state = FLAME;
  ROS_INFO_STREAM("state: " << state);

  while(ros::ok()) {
    if(start){
      geometry_msgs::Twist t;
      geometry_msgs::Quaternion q;
      int angleDiff;
      int offset;
      switch(state){
      //Search & extinguish sequence goes here
      /*
      state CAMSPIN:
        Spin 360 to camscan surrounding area
      */
      case CAMSPIN:
        q.z = 359;
        headAnglePub.publish(q);
        if(head_angle >=350) state = EXPLORE; //probs good enough i guess.
        break;
      /*
      state EXPLORE:
        Begin line following, camscanning in the direction you are moving
      */
      case EXPLORE:
      /*
      get next point
      turn wheels to (left right up down) - current position
      */

        if(pathPoints.size()>1) {
          if(pathPoints[1] > 0) {
            int nextPixel_X = pathPoints[0]/350;
            int nextPixel_Y = pathPoints[0]%350;
            int currentPixel_X = pathPoints[1]/350;
            int currentPixel_Y = pathPoints[1]%350;
            ROS_INFO_STREAM("Current :" << currentPixel_X << ", " << currentPixel_Y);
            ROS_INFO_STREAM("Next :" << nextPixel_X << ", " << nextPixel_Y);
            if(nextPixel_X > currentPixel_X) {
              ROS_INFO_STREAM("down");
              geometry_msgs::Twist v;
              v.angular.z = 360+slam_angle;
              v.linear.z = .5;
              driveVectorPub.publish(v);
            } else if (nextPixel_X < currentPixel_X) {
              ROS_INFO_STREAM("up");
              geometry_msgs::Twist v;
              v.angular.z = 360+slam_angle;
              v.linear.z = .5;
              driveVectorPub.publish(v);
            } else if (nextPixel_Y > currentPixel_Y) {
              ROS_INFO_STREAM("Left");
              geometry_msgs::Twist v;
              v.angular.z = 360+slam_angle+90;
              v.linear.z = .5;
              driveVectorPub.publish(v);
            } else if (nextPixel_Y < currentPixel_Y) {
              ROS_INFO_STREAM("right");
              geometry_msgs::Twist v;
              v.angular.z = 360+slam_angle+90;
              v.linear.z = .5;
              driveVectorPub.publish(v);
            }
          }
        } else {
          ROS_INFO_STREAM("No map");
          ROS_INFO_STREAM("right");
          geometry_msgs::Twist v;
          v.angular.z = 360+slam_angle;
          v.linear.z = 0;
          driveVectorPub.publish(v);
        }
        //this bit noodles the head around in the direction the base is pointing. hopefully.
        if(flame_x >= 0) {
          state = FLAME;
          geometry_msgs::Twist v;
          v.angular.z = base_angle;
          v.linear.z = 0;
          driveVectorPub.publish(v); // should essentially stop the bot
          break;
        }
        q.z = 360+slam_angle;
        headAnglePub.publish(q);
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
          //state = EXPLORE;
          ROS_INFO_STREAM("no flame");
          break;
        }
        offset = (640/2) - flame_x; // a negative number will be to the right?
        if(offset < 0 && abs(offset) > 20) {
          q.z = (head_angle+360 + 3)%360;
          headAnglePub.publish(q);
        } else if (offset > 0 && abs(offset) > 20) {
          q.z = (head_angle+360 - 3)%360;
          headAnglePub.publish(q);
        } else {
          // candle is already centered!
          //figure out how far away the candle is based on y value
          ROS_INFO_STREAM("candle is centered");
          if(false) {
            t.angular.z = head_angle;
            t.linear.x = 1; //??????
            driveVectorPub.publish(t);
          } else {
            t.angular.x = head_angle;
            t.linear.x = 0;
            driveVectorPub.publish(t);
            state = EXTINGUISH;
          }
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
