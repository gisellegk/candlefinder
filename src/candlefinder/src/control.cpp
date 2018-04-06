#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

bool start = false;
int flame_x = -1;
int flame_y = -1;
int head_angle = 0;
int base_angle = 0;
int slam_angle = 0;
int navAngle;

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

void saveNavAngle(const std_msgs::Int16& msg) {
  navAngle = msg.data;
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
  ros::Subscriber navAngleSub = nh.subscribe("exploration_target_angle", 1000, &saveNavAngle);

  ros::Subscriber fftSub = nh.subscribe("start_bool", 1000, &saveStartBool);
  ros::Subscriber flameSub = nh.subscribe("candle_loc", 1000, &saveFlameCoord);

  ros::Rate rate(20); //idk
  ROS_INFO_STREAM("let's do this!!!");
  state = EXPLORE;
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
        Spin 360 to camscan surrounding area to make sure the fire isn't in starting room.
      */
      case CAMSPIN:
        q.z = 180;
        headAnglePub.publish(q);

        if(head_angle >=350)
          state = EXPLORE; //probs good enough i guess.
        break;
      /*
      state EXPLORE:
        Begin following navigation, camscanning in the direction you are moving
      */
      case EXPLORE:
        q.z = navAngle;
        headAnglePub.publish(q);
        t.angular.z = navAngle;
        t.linear.x = 50;
        if(flame_x > 0) {
          t.linear.x = 0;
          state = FLAME;
        }
        driveVectorPub.publish(t);
        break;
      /*
      state FLAME:
        Center camera on flame. Get camera's angle & direct drive base to that angle. Adjust this angle as you drive forward, keeping the flame horizontally centered in the camera.

        Stop driving forward when costmap wall is hit.

        Move to EXTINGUISH if heat sensor is positive.

        Wiggle the head a little bit for like 5 seconds maybe if it's negative. Move back to EXPLORE if heat sensor is still negative.
      */
      case FLAME:


        break;
      /*
      state EXTINGUISH:
        CO2 that shit, Open solenoid and move head back and forth a few degrees

      */
      case EXTINGUISH:
        break;
    }

    }
    ros::spinOnce();
    rate.sleep();
  }
}
