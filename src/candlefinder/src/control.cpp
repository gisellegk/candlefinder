#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

bool start = false;
int flame_x = -1;
int flame_y = -1;
int head_angle = 0;
int base_angle = 0;
int slam_angle = 0;
int navAngle;

int candle_angle;
bool point_a = false;
int center_a;
int flame_a;
bool point_b = false;
int center_b;
int flame_b;

double startTime;

enum STATE {
  CAMSPIN,
  EXPLORE,
  FLAME,
  TRIANGULATE,
  APPROACH,
  EXTINGUISH
};

STATE state;

void saveStartBool(const std_msgs::Bool& msg) {
  if(!start && msg.data) {
    ROS_INFO_STREAM("ALARM DETECTED!!! WEEEEOOOOOOOWEEEEOOOO");
    startTime = ros::Time::now().toSec();
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
  state = CAMSPIN;
  ROS_INFO_STREAM("state: " << state);

  while(ros::ok()) {
    if(start){
      geometry_msgs::Twist t;
      geometry_msgs::Quaternion q;
      int angleDiff;
      int diffFromCenter;
      int thresh = 1;

      double currentTime = ros::Time::now().toSec();
      switch(state){
        //Search & extinguish sequence goes here
        /*
        state CAMSPIN:
        Spin 360 to camscan surrounding area to make sure the fire isn't in starting room.
        */
        case CAMSPIN:
        if(currentTime - startTime < 0.5) {
          t.angular.z = 0;
          t.linear.x = 30;
        } else {
          if(head_angle < 90 || head_angle > 350) {
            q.z = 120;
            t.angular.z = 0;
            t.linear.x = 0;
          } else if (head_angle >= 90 && head_angle < 180) {
            q.z = 240;
          } else if (head_angle >= 180 && head_angle < 270) {
            q.z = 0;
          }
        }
        if(flame_x > 0) {
          t.linear.x = 0;
          state = FLAME;
        }
        driveVectorPub.publish(t);
        headAnglePub.publish(q);
        if(head_angle > 320 && head_angle <= 350)
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
        t.linear.x = 100;
        if(flame_x > 0) {
          t.linear.x = 0;
          state = FLAME;
        }
        driveVectorPub.publish(t);
        break;

        /*
        state FLAME:
        Center camera on flame. Record final head angle position.
        */
        case FLAME:
        // x values range from 0 to 60.
        // This is positive if the flame is to the left of the robot.
        // head will turn CCW if +, CW if -
        diffFromCenter = 30 - flame_x;
        thresh = 2; // a deadly disease

        if(diffFromCenter > thresh || diffFromCenter < -thresh) {
          // didnt feel like importing cmath
          angleDiff = diffFromCenter*40/60;
          q.z = head_angle + angleDiff;
          if(flame_x != -1) {
            t.angular.z = q.z;
            t.linear.x = 0;
            headAnglePub.publish(q);
            driveVectorPub.publish(t);
          }
        } else {
          // Candle is within threshhold of center of FLIR
          ROS_INFO_STREAM("candle centered");
          candle_angle = head_angle;
          state = TRIANGULATE;
        }
        break;

        /*
        state TRIANGULATE:
        move the candle as far right as possible on the screen.
        Record this position, and the x value of the flame.

        move the candle as far left as possible on the screen.
        Record this position, and the x value of the flame.
        */
        case TRIANGULATE:
        if(!point_a){
          if (flame_x < 55) {
            // candle is still on screen
            // keep turning right
            flame_a = flame_x;
            center_a = head_angle;
            q.z = head_angle + 5;
            headAnglePub.publish(q);
            ROS_INFO_STREAM(flame_a);
          } else {
            ROS_INFO_STREAM("found point a");
            point_a = true;
            q.z = head_angle - 10;
            headAnglePub.publish(q);
            ROS_INFO_STREAM(flame_x);
            //set angle to candle_angle
          }
        } else if (!point_b) {
          if(flame_x > 0 || flame_x == -1) {
            // candle still on screen
            // keep turning left
            flame_b = flame_x;
            center_b = head_angle;
            q.z = head_angle - 5;
            headAnglePub.publish(q);
            ROS_INFO_STREAM(flame_b);
          } else {
            point_b = true;
            ROS_INFO_STREAM("found point b");
            ROS_INFO_STREAM(flame_x);
          }
        } else {
          ROS_INFO_STREAM("candle_angle: " << candle_angle);
          ROS_INFO_STREAM("center_a: " << center_a);
          ROS_INFO_STREAM("flame_a: " << flame_a);
          ROS_INFO_STREAM("flame_a deg: " << ((30 - flame_a)*40/60));
          ROS_INFO_STREAM("center_b: " << center_b);
          ROS_INFO_STREAM("flame_b: " << flame_b);
          ROS_INFO_STREAM("flame_b deg: " << ((30 - flame_b)*40/60));
          ROS_INFO_STREAM("time to do math i guess");
          int beta = (30 - flame_b)*40/60;
          int theta_b = abs(center_b - candle_angle);
          int gamma_b = abs( beta - theta_b);
          int the_d_cc = 20 * sin((180-beta)/57.296)/sin(abs(beta-abs(center_b - candle_angle))/57.296);
          
        }

        break;

        /*
        state APPROACH:
        Plot calculated candle location.
        Find closest valid position (not in cost map).
        Drive to candle, keeping head centered on candle or towards candle when possible.

        Move to EXTINGUISH when path is complete.
        */
        case APPROACH:
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
