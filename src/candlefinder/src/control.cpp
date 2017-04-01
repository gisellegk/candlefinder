#include <ros/ros.h>

bool start = false;
enum STATE {
  CAMSPIN = 0,
  EXPLORE = 1,
  FLAME = 2
};
void saveStartBool(const std_msgs::Bool& msg) {
  if(!start && msg.data) {
    ROS_INFO_STREAM("ALARM DETECTED!!! WEEEEOOOOOOOWEEEEOOOO");
    //it's 2am pls send help
  }
  start = msg.data;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "cost_map_node");
  ros::NodeHandle nh;

  ros::Subscriber fftSub = nh.subscribe("start_bool", 1000, &saveStartBool);

  ros::Rate rate(10); //idk
  ROS_INFO_STREAM("let's do this!!!");

  while(ros::ok()) {
    if(start){
      //Search & extinguish sequence goes here
      /*
      state CAMSPIN:
        Spin 360 to camscan surrounding area
      state EXPLORE:
        Begin line following, camscanning in the direction you are moving
      state FLAME:
        Center camera on flame. Get camera's angle & direct drive base to that angle. Adjust this angle as you drive forward, keeping the flame horizontally centered in the camera.

        Stop driving forward when costmap wall is hit.

        Move to EXTINGUISH if heat sensor is positive.

        Wiggle the head a little bit for like 5 seconds maybe if it's negative. Move back to EXPLORE if heat sensor is still negative.

      state EXTINGUISH:
        CO2 that shit, Open Servo and move head back and forth a few degrees

      */

    }
    ros::spinOnce();
    rate.sleep();
  }
}
