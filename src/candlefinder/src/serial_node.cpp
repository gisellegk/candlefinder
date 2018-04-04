#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include "serial/serial.h"
#include <string>

/*
read and write to serial port.
/dev/ttyUSB1
9600

Read and publish:
  start_bool
  base_pose
  current_head_angle
  chatter

Process and Write to serial:
  drive_vector
  target_head_angle
  extinguish

*/

int driveAngle = 0; // degrees, 0 to 360
int driveSpeed = 0; // -1 to 1

int targetHeadAngle = 0; // degrees

bool extinguishFlag = false;
serial::Serial * my_serial;

void saveDriveVector(const geometry_msgs::Twist& msg) {
  driveAngle = msg.angular.z;
  driveSpeed = msg.linear.x;
}

void saveTargetHeadAngle(const geometry_msgs::Quaternion& msg) {
  targetHeadAngle = msg.z;
}

void saveExtinguish(const std_msgs::Bool& msg){
  extinguishFlag = msg.data;
}


std::string readSerial(){
  bool open = my_serial->isOpen();
  size_t available = my_serial->available();
  std::string str = "";
  if(open && available > 0) {
    str = (my_serial->readline());
  }
  return str;
}

bool writeSerial(std::string msg){
  bool open = my_serial->isOpen();
  if(open){
    my_serial->write(msg);
  }
  return open;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;

  ros::Publisher start_bool_pub = nh.advertise<std_msgs::Bool>("start_bool", 1000);
  ros::Publisher base_pose_pub = nh.advertise<geometry_msgs::Twist>("base_pose", 1000);
  ros::Publisher current_head_angle_pub = nh.advertise<geometry_msgs::Quaternion>("current_head_angle", 1000);
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Subscriber drive_vector_sub = nh.subscribe("drive_vector", 1000, &saveDriveVector);
  ros::Subscriber target_head_angle_sub = nh.subscribe("target_head_angle", 1000, &saveTargetHeadAngle);
  ros::Subscriber extinguish_sub = nh.subscribe("extinguish", 1000, &saveExtinguish);

  ros::Rate rate(10); //idk

  std::string port("/dev/ttyUSB1");
  unsigned long baud = 9600;
  my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));

  while(ros::ok()) {
    ROS_INFO_STREAM("Serial port recived " << readSerial());

    ros::spinOnce();
    rate.sleep();

  }
}
