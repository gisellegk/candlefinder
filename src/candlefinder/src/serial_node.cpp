#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include "serial/serial.h"
#include <string>

int driveAngle = 0; // degrees, 0 to 360
int driveSpeed = 0; // -1 to 1

int targetHeadAngle = 0; // degrees

bool extinguishFlag = false;
serial::Serial * my_serial;

bool writeSerial(std::string);

void sendDriveVector(const geometry_msgs::Twist& msg) {
  if(driveAngle != msg.angular.z) {
    driveAngle = msg.angular.z;
    writeSerial("a" + std::to_string(driveAngle));
  }
  if(driveSpeed != msg.linear.x) {
    driveSpeed = msg.linear.x;
    writeSerial("s" + std::to_string(driveSpeed));
  }
}

void sendTargetHeadAngle(const geometry_msgs::Quaternion& msg) {
  if(targetHeadAngle != msg.z) {
    targetHeadAngle = msg.z;
    writeSerial("h" + std::to_string(targetHeadAngle));
  }
}

void sendExtinguish(const std_msgs::Bool& msg){
  if(extinguishFlag != msg.data){
    extinguishFlag = msg.data;
    writeSerial("e" + std::to_string(extinguishFlag));
  }
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
  msg = msg + "\n";
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

  ros::Subscriber drive_vector_sub = nh.subscribe("drive_vector", 1000, &sendDriveVector);
  ros::Subscriber target_head_angle_sub = nh.subscribe("target_head_angle", 1000, &sendTargetHeadAngle);
  ros::Subscriber extinguish_sub = nh.subscribe("extinguish", 1000, &sendExtinguish);

  ros::Rate rate(10); //idk

  std::string port("/dev/ttyUSB1");
  unsigned long baud = 9600;
  my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));

  while(ros::ok()) {
    //ROS_INFO_STREAM("Serial port recived " << readSerial());
    std::string msg = readSerial();


    ros::spinOnce();
    rate.sleep();

  }
}
