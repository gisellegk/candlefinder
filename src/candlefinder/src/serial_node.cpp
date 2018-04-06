#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include "serial/serial.h"
#include <string>

std::string::size_type sz;   // alias of size_t

//stuff to publish
geometry_msgs::Twist base_pose_msg;
geometry_msgs::Quaternion current_head_angle_msg;
std_msgs::Bool start_bool_msg;

//stuff to write:
int driveAngle = 0; // degrees, 0 to 360
int driveSpeed = 0; // 0 to 100

int targetHeadAngle = 0; // degrees

bool extinguishFlag = false;

serial::Serial * my_serial;

bool writeSerial(std::string);

void sendDriveVector(const geometry_msgs::Twist& msg) {
  if(driveAngle != msg.angular.z) {
    driveAngle = msg.angular.z;
    //ROS_INFO_STREAM("sending drive angle:");
    //ROS_INFO_STREAM(driveAngle);
    writeSerial("a" + std::to_string(driveAngle));
  }
  if(driveSpeed != msg.linear.x) {
    driveSpeed = msg.linear.x;
    //ROS_INFO_STREAM("sending drive speed:");
    //ROS_INFO_STREAM(driveSpeed);
    writeSerial("s" + std::to_string(driveSpeed));
  }
}

void sendTargetHeadAngle(const geometry_msgs::Quaternion& msg) {
  if(targetHeadAngle != msg.z) {
    targetHeadAngle = msg.z;
    //ROS_INFO_STREAM("sending head angle:");
    //ROS_INFO_STREAM(targetHeadAngle);
    writeSerial("h" + std::to_string(targetHeadAngle));
  }
}

void sendExtinguish(const std_msgs::Bool& msg){
  if(extinguishFlag != msg.data){
    extinguishFlag = msg.data;
    //ROS_INFO_STREAM("sending extinguish flag:");
    //ROS_INFO_STREAM(extinguishFlag);
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

  ros::Subscriber drive_vector_sub = nh.subscribe("drive_vector", 1000, &sendDriveVector);
  ros::Subscriber target_head_angle_sub = nh.subscribe("target_head_angle", 1000, &sendTargetHeadAngle);
  ros::Subscriber extinguish_sub = nh.subscribe("extinguish", 1000, &sendExtinguish);

  ros::Rate rate(100); //idk

  std::string port("/dev/ttyUSB1");
  unsigned long baud = 9600;
  my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));

  base_pose_msg.angular.z = 0;
  base_pose_msg.linear.x = 0;
  current_head_angle_msg.z = 0;
  start_bool_msg.data = false;

  while(ros::ok()) {
    //ROS_INFO_STREAM("Serial port recived " << readSerial());
    std::string msg = readSerial();

    //msg.trim();
    //ROS_INFO_STREAM(msg);
    int strLength = msg.length();

    if(strLength >= 2) {
      if(msg.at(0) == 'r') {
        if(msg.at(1) == 'a') {
          int newDriveAngle = std::stoi(msg.substr(2, strLength), &sz);
          //ROS_INFO_STREAM("base pose angle:");
          //ROS_INFO_STREAM(newDriveAngle);
          base_pose_msg.angular.z = newDriveAngle;
          base_pose_pub.publish(base_pose_msg);
        } else if(msg.at(1) == 's') {
          int newSpeed = std::stoi(msg.substr(2, strLength), &sz);
          if(newSpeed > 100) newSpeed = 100;
          driveSpeed = newSpeed;
          //ROS_INFO_STREAM("base pose speed:");
          //ROS_INFO_STREAM(newSpeed);
          base_pose_msg.linear.x = newSpeed;
          base_pose_pub.publish(base_pose_msg);
        } else if(msg.at(1) == 'h') {
          int newHeadAngle = std::stoi(msg.substr(2, strLength), &sz);
          //ROS_INFO_STREAM("current head angle:");
          //ROS_INFO_STREAM(newHeadAngle);
          current_head_angle_msg.z = newHeadAngle;
          current_head_angle_pub.publish(current_head_angle_msg);
        } else if(msg.at(1) == 'g') {
          bool startVal = (bool)(std::stoi(msg.substr(2, strLength), &sz));
          //ROS_INFO_STREAM("alarm detect publish:");
          //ROS_INFO_STREAM(startVal);
          start_bool_msg.data = startVal;
          start_bool_pub.publish(start_bool_msg);
        }
      }
    }


    ros::spinOnce();
    rate.sleep();

  }
}
