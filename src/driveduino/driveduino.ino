/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

/* Pins for L298N Motor Controller (drive wheels - f/b, speed) */
#define ENABLE 3 //PWM
#define FORWARD 4
#define BACKWARD 5

/* Pins for Stepper Motor Controller (drive angle)*/
#define STEP1 6
#define STEP2 7
#define STEP3 8
#define STEP4 9

ros::NodeHandle  nh;


void drive( const geometry_msgs::Twist& msg){
  //read x linear for speed & f/b
  analogWrite(ENABLE, abs(msg.linear.x)); //x will be betweeen -1 and 1, should multiply by 255 or something?
  
}

ros::Subscriber<std_msgs::Empty> sub("drive_vector", drive );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";


void setup()
{
  pinMode(ENABLE, OUTPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(STEP4, OUTPUT);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
