//toppings go here
#include <ros.h>
#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library
#include <Servo.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// IR stuff
IRTherm therm; // Create an IRTherm object to interact with throughout

const byte fire_detect_LED = 13; // Optional LED attached to pin 8 (active low)

float temp = 0;
float amb = 0;

ros::NodeHandle  nh;
std_msgs::Bool bool_msg;
std_msgs::Float32 tempMsg;
ros::Publisher pub("heat_found", &bool_msg);
ros::Publisher tempPub("temperature", &tempMsg);

//CO2 stuff
Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer

void extinguishFlame(const std_msgs::Bool&);
ros::Subscriber<std_msgs::Bool> sub("extinguish", extinguishFlame ); 

std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

void debugPrint(String str) {
  str_msg.data = str.c_str();
 // chatter.publish(&str_msg);
}

void setup()
{
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(tempPub);
  //nh.advertise(chatter);

  therm.begin(); // Initialize thermal IR sensor
  therm.setUnit(TEMP_F);

  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  myservo.write(70);

  pinMode(fire_detect_LED, OUTPUT); // LED pin as output
  setLED(LOW); // LED OFF

}

void loop()
{


  if (therm.read()) // On success, read() will return 1, on fail 0.
  {
    temp = therm.object();
    tempMsg.data = temp;
    tempPub.publish(&tempMsg);
    amb = therm.ambient();
    if(temp > (amb+5)) {
      bool_msg.data = true;
      pub.publish( &bool_msg );
      setLED(HIGH);
    } else {
      bool_msg.data = false;
      pub.publish( &bool_msg );
      setLED(LOW);
    }

    debugPrint("Object: " + String(therm.object(), 2));
    debugPrint("Ambient: " + String(therm.ambient(), 2));
    //these might not play nice with ros string message, not sure
  }
  nh.spinOnce();
  delay(500);
}

void setLED(bool on)
{
  if (on)
  digitalWrite(fire_detect_LED, LOW);
  else
  digitalWrite(fire_detect_LED, HIGH);
}


void extinguishFlame(const std_msgs::Bool& b){
  myservo.write(140);
}
