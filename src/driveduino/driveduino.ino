#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

/* Pins for L298N Motor Controller (drive wheels - f/b, speed) */
#define ENABLE 9 //PWM
#define FORWARD 10
#define BACKWARD 11

/* Pins for Stepper Motor Controller (drive angle)*/
#define STEP1 2
#define STEP2 3
#define STEP3 4
#define STEP4 5

#define HOME 8

const long DELAY = 2;
const int STEPS_PER_ROTATION = 2048/4; // steps per rotation

int stepperPosition = 0;
int currentAngle = 0;
int targetAngle = 0;
double currentSpeed = 0;

void receiveMessage(const geometry_msgs::Twist&);


ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("drive_vector", receiveMessage );

geometry_msgs::Twist twist_msg;
ros::Publisher pub("base_pose", &twist_msg);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void debugPrint(String str) {
  str_msg.data = str.c_str();
  chatter.publish(&str_msg);
}

//Angle should be 0 - 360, try to deal with fringe cases
void receiveMessage( const geometry_msgs::Twist& msg){
  // this converts degrees to steps. does not deal with negative angles. try getting rid of mod and using the full double value?
  targetAngle = ((int)msg.angular.z % 360)/360.0*STEPS_PER_ROTATION;
  currentSpeed = msg.linear.x;
}

void setup()
{
  pinMode(HOME, INPUT);
  digitalWrite(HOME, HIGH);
  pinMode(ENABLE, OUTPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(STEP4, OUTPUT);

  nh.initNode();
  nh.advertise(pub);
  nh.advertise(chatter);
  nh.subscribe(sub);

  //Serial.begin(9600);

  findHome();
}

void loop()
{
  if(currentAngle == targetAngle) {
    stepperOff();
    drive(currentSpeed);
  } else {
    drive(0);
    if(targetAngle > currentAngle) {
      turnCCW();
    } else { //if(targetAngle < currentAngle)
      turnCW();
    }
  }
  twist_msg.linear.x = currentSpeed;
  twist_msg.angular.z = currentAngle;
  pub.publish( &twist_msg );
  nh.spinOnce();
  delay(500);
}

void turnCW(){
  digitalWrite(STEP1, HIGH);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, HIGH);
  delay(DELAY);
  digitalWrite(STEP1, HIGH);
  digitalWrite(STEP2, HIGH);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, LOW);
  delay(DELAY);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, HIGH);
  digitalWrite(STEP3, HIGH);
  digitalWrite(STEP4, LOW);
  delay(DELAY);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, HIGH);
  digitalWrite(STEP4, HIGH);
  delay(DELAY);
  currentAngle--;
  if(currentAngle < 0) currentAngle = STEPS_PER_ROTATION - 1;
}

void turnCCW(){

  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, HIGH);
  digitalWrite(STEP3, HIGH);
  digitalWrite(STEP4, LOW);
  delay(DELAY);
  digitalWrite(STEP1, HIGH);
  digitalWrite(STEP2, HIGH);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, LOW);
  delay(DELAY);
  digitalWrite(STEP1, HIGH);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, HIGH);
  delay(DELAY);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, HIGH);
  digitalWrite(STEP4, HIGH);
  delay(DELAY);
  currentAngle = (currentAngle + 1) % STEPS_PER_ROTATION;
}

void stepperOff(){
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, LOW);
  debugPrint("stepper off");
}

void findHome(){
  debugPrint("Finding home...");
  while(digitalRead(HOME)){
    turnCW();
  }
  debugPrint("Done!");
  stepperOff();
  currentAngle = 0;
}

void drive(double spd){
  debugPrint("drive");
  if(spd >= 0) { //positive =  forward
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACKWARD, LOW);
  } else {
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, HIGH);
  }
  analogWrite(ENABLE, abs(spd)*100); //max speed = *255

}
