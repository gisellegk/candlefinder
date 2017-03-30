#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

/* Pins for Stepper Motor Controller (camera angle)*/
#define STEP1 2
#define STEP2 3
#define STEP3 4
#define STEP4 5

#define HOME 13

const long DELAY = 5;
const int STEPS_PER_ROTATION = 2048/4; // steps per rotation

int stepperPosition = 0;
int currentAngle = 0;
int targetAngle = 0;

void receiveMessage(const geometry_msgs::Quaternion&);

ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Quaternion> sub("target_head_angle", receiveMessage );

geometry_msgs::Quaternion angle_msg;
ros::Publisher pub("current_head_angle", &angle_msg);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void debugPrint(String str) {
  str_msg.data = str.c_str();
  chatter.publish(&str_msg);
}

//Angle should be 0 - 360, try to deal with fringe cases
void receiveMessage( const geometry_msgs::Quaternion& msg){
  // this converts degrees to steps. does not deal with negative angles. try getting rid of mod and using the full double value?
  targetAngle = ((int)msg.z % 360)/360.0*STEPS_PER_ROTATION;
}

void setup()
{
  pinMode(HOME, INPUT);
  digitalWrite(HOME, HIGH);
  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(STEP4, OUTPUT);

  nh.initNode();
  nh.advertise(pub);
  nh.advertise(chatter);
  nh.subscribe(sub);

  findHome();
}

void loop()
{
  if(currentAngle == targetAngle) {
    stepperOff();
  } else {
    if(targetAngle > currentAngle) {
      turnCCW();
    } else { //if(targetAngle < currentAngle)
      turnCW();
    }
  }
  angle_msg.z = currentAngle;
  pub.publish( &angle_msg );
  nh.spinOnce();
  delay(00);
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
