#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

/* Pins for L298N Motor Controller (drive wheels - f/b, speed) */
#define ENABLE 3 //PWM
#define FORWARD 4
#define BACKWARD 5

/* Pins for Stepper Motor Controller (drive angle)*/
#define STEP1 6
#define STEP2 7
#define STEP3 8
#define STEP4 9

#define HOME 10

const long DELAY = 2;
const int STEPS_PER_ROTATION = 2048/4; // steps per rotation

int stepperPosition = 0;
int currentAngle = 0;
int currentSpeed = 0;

void drive(const geometry_msgs::Twist&);


ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("drive_vector", drive );

geometry_msgs::Twist twist_msg;
ros::Publisher pub("base_pose", &twist_msg);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void debugPrint(String str) {
  str_msg.data = str.c_str();
  chatter.publish(&str_msg);
}

//Angle should be in "steps"
void drive( const geometry_msgs::Twist& msg){
  int angle = (int)msg.angular.z % STEPS_PER_ROTATION;
  //turn to angle
  for(int i = 0; angle - currentAngle != 0; i++) {
    //Serial.println((String)"current: " + currentAngle + (String)" target: " + angle);
    debugPrint("turning " + String(angle) + ", " + String(currentAngle));
    if(angle > currentAngle) {
      turnCCW();
    } else if(angle < currentAngle) {
      turnCW();
    } else {
      //why are you turning
      //Serial.println("angle = current angle");
    }
  }
  stepperOff();
  //read x linear for speed & f/b
  analogWrite(ENABLE, abs(msg.linear.x)*255); //x will be betweeen -1 and 1, should multiply by 255 or something?
  //Serial.println("Speed: " + (long)msg.linear.x);
  debugPrint("drive speed");
  if(msg.linear.x >= 0) { //positive =  forward
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACKWARD, LOW);
  } else {
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, HIGH);
  }
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
  while(!digitalRead(HOME)){
    turnCW();
  }
  debugPrint("Done!");
  stepperOff();
  currentAngle = 0;
}
