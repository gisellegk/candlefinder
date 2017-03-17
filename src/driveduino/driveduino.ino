#include <ros.h>
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

#define DELAY 15
#define STEPS_PER_ROTATION 360 // steps per rotation

int stepperPosition = 0;
int currentAngle = 0;
int currentSpeed = 0;

ros::NodeHandle  nh;

//Angle should be in "steps"
void drive( const geometry_msgs::Twist& msg){
  int angle = msg.angular.z % STEPS_PER_ROTATION;
  //turn to angle
  for(int i = 0; angle - currentAngle != 0; i++) {
    if(msg.angular.z > currentAngle) {
      turnCCW();
    } else if(angle < currentAngle) {
      turnCW();
    } else {
      //why are you turning
    }
  }
  //read x linear for speed & f/b
  analogWrite(ENABLE, abs(msg.linear.x)*255); //x will be betweeen -1 and 1, should multiply by 255 or something?
  if(msg.linear.x >= 0) { //positive =  forward
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACKWARD, LOW);
  } else {
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, HIGH);
  }
  currentSpeed = msg.linear.x;
}

ros::Subscriber<std_msgs::Empty> sub("drive_vector", drive );

geometry_msgs::Twist twist_msg;
ros::Publisher pub("base_pose", &twist_msg);

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
  nh.advertise(pub);
  nh.subscribe(sub);
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
  currentAngle++ % STEPS_PER_ROTATION;
}

