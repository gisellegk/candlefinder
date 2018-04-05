#include <Encoder.h>

#define VECTOR_MC_PIN_A   32 //PC5
#define VECTOR_MC_PIN_B   33 //PC4
#define VECTOR_MC_PIN_EN  11 //PB5
#define VECTOR_ENC_PIN_A  20 //PD1
#define VECTOR_ENC_PIN_B  21 //PD0
#define VECTOR_HOME_PIN   8 //PH5

#define HEAD_MC_PIN_A     34 //PC3
#define HEAD_MC_PIN_B     35 //PC2
#define HEAD_MC_PIN_EN    10 //PB4
#define HEAD_ENC_PIN_A    18 //PD3
#define HEAD_ENC_PIN_B    19 //PD2
#define HEAD_HOME_PIN     9 //PH6

#define DRIVE_MC_PIN_A    30 //PC7
#define DRIVE_MC_PIN_B    31 //PC6
#define DRIVE_MC_PIN_EN   12 //PB6

#define SOLENOID_PIN      37 //PC0

#define VIDEO_LED_PIN     27 //PA5
#define FLAME_LED_PIN     26 //PA4

#define ALARM_DETECT_PIN  25 //PA3

#define VECTOR_ENC_PULSES 1440
#define HEAD_ENC_PULSES 30*3.7 // 24 ticks per encoder rev * 3.7 gear ratio

#define VECTORING_THRESHOLD 5

Encoder vectorEncoder(VECTOR_ENC_PIN_A, VECTOR_ENC_PIN_B);
Encoder headEncoder(HEAD_ENC_PIN_A, HEAD_ENC_PIN_B);

String inputString = "";
boolean stringComplete = false;

int targetHeadAngle = 0;
int targetBaseAngle = 0;
int targetSpeed = 0;

void setup() {

  // SET I/O PINS
  pinMode(VECTOR_MC_PIN_A, OUTPUT);
  pinMode(VECTOR_MC_PIN_B, OUTPUT);
  pinMode(VECTOR_MC_PIN_EN, OUTPUT);
  pinMode(VECTOR_HOME_PIN, INPUT);

  pinMode(HEAD_MC_PIN_A, OUTPUT);
  pinMode(HEAD_MC_PIN_B, OUTPUT);
  pinMode(HEAD_MC_PIN_EN, OUTPUT);
  pinMode(HEAD_HOME_PIN, INPUT_PULLUP);

  pinMode(DRIVE_MC_PIN_A, OUTPUT);
  pinMode(DRIVE_MC_PIN_B, OUTPUT);
  pinMode(DRIVE_MC_PIN_EN, OUTPUT);

  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(VIDEO_LED_PIN, OUTPUT);
  pinMode(FLAME_LED_PIN, OUTPUT);

  pinMode(ALARM_DETECT_PIN, INPUT);
  
  Serial.begin(9600);
  inputString.reserve(200);
  
  homeVectorMotors();
  homeHead();
}

int count = 0;
int val = 0;
void loop() {
if (stringComplete) {
    inputString.trim();
    Serial.println(inputString);
    int strLength = inputString.length();
    
    if(strLength >= 2) {
      if(inputString.charAt(0) == 'a') {
        int angle = inputString.substring(1, strLength).toInt();
        char buffer[4];
        sprintf(buffer, "a%03d", angle);
        Serial.println(buffer);
        targetBaseAngle = angle;
      } else if(inputString.charAt(0) == 's') {
        int newSpeed = inputString.substring(1, strLength).toInt();
        char buffer[4];
        sprintf(buffer, "s%03d", newSpeed);
        Serial.println(buffer);
        if(newSpeed > 100) newSpeed = 100;
        targetSpeed = newSpeed;
      } else if(inputString.charAt(0) == 'h') {
        int angle = inputString.substring(1, strLength).toInt();
        char buffer[4];
        sprintf(buffer, "h%03d", angle);
        Serial.println(buffer);
        targetHeadAngle = angle;
      }
      if(inputString.charAt(0) == 'e') {
        bool solenoidValue = (bool)(inputString.substring(1, strLength).toInt() != 0);
        char buffer[2];
        sprintf(buffer, "e%01d", (int)solenoidValue);
        Serial.println(buffer);
        setSolenoid(solenoidValue);
      }
    }
    inputString = "";
    stringComplete = false;
  }
  drive(targetBaseAngle, targetSpeed*0.01);
  count++;
  if(count > 1000) {
    count = 0;
    Serial.println("tick");
  }
  delay(1);
}

void homeVectorMotors() {
    setDriveMC(.05);
    delay(1);
    setVectorMC(-1);
    while(!digitalRead(VECTOR_HOME_PIN)) delay(1);
    setDriveMC(0);
    setVectorMC(0);
    delay(500);
    vectorEncoder.write(0);
    while(!setVectorAngle(20)) delay(1);
    vectorEncoder.write(0);  
}

void homeHead() {
    setHeadMC(1);
    while(digitalRead(HEAD_HOME_PIN)) delay(1);
    setHeadMC(0);
    delay(500);
    headEncoder.write(0);
    //while(!setHeadAngle(90)) delay(1);
    //headEncoder.write(0);  
}

void drive(int angle, float velocity) {
  setDriveMC(velocity*setVectorAngle(angle));
}



int VECTOR_PULSES_PER_QUADRANT = VECTOR_ENC_PULSES / 4;
int HEAD_PULSES_PER_QUADRANT = HEAD_ENC_PULSES / 4;

int vectorSpeedSign = 1;
int headSpeedSign = 1;

int setVectorAngle(int targetAngle) {
  while(targetAngle <0)
    targetAngle += 360;
  targetAngle = targetAngle % 360;  
  
  int currentAngle = vectorEncoder.read() * 360 / VECTOR_ENC_PULSES;
  while(currentAngle <0)
    currentAngle += 360;
  currentAngle = currentAngle % 360;  

  int currentQuadrant = (currentAngle/90) + 1;
  int targetQuadrant = (targetAngle/90) + 1;

  /*Serial.println("\n");
  Serial.print("target angle: "); 
  Serial.println(targetAngle);
  Serial.print("target quadrant: ");
  Serial.println(targetQuadrant);
  Serial.print("current angle: ");
  Serial.println(currentAngle);
  Serial.print("current quadrant: ");
  Serial.println(currentQuadrant);
  Serial.println("\n");
  */

  if((currentQuadrant + 1)%4 == targetQuadrant) {
    setVectorMC(1); //CCW, same dir
    //Serial.println("CCW\n\n");
  } else if ((currentQuadrant + 3)%4 == targetQuadrant) {
    setVectorMC(-1); //CW, same dir
    //Serial.println("CW\n\n");
  } else {
    if(currentQuadrant != targetQuadrant){
      //vectorSpeedSign *= -1; //toggle dir
      targetAngle = (targetAngle + 180) % 360;
      
    }
    if (abs(currentAngle - targetAngle) < VECTORING_THRESHOLD){// we are close enough to the target
      setVectorMC(0);
      if(currentQuadrant != targetQuadrant) return 0;
      else return 1;
    }
    else if(currentAngle > targetAngle)  {
      setVectorMC(-1); //CW
      //Serial.println("CW\n\n");
    }
    else if(currentAngle < targetAngle)  {
      setVectorMC(1); //cCW
      //Serial.println("CCW\n\n");
    }
  }
  return 0;
}

bool setHeadAngle(int angle) {
  while(angle <0)
    angle += 360;
  angle = angle % 360;
  Serial.print("Current Head Angle: ");
  Serial.println(headEncoder.read()*360/HEAD_ENC_PULSES);
  int targetPulse = angle*HEAD_ENC_PULSES/360;
  if(abs(headEncoder.read() - targetPulse) < 5) {
    setHeadMC(0);
    return true;
  } else{
    if(headEncoder.read() < targetPulse) {
      setHeadMC(1);
    } else if(headEncoder.read() > targetPulse) {
      setHeadMC(-1);
    }
  }
  return false;
}

void setHeadMC(int d) {
  if(d == 0) {
    digitalWrite(HEAD_MC_PIN_A, LOW);
    digitalWrite(HEAD_MC_PIN_B, LOW);
    digitalWrite(HEAD_MC_PIN_EN, LOW);
  } else if(d < 0) {
    digitalWrite(HEAD_MC_PIN_A, HIGH);
    digitalWrite(HEAD_MC_PIN_B, LOW);
    digitalWrite(HEAD_MC_PIN_EN, HIGH);
  } else if(d > 0) {
    digitalWrite(HEAD_MC_PIN_A, LOW);
    digitalWrite(HEAD_MC_PIN_B, HIGH);
    digitalWrite(HEAD_MC_PIN_EN, HIGH);
  }
}

void setVectorMC(int d) {
  if(d == 0) {
    digitalWrite(VECTOR_MC_PIN_A, LOW);
    digitalWrite(VECTOR_MC_PIN_B, LOW);
    digitalWrite(VECTOR_MC_PIN_EN, LOW);
  } else if(d < 0) {
    digitalWrite(VECTOR_MC_PIN_A, HIGH);
    digitalWrite(VECTOR_MC_PIN_B, LOW);
    digitalWrite(VECTOR_MC_PIN_EN, HIGH);
  } else if(d > 0) {
    digitalWrite(VECTOR_MC_PIN_A, LOW);
    digitalWrite(VECTOR_MC_PIN_B, HIGH);
    digitalWrite(VECTOR_MC_PIN_EN, HIGH);
  }
}

void setDriveMC(float d) {
  if(d == 0) {
    digitalWrite(DRIVE_MC_PIN_A, LOW);
    digitalWrite(DRIVE_MC_PIN_B, LOW);
    digitalWrite(DRIVE_MC_PIN_EN, LOW);
  } else if(d < 0) {
    digitalWrite(DRIVE_MC_PIN_A, HIGH);
    digitalWrite(DRIVE_MC_PIN_B, LOW);
    analogWrite(DRIVE_MC_PIN_EN, abs(d) * 255);
  } else if(d > 0) {
    digitalWrite(DRIVE_MC_PIN_A, LOW);
    digitalWrite(DRIVE_MC_PIN_B, HIGH);
    analogWrite(DRIVE_MC_PIN_EN, d * 255);
  }
}

void setSolenoid(bool solenoidValue) {
  digitalWrite(SOLENOID_PIN, solenoidValue);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


