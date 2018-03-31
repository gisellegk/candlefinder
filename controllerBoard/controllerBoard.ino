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

const float VECTOR_ENC_PULSES = 1440;

Encoder vectorEncoder(VECTOR_ENC_PIN_A, VECTOR_ENC_PIN_B);
Encoder headEncoder(HEAD_ENC_PIN_A, HEAD_ENC_PIN_B);

String inputString = "";
boolean stringComplete = false;

int targetAngle = 0;
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
  pinMode(HEAD_HOME_PIN, INPUT);

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
}

int count = 0;
int val = 0;
void loop() {
if (stringComplete) {
    inputString.trim();
    Serial.println(inputString);
    int strLength = inputString.length();
    if(strLength > 3) {
      if(inputString.charAt(0) == 'a') {
        int angle = inputString.substring(1, strLength).toInt();
        Serial.print("Angle: ");
        Serial.println(angle);
        targetAngle = angle;
      } else if(inputString.charAt(0) == 's') {
        int newSpeed = inputString.substring(1, strLength).toInt();
        Serial.print("Speed: ");
        Serial.println(newSpeed);
        if(newSpeed > 100) newSpeed = 100;
        targetSpeed = newSpeed;
      }
    }
    inputString = "";
    stringComplete = false;
  }
  setVectorAngle(targetAngle);
  setDriveMC(targetSpeed*0.01);
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
    setVectorMC(1);
    while(!digitalRead(VECTOR_HOME_PIN)) delay(1);
    setDriveMC(0);
    setVectorMC(0);
    delay(500);
    vectorEncoder.write(0);
    while(!setVectorAngle(30)) delay(1);
   vectorEncoder.write(0);  
}

bool setVectorAngle(int angle) {
  while(angle <0)
    angle += 360;
  angle = angle % 360;
  //Serial.print("Current Angle: ");
  //Serial.println(vectorEncoder.read()*360/VECTOR_ENC_PULSES);
  int targetPulse = angle*VECTOR_ENC_PULSES/360;
  if(abs(vectorEncoder.read()- targetPulse) < 5) {
    setVectorMC(0);
    return true;
  } else if(vectorEncoder.read() < targetPulse) {
    setVectorMC(1);
  } else if(vectorEncoder.read() > targetPulse) {
    setVectorMC(-1);
  }
  return false;
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

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


