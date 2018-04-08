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
#define HEAD_ENC_PULSES 600
//(195*3.7) // 24 ticks per encoder rev * 3.7 gear ratio

#define VECTORING_THRESHOLD 5
#define HEAD_THRESHOLD 2

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

  //homeHeadAndVector();
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
        sprintf(buffer, "ra%03d", angle);
        Serial.println(buffer);
        targetBaseAngle = angle;
      } else if(inputString.charAt(0) == 's') {
        int newSpeed = inputString.substring(1, strLength).toInt();
        char buffer[4];
        sprintf(buffer, "rs%03d", newSpeed);
        Serial.println(buffer);
        if(newSpeed > 100) newSpeed = 100;
        targetSpeed = newSpeed;
      } else if(inputString.charAt(0) == 'h') {
        int angle = inputString.substring(1, strLength).toInt();
        targetHeadAngle = angle;
      }
      if(inputString.charAt(0) == 'e') {
        bool solenoidValue = (bool)(inputString.substring(1, strLength).toInt() != 0);
        char buffer[2];
        sprintf(buffer, "re%01d", (int)solenoidValue);
        Serial.println(buffer);
        setSolenoid(solenoidValue);
      }
    }
    inputString = "";
    stringComplete = false;
  }

  
  drive(targetBaseAngle, targetSpeed*0.01);
  setHeadAngle(targetHeadAngle);

  if(digitalRead(ALARM_DETECT_PIN)) {
    Serial.println("rg1");
  }


  
  count++;
  if(count > 1000) {
    count = 0;       
    char buffer[4];
    sprintf(buffer, "rs%03d", targetSpeed);
    Serial.println(buffer);
  }
  if(count % 100 == 0) {
    int currentHeadAngle = headEncoder.read() * 360 / HEAD_ENC_PULSES;
    while(currentHeadAngle < 0) currentHeadAngle += 360;
    char buffer[4];
    sprintf(buffer, "rh%03d", currentHeadAngle);
    Serial.println(buffer);
  }
  delay(1);
}

void homeHeadAndVector() {
  setDriveMC(.05);
  delay(1);
  setVectorMC(-1);
  setHeadMC(1);
  while(!digitalRead(VECTOR_HOME_PIN) || digitalRead(HEAD_HOME_PIN)) 
  {
    if(digitalRead(VECTOR_HOME_PIN)) {
      setDriveMC(0);
      setVectorMC(0);
    }
    if(!digitalRead(HEAD_HOME_PIN)){
      setHeadMC(0);
    }
    delay(1);
  }
  delay(500);
  vectorEncoder.write(0);
  headEncoder.write(0);
  while(true){
    delay(1);
    if(setVectorAngle(20) != 0 && setHeadAngle(135) != 0) break;
  }
  vectorEncoder.write(0); 
  headEncoder.write(0);
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
    while(!setVectorAngle(35)) delay(1);
    vectorEncoder.write(0);  
    setVectorMC(0);
    delay(500);
}

void homeHead() {
    setHeadMC(1);
    while(digitalRead(HEAD_HOME_PIN)) delay(1);
    setHeadMC(0);
    delay(500);
    headEncoder.write(0);
    while(!setHeadAngle(135)) delay(1);
    headEncoder.write(0);  
}

void drive(int angle, float velocity) {
  setDriveMC(velocity*setVectorAngle(angle));
}

float setVectorAngle(int targetAngle) {
  while(targetAngle <0)
    targetAngle += 360;
  targetAngle = targetAngle % 360;  
  
  int currentAngle = vectorEncoder.read() * 360 / VECTOR_ENC_PULSES;
  while(currentAngle <0)
    currentAngle += 360;
  currentAngle = currentAngle % 360;  

  /*Serial.println("\n");
  Serial.print("target angle: "); 
  Serial.println(targetAngle);
  Serial.print("current angle: ");
  Serial.println(currentAngle);
  Serial.println("\n");
  */

  if (abs(currentAngle - targetAngle) < VECTORING_THRESHOLD){// we are close enough to the target
    setVectorMC(0);
    return 1;
  } else {
    int currentAngle_opposite = (currentAngle + 180) % 360;
    int targetAngle_opposite = (targetAngle + 180) % 360;

    int diffs[4];
    diffs[0] = currentAngle - targetAngle;
    diffs[1] = currentAngle - targetAngle_opposite;
    diffs[2] = currentAngle_opposite - targetAngle_opposite;
    diffs[3] = currentAngle_opposite - targetAngle;

    int smallest = 0;
    for(int i = 1; i < 4; i++){
      if(abs(diffs[i]) < abs(diffs[smallest])){
        smallest = i;
      }
    }

     if(abs(diffs[smallest]) > VECTORING_THRESHOLD) {
      if(diffs[smallest] > 0) {
        setVectorMC(-1); // diffs is positive, go clockwise
        //Serial.println("Clockwise");
        //Serial.println("\n");
       } else {
        setVectorMC(1); //diffs is negative, go ccw
        //Serial.println("Counter-clockwise");
        //Serial.println("\n");
       }
     } else {
      setVectorMC(0);
      if(abs(currentAngle - targetAngle) <= VECTORING_THRESHOLD || abs(abs(currentAngle - targetAngle) - 360) <= VECTORING_THRESHOLD) {
        return 1; 
      } else {
        return -1;
      }
     }     

    if(abs(currentAngle - targetAngle) <= 2*VECTORING_THRESHOLD || abs(abs(currentAngle - targetAngle) - 360) <= 2*VECTORING_THRESHOLD) {
      return 0.5;
    } else if (abs(abs(currentAngle - targetAngle) - 180) <= 2*VECTORING_THRESHOLD) {
      return -0.5;
    } else {
      return 0;
    }
  }  
}

bool setHeadAngle(int targetAngle) {
  
  while(targetAngle <0)
    targetAngle += 360;
  targetAngle = targetAngle % 360;  
  
  int currentAngle = headEncoder.read() * 360 / HEAD_ENC_PULSES;
  while(currentAngle <0)
    currentAngle += 360;
  currentAngle = currentAngle % 360;

  int diff = currentAngle - targetAngle;
  float dirCoeff = 1;

  if(abs(targetAngle - currentAngle) < HEAD_THRESHOLD || abs(abs(targetAngle - currentAngle)-360) < HEAD_THRESHOLD ) {
    setHeadMC(0);
    return true;
  } else {
    if(abs(diff) < 180) { // LESS THAN 180
      //Negative # = CCW
      //Positive #  = CW
      dirCoeff = 1;
    } else { // GREATER THAN 180 switch directions
      //Negative # = CW
      //Positive #  = CCW
      dirCoeff = -1;
    }
    if(abs(targetAngle - currentAngle) < HEAD_THRESHOLD  * 10 || abs(abs(targetAngle - currentAngle)-360) < HEAD_THRESHOLD * 10) {
      dirCoeff *= 0.2;
    }
    if(diff < 0) { //  if neg diff less than 180 go ccw
      setHeadMC(dirCoeff * -1);
    } else { // if pos diff less than 180 go cw
      setHeadMC(dirCoeff * 1);
    }
    return false;
  }
}

void setHeadMC(float d) {
  if(d == 0) {
    digitalWrite(HEAD_MC_PIN_A, LOW);
    digitalWrite(HEAD_MC_PIN_B, LOW);
    digitalWrite(HEAD_MC_PIN_EN, LOW);
  } else if(d < 0) {
    digitalWrite(HEAD_MC_PIN_A, HIGH);
    digitalWrite(HEAD_MC_PIN_B, LOW);
    analogWrite(HEAD_MC_PIN_EN, abs(d) * 255);
  } else if(d > 0) {
    digitalWrite(HEAD_MC_PIN_A, LOW);
    digitalWrite(HEAD_MC_PIN_B, HIGH);
    analogWrite(HEAD_MC_PIN_EN, d * 255);
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


