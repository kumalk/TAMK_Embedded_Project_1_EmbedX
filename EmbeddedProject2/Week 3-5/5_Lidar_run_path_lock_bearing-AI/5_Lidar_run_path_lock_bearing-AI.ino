//---------------------------------------
// Tampere Universisty of Appied Science
// Team : EmbedX
// Final Aruduino Code 
// Date : 09/12/2025 
// Team Members :
//    Prashantha Kumanayake
//    Lasanthi Ayesha
//    Nimeshika Rodrigo 
//---------------------------------------

#include <LiquidCrystal.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

//---Creating Lidar------
LIDARLite_v4LED myLIDAR;
int newDistance;
int DistanceSampleSize=10;
//FOr Room measurement and store calculated values
float RoomData[6]; //[fwDist,riDist,bkDist,lfDist,calcArea,clacVolume]
volatile boolean followFlag=false;


//--- Configuration Constants ---
#define Motor_forward   1
#define Motor_return    0
#define CMPS14_ADDRESS  0x60 //I2C address for compass

// --- Pin Definitions ---
// Motor Pins
#define Motor_L_dir_pin 8
#define Motor_R_dir_pin 7
#define Motor_L_pwm_pin 10
#define Motor_R_pwm_pin 9

// Joystick Pins
#define ANALOG_X_PIN A8
#define ANALOG_Y_PIN A9 
#define ANALOG_BUTTON_PIN 19 

// LCD Pins
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32; 

// --- Global Variables ---
String controlMode = "ESP"; 
volatile unsigned long lastInterruptTime = 0; 
volatile bool buttonPressedFlag = false; 

// Motor Encoderand Compass Variabls
volatile long encoderCount_left = 0;
volatile long encoderCount_right = 0;
float distperpuls = 1.2; // 1.2 pulses per mm
volatile float bearingDegrees = 0;
int bearingMaxError = 2; // tolarance in bearing angle for turning accuracy
int tuningSpeed = 20;  // This is motor speed percentage.When turning speed increase accuracy decreases  
bool motorRunning = false; // status of motors
int joyX_val = 0; // Raw joystick X reading for LCD
int joyY_val = 0; // Raw joystick Y reading for LCD

// Travel Plan
int travelPlan[1][5] = {
  {75,0,'f',0,'l'} // {Speed as %, Distance(mm), 'f'/'b', Angle(deg), 'l'/'r'}. 
  //.  'f'/'b' - moving direction | 'l'/'r' turning direction. 
};
int currentTargetPulsCount = 0;  
int currentTravelSection = 0;
int totalSectionsInTravelPlan = sizeof(travelPlan) / sizeof(travelPlan[0]);  // this code is previously designed to run preplanned path as multiple sections , so to keep that ability for future as well , I kept the skelton without breaking

//Path planinig.  //{rotation,traveldistance,finalGapLimit}
int followPlan[4][4] = {
  {0,'u',30,'l'},
  {90,'u',15,'l'},
  {90,'u',30,'l'},
  {90,'u',15,'l'}
  
};

float GapValue;
float MaxErrGap = 0.5;
float currentDistance;
boolean isBearingLocked = false;
float LockedbearingDegrees;
int currentPathStep=0;
int totalSectionsInPathPlan = sizeof(followPlan) / sizeof(followPlan[0]);

volatile boolean pathFlag=false;

String LCDcommandText = "";

// Joystick Calibration Values
int maxX[] = {0,522,1023}; 
int maxY[] = {0,494,1023}; 

// LCD Object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

// --- Structures ---
struct Joystick {
  int x = 0;
  int y = 0;
};
 
// ====================================================================
// --- SETUP ---
// ====================================================================
void setup() {
  lcd.begin(16, 2); 
  Wire.begin(); 

  // Joystick Button Setup (Mode Toggle)
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANALOG_BUTTON_PIN), joyPressed, FALLING); 

  // Motor Pins Setup
  pinMode(Motor_L_dir_pin, OUTPUT); 
  pinMode(Motor_R_dir_pin, OUTPUT); 
  pinMode(Motor_L_pwm_pin, OUTPUT); 
  pinMode(Motor_R_pwm_pin, OUTPUT); 

  // Encoder Pins Setup
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countEncoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countEncoder_right, RISING);
  
  Serial.begin(115200);

  //LIDAR detection
  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("LIDAR acknowledged!");
}

// ====================================================================
// --- LOOP ---
// ====================================================================
void loop() {
  // --- Print average distace from LIDAR
  //Serial.print("Avg distance: ");
  //Serial.print(getAvgDistance(7));
  //Serial.println("cm");

  //follow logic
  if(followFlag){
    GapValue = 30.00;
    MaxErrGap = 0.50;
    currentDistance = getAvgDistance(3);
    updateScreen();
    Serial.println(currentDistance);
    if(currentDistance > GapValue+ MaxErrGap){
      //run motor forward
      if(currentDistance >GapValue+  3*MaxErrGap){
      //run motor forward slowly
      runMotors(50,'f',0,'r');
      }else{
        //run fast forward
        runMotors(20,'f',0,'r');
      }

    }else if(currentDistance < GapValue - MaxErrGap){
      //run motor backward
      if(currentDistance < GapValue-  3*MaxErrGap){
      //run motor backward slowly
      runMotors(50,'b',0,'r');
      }else{
        //run fast backward
        runMotors(20,'b',0,'r');
      }
    }else{
      stopMotors();
    }
  }

  //Path distance keeping logic
if(pathFlag) {
    currentDistance = getAvgDistance(3);
    float bearingError = bearingDegrees - LockedbearingDegrees;

    // Normalize error for 360-degree wrap-around
    if (bearingError > 180) bearingError -= 360;
    if (bearingError < -180) bearingError += 360;

    // 1. Check if heading is significantly off (Use a 2-3 degree buffer)
    if(abs(bearingError) > 3.0) {
        char correctDir;
        getRotationAngle(LockedbearingDegrees, correctDir);
        // Fix ONLY to the locked degree, do not call handleFollowPath
        turnToBearing(LockedbearingDegrees, correctDir); 
    } 
    // 2. If heading is okay, manage distance
    else {
        if(currentDistance > GapValue + MaxErrGap) {
            runMotors(25, 'f', 0, 'r'); // Drive straight
        } else if(currentDistance < GapValue - MaxErrGap) {
            runMotors(25, 'b', 0, 'r'); // Back up
        } else {
            // STEP COMPLETE
            stopMotors();
            pathFlag = false; 
            if(currentPathStep + 1 < totalSectionsInPathPlan) {
                currentPathStep++;
                startNextPathStep(); // Move to the next plan item
            }
        }
    }
}


  // --- Check for Mode Change ---
  if (buttonPressedFlag) {
    buttonPressedFlag = false;
    stopMotors(); 
    Serial.print("Current Mode: ");
    Serial.println(controlMode);
    LCDcommandText = ""; // Clear command text on mode switch
  }
  
  // --- Common Sensor Read ---
  bearingDegrees = readBearing16Bit(); 

  // --- Control Mode Logic ---
  if (controlMode == "ESP") {
      // 1. Serial Command Processing
      if (Serial.available() > 0){
        String message = Serial.readStringUntil('\n');
        Serial.print("Message received, content: ");
        Serial.println(message);
        processCommand(message);
      }
  
  }else if (controlMode == "JOY") {
    
    // 1. Read Joystick and Control Motors
    Joystick joystick; 
    joystick.x = analogRead(ANALOG_X_PIN); 
    joystick.y = analogRead(ANALOG_Y_PIN); 
    
    // Store raw values for LCD display
    joyX_val = joystick.x;
    joyY_val = joystick.y; 
    
    int* joyPercents = joystickPercentages(joystick.x, joystick.y); 

    runMotors_joy(joyPercents[0], joyPercents[1]);

    // Debug output in Serial monitor
    Serial.print("JOY Mode - X%: ");
    Serial.println(joyPercents[0]);
    Serial.print("JOY Mode - Y%: ");
    Serial.println(joyPercents[1]);
  }

  // 2. Start Travel Plan
      if (buttonPressedFlag) {
        buttonPressedFlag = false;
        currentTravelSection = 0;
        encoderCount_left = 0;
        encoderCount_right = 0;
        currentTargetPulsCount = targetPulsCountCalc(travelPlan[currentTravelSection][1]);
        runMotors(travelPlan[currentTravelSection][0],travelPlan[currentTravelSection][2],travelPlan[currentTravelSection][3],travelPlan[currentTravelSection][4]);
        motorRunning = true;
      }

      // 3. Motor Movement Control
      if (motorRunning && encoderCount_left >= currentTargetPulsCount) {
        stopMotors(); 
        currentTravelSection++; 

        if (currentTravelSection >= totalSectionsInTravelPlan) { 
          Serial.println("--- Travel Plan Completed and Stopped ---");
        } else {
          encoderCount_left = 0;
          encoderCount_right = 0;
          currentTargetPulsCount = targetPulsCountCalc(travelPlan[currentTravelSection][1]);
          runMotors(travelPlan[currentTravelSection][0],
                    travelPlan[currentTravelSection][2],
                    travelPlan[currentTravelSection][3],
                    travelPlan[currentTravelSection][4]);
          motorRunning = true;
        }
      
      }
  // --- Common LCD Update ---
  updateScreen();

  delay(20);
}

// ====================================================================
// --- FUNCTIONS ---
// ====================================================================

//Process Command Function to handle all movement in a resuable way , internally and externally

void processCommand(String message) {
  int pos_s = message.indexOf("Move:");
  int pos_r = message.indexOf("Turn:");
  int pos_l = message.indexOf("lcd:");
  int pos_n = message.indexOf("ToNorth");
  int pos_sp = message.indexOf("NewSpeed:");
  int pos_dir = message.indexOf("Dir:");
  int pos_rm = message.indexOf("Room");
  int pos_fl = message.indexOf("Follow");
  int pos_pt = message.indexOf("Path");

        

   if(pos_s > -1){
      LCDcommandText = message;
      Serial.println("Command = Dist");
      pos_s = message.indexOf(":");
      if(pos_s >-1){
        String sometexthere = message.substring(pos_s+1);
              
        if(sometexthere.toInt()>0){
          travelPlan[0][2]='f';
          travelPlan[0][1] = abs(sometexthere.toInt()) *10;
          travelPlan[0][3]=0;
        }else{
          travelPlan[0][1] = abs(sometexthere.toInt()) *10;
          travelPlan[0][2]='b';
          travelPlan[0][3]=0;
        }
        buttonPressedFlag = true;
      }
    }     

        if(pos_r > -1){
          LCDcommandText = message;
          Serial.println("Command = Turn");
          pos_r = message.indexOf(":");
            if(pos_r >-1){
              String sometexthere = message.substring(pos_r+1);
              
                travelPlan[0][3] = abs(sometexthere.toInt());

                if(sometexthere.toInt()<0){
                  travelPlan[0][4] = 'l';
                }else{
                  travelPlan[0][4] = 'r';
                }
                travelPlan[0][1]=0;
              buttonPressedFlag = true;
            }
        }

        if(pos_sp > -1){
          LCDcommandText = message;
          Serial.println("Command = NewSpeed");
          pos_r = message.indexOf(":");
            if(pos_r >-1){
              String sometexthere = message.substring(pos_r+1);
              travelPlan[0][0] = abs(sometexthere.toInt());
              travelPlan[0][1]=0;
              buttonPressedFlag = true;
            }
        }

        if(pos_n > -1){
          LCDcommandText = message;
          Serial.println("Command = ToNorth");
            
              int NorthDir = 256;
              char rotationDir;
              travelPlan[0][3] = getRotationAngle(NorthDir, rotationDir);
              travelPlan[0][4] = rotationDir; 
              travelPlan[0][1]=0;
              buttonPressedFlag = true;
        }

        if(pos_dir>-1){
          LCDcommandText = message;
          Serial.println("Command = Dir");
          pos_dir = message.indexOf(":");
            if(pos_dir>-1){
              String sometexthere = message.substring(pos_dir+1);
              int turnToDir = sometexthere.toInt();
              char rotationDir;
              travelPlan[0][3] = getRotationAngle(turnToDir, rotationDir);
              travelPlan[0][4] = rotationDir; 
              travelPlan[0][1]=0;
              buttonPressedFlag = true;
             
            }
        }

        if(pos_l > -1){
          LCDcommandText = message;
          Serial.println("Command = LCD");
        }

        if(pos_rm > -1){
          GetRoomMeasurements();
        
        }

        if(pos_fl > -1){
          stopMotors();
          if(followFlag==false){
            followFlag=true;
            Serial.println('Follow mode turned on!');
          }else{
            followFlag=false;
            Serial.println('Follow mode is off!');
          }
        }

        if(pos_pt > -1){
          stopMotors();
          if(pathFlag == false){
            Serial.println("Follow Path mode turned on!");
            encoderCount_left = 0;
            currentPathStep = 0;   // Reset to the beginning of your plan
            startNextPathStep();   // USE THE NEW FUNCTION HERE
          } else {
            pathFlag = false;
            Serial.println("Follow Path mode is off!");
          }
        }
      
      
  
}



// --- Joystick/Mapping Functions ---
int* joystickPercentages(int x, int y){
  int xDifferece = x-maxX[1];
  int yDifferece = y-maxY[1];

  static int joystickPercentageValues[2]; 

  if(xDifferece > 0){ 
    joystickPercentageValues[0] = (int)(100.0 * xDifferece / (maxX[2]-maxX[1]));
  }else{
    joystickPercentageValues[0] = (int)(100.0 * xDifferece / (maxX[1]-maxX[0]));
  }

  if(yDifferece > 0){
    joystickPercentageValues[1] = (int)(100.0 * yDifferece / (maxY[2]-maxY[1]));
  }else{
    joystickPercentageValues[1] = (int)(100.0 * yDifferece / (maxY[1]-maxY[0]));
  }
  
  return joystickPercentageValues;
}

int motorSpeed(int percentage){
  return abs(255*percentage/100);
}

int targetPulsCountCalc(int travelDistance){
  return (int)(travelDistance*distperpuls);
}

// --- ISR/Mode Functions ---
void joyPressed(){
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    lastInterruptTime = interruptTime;
    Serial.println("JoyPressed! Toggling mode.");
    toggleMode(); 
    buttonPressedFlag = true; 
  }
}

void toggleMode(){
    if(controlMode=="ESP"){
      controlMode="JOY";
    }else{
      controlMode="ESP";
    }
}

void countEncoder_left() {
  encoderCount_left++;
}

void countEncoder_right() {
  encoderCount_right++;
}


// --- Motor Control Functions (ESP Mode) ---
void runMotors(int speedPercent, char direction, int rotationAngle,char rotationDirection) {
  turnToBearing(targetBearingCal(rotationAngle,rotationDirection),rotationDirection); 
  encoderCount_left = 0;
  encoderCount_right = 0;

  if (direction == 'f') {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  } else {
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);
  }

  analogWrite(Motor_R_pwm_pin, motorSpeed(speedPercent));
  analogWrite(Motor_L_pwm_pin, motorSpeed(speedPercent));
}


void stopMotors(){
  analogWrite(Motor_R_pwm_pin, 0);
  analogWrite(Motor_L_pwm_pin, 0);
  motorRunning = false;
}

// --- Motor Control Function (JOY Mode) ---
void runMotors_joy(int xPer,int yPer) {
  motorRunning = false; 

  if(xPer > 0 && abs(yPer) < 40){ // Turn Left 
    digitalWrite(Motor_L_dir_pin, Motor_forward); 
    digitalWrite(Motor_R_dir_pin, Motor_return);
    analogWrite(Motor_L_pwm_pin, motorSpeed(xPer)); 
    analogWrite(Motor_R_pwm_pin, 0); 
  }
  else if(xPer < 0 && abs(yPer) < 40){ // Turn Right 
    digitalWrite(Motor_L_dir_pin, Motor_return); 
    digitalWrite(Motor_R_dir_pin, Motor_forward); 
    analogWrite(Motor_L_pwm_pin, 0); 
    analogWrite(Motor_R_pwm_pin, motorSpeed(abs(xPer)));
  }
  else if(abs(xPer) < 20 && yPer > 10){ // Go Forward
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);  
    analogWrite(Motor_L_pwm_pin, motorSpeed(yPer));
    analogWrite(Motor_R_pwm_pin, motorSpeed(yPer));
  }
  else if(abs(xPer) < 20 && yPer < -10){ // Go Backward
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);  
    analogWrite(Motor_L_pwm_pin, motorSpeed(abs(yPer)));
    analogWrite(Motor_R_pwm_pin, motorSpeed(abs(yPer)));
  }
  else { // Stop (Dead zone)
    analogWrite(Motor_R_pwm_pin, 0);
    analogWrite(Motor_L_pwm_pin, 0);
  }
}

// --- Bearing/Compass Functions ---
void turnToBearing(int targetBearing,char rotationAngle) {
  while (!isBearingCorrect(targetBearing)) {
    char dir = rotationAngle;

    if (dir == 'r') {
      digitalWrite(Motor_L_dir_pin, Motor_return);
      digitalWrite(Motor_R_dir_pin, Motor_forward);
    } else {
      digitalWrite(Motor_L_dir_pin, Motor_forward);
      digitalWrite(Motor_R_dir_pin, Motor_return);
    }

    analogWrite(Motor_R_pwm_pin, motorSpeed(tuningSpeed));
    analogWrite(Motor_L_pwm_pin, motorSpeed(tuningSpeed));

    bearingDegrees = readBearing16Bit(); 
    delay(50);
    updateScreen();
  }
  stopMotors(); 
}

int targetBearingCal(int rotationAngle,char rotationDirection){
  int targetBearing = int(bearingDegrees);
  if(rotationDirection=='r'){
    targetBearing += rotationAngle;
    if(targetBearing>360){
      targetBearing-=360;
    }
  }else if(rotationDirection=='l'){
    targetBearing -= rotationAngle;
    if(targetBearing<0){
      targetBearing+=360;
    }
  }
  return targetBearing;
}

bool isBearingCorrect(int targetBearing) {
  int current = int(bearingDegrees);
  int diff = targetBearing - current;

  // Normalize difference to -180..180
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  bool result = abs(diff) <= bearingMaxError; 
  return result;
}

int getRotationAngle(int targetBearing, char &rotationDirection) {
  int diff = targetBearing - bearingDegrees;

  // Normalize difference to range -180 .. 180
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  if (diff > 0) {
    rotationDirection = 'r';
  } else {
    rotationDirection = 'l';
  }

  return abs(diff);
}

float readBearing16Bit() {
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(2);
  Wire.endTransmission(); 

  if (Wire.requestFrom(CMPS14_ADDRESS, 2) == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int rawBearing = ((unsigned int)highByte << 8) | lowByte;
    return (float)rawBearing / 10.0;
  }
  return 0.0;
}

// --- Display Functions ---
// Return Direction according to bearing value
String getDirection(int bearing) {
  if (bearing < 23 || bearing >= 338) return "N";
  else if (bearing < 68) return "NE";
  else if (bearing < 113) return "E";
  else if (bearing < 158) return "SE";
  else if (bearing < 203) return "S";
  else if (bearing < 248) return "SW";
  else if (bearing < 293) return "W";
  else return "NW";
}

void updateScreen(){
  lcd.clear();
  
  // Convert pulse counts to distance in CM (Pulses * 1.3 pulses/mm * 0.1 mm/cm = Pulses * 0.13 cm/pulse)
  // Or: Distance (mm) = Pulses / 1.3 pulses/mm. Distance (cm) = Distance (mm) / 10.
  float dist_L_cm = (float)encoderCount_left / distperpuls / 10.0;
  float dist_R_cm = (float)encoderCount_right / distperpuls / 10.0;
  
  // --- ROW 0 ---
  // [ ] Mode: Displays the current mode (ESP/Joystick)
  lcd.setCursor(0, 0);
  lcd.print(controlMode); 
  lcd.setCursor(4, 0);
  
  if (controlMode == "ESP") {
    // [ ] ESP commands: Displays the latest command given from ESP (only in ESP mode)
    if(pathFlag==true){
      lcd.print("Dist:");
      lcd.print(encoderCount_left*distperpuls/10);
      lcd.print("cm");
    }else{
    lcd.print("CMD:");
    lcd.print(LCDcommandText);
    }
  } else { 
    // [ ] Joystick values: Displays analog values for both joystick axes (only in joystick mode)
    lcd.print("X:");
    lcd.print(joyX_val);
    lcd.print(" Y:");
    lcd.print(joyY_val);
  }



  
  // [ ] pulse counts and distances (cm’s) traveled for both wheels
  lcd.setCursor(0, 1);
  lcd.print(bearingDegrees,1);
  lcd.print((char)223); // Degree symbol
  lcd.print(getDirection(int(bearingDegrees))); // Direction (N, NE, E...)
  
  // Display distance traveled
  lcd.print("|L");
  lcd.print(dist_L_cm, 1); // Display L distance in CM
  lcd.print("|R");
  lcd.print(dist_R_cm, 1); // Display R distance in CM
}

// Lidar distance averager
float getAvgDistance(int sampleSize){
  float delaytime=millis();
  float DistanceTotal=0;
  if(sampleSize>0){
  for(int i=0;i< sampleSize;i++){
    DistanceTotal += myLIDAR.getDistance();
    delay(1);
  }
  delaytime=millis()-delaytime;
  //Serial.println(delaytime);
  return DistanceTotal/sampleSize;
  }else{
  Serial.println("Invalid sample size!");
  return NULL;
  }
}

float GetRoomMeasurements(){
  
  RoomData[0]=getAvgDistance(10);
  turnToBearing(targetBearingCal(90,'l'),'l');
  RoomData[1]=getAvgDistance(10);
  turnToBearing(targetBearingCal(90,'l'),'l');
  RoomData[2]=getAvgDistance(10);
  turnToBearing(targetBearingCal(90,'l'),'l');
  RoomData[3]=getAvgDistance(10);
  turnToBearing(targetBearingCal(90,'l'),'l');

  
  float lengthX = RoomData[0]+RoomData[2];
  Serial.print("Length X:");
  Serial.print(lengthX );
  Serial.println("cm");

  float lengthY = RoomData[1]+RoomData[3];
  Serial.print("Length Y:");
  Serial.print(lengthY );
  Serial.println("cm");

  float roomArea = lengthX*lengthY;
  RoomData[4]=roomArea;
  Serial.print("Area of room floor:");
  Serial.print(RoomData[4]);
  Serial.println("cm^2");
  

  float roomVolume = lengthX*lengthY*100;
  RoomData[5]=roomVolume;
  Serial.print("Volume of room :");
  Serial.print(RoomData[5]);
  Serial.println("cm^3");

}

float handleFollowPath(){
  turnToBearing(targetBearingCal(followPlan[currentPathStep][0],followPlan[currentPathStep][3]),followPlan[currentPathStep][3]);
  LockedbearingDegrees=bearingDegrees;
  GapValue=followPlan[currentPathStep][2];
  pathFlag=true;
}

void startNextPathStep() {
  // 1. Calculate the target bearing ONCE based on where we were
  int turnAmount = followPlan[currentPathStep][0];
  char turnDir = followPlan[currentPathStep][3];
  
  // Calculate the absolute target heading we WANT to reach
  int targetHeading = targetBearingCal(turnAmount, turnDir);
  
  // 2. Perform the physical turn
  turnToBearing(targetHeading, turnDir);
  
  // 3. LOCK this absolute heading for the duration of this step
  LockedbearingDegrees = targetHeading; 
  GapValue = followPlan[currentPathStep][2];
  
  pathFlag = true;
  Serial.print("Step Initialized. Target Heading: ");
  Serial.println(LockedbearingDegrees);
}
 