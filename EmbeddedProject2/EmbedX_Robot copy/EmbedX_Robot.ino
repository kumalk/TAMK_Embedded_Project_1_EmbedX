//---------------------------------------
// Tampere Universisty of Appied Science
// Team : EmbedX
// Final Aruduino Code 
// Date : 12/02/2026 
// Team Members :
//    Prashantha Kumanayake
//    Lasanthi Ayesha
//    Nimeshika Rodrigo 
//---------------------------------------

#include <LiquidCrystal.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include <EEPROM.h> 

// --- EEPROM Address ---
const int DIST_PER_PULSE_ADDR = 0;

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

int NorthDir = 0; //hardcoded defualt to use until calibrate
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
  Serial2.begin(115200);
  
  // Load calibrated distperpuls from EEPROM
  float storedDist;
  EEPROM.get(DIST_PER_PULSE_ADDR, storedDist);
  
  // Check if a valid value exists (EEPROM defaults to NaN or 0.0 if never written)
  if (!isnan(storedDist) && storedDist > 0.1) {
    distperpuls = storedDist;
    Serial.print("Loaded Calibration from EEPROM: ");
    Serial.println(distperpuls);
  } else {
    Serial.println("No valid calibration in EEPROM. Using default 1.2");
  }


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
      if (Serial2.available() > 0 ){
        String message = Serial2.readStringUntil('\n');
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

  // --- Send Lidar Data to ESP ---
  static unsigned long lastESPUpdate = 0;
  if (millis() - lastESPUpdate > 500) { 
    float dist = getAvgDistance(3);
    
    // This matches your ESP's "if (data.startsWith("LIDAR:"))"
    // Inside Arduino Loop
  Serial2.println("LIDAR:" + String(dist));       // Use println for clear separation
  Serial2.println("Compass:" + String(bearingDegrees));
    
    lastESPUpdate = millis();
  }
  delay(20);
}

