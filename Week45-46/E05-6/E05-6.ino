#include <LiquidCrystal.h>
#include <Wire.h>

// Motor pins
#define Motor_forward   1
#define Motor_return    0
#define Motor_L_dir_pin 8
#define Motor_R_dir_pin 7
#define Motor_L_pwm_pin 10
#define Motor_R_pwm_pin 9

// Joystick pins
#define ANALOG_X_PIN A8
#define ANALOG_Y_PIN A9 
#define ANALOG_BUTTON_PIN 19 

// Define I2C address for the CMPS14 compass module
#define CMPS14_ADDRESS 0x60

int pressCount = 0;// this is to display press count on the screen
volatile unsigned long lastInterruptTime = 0; // this is to elemenate boutton press debounce
volatile long encoderCount_left = 0;
volatile long encoderCount_right = 0;
float distperpuls = 1.3;
int travelDist = 100;// travelling distance in mm
int travelPlan[1][5] = {
  {75,0,'f',0,'l'}
  //{75,200,'f',0,'l'},{75,200,'f',90,'l'},{75,200,'f',90,'l'},{75,200,'f',90,'l'}
  //{speedPercentage,distanceInMM,'forward/backward',turningAngle,turningDirection},{25,50,'f',170},{75,100,'f',290}
};
int currentTargetPulsCount = 0;  
int currentTravelSection = 0;
int totalSectionsInTravelPlan = sizeof(travelPlan) / sizeof(travelPlan[0]); 
String LCDcommandText = "";


bool motorRunning = false;


struct Button { // we created this button stucture just like an class to handle button press later
  bool pressed = false;
};

struct Joystick { // Joystick object(structure) to handle joystic direction and button press
  int x = 0;
  int y = 0;
  Button button; // creating a new button object from the Button class.
};

int maxX[] = {0,522,1023}; // These are the left maximum , middle and right maximum analog values of the joystic direction in X axis
int maxY[] = {0,494,1023}; // These are the left maximum , middle and right maximum analog values of the joystic direction in y axis
volatile bool buttonPressedFlag = false;  // ISR sets this flag to identify button pressed , so loop can uodate the display
volatile float bearingDegrees = 0;
// LCD
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32; // definiing LCD screen pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Creating LiquidCrystal object from imported class from the library at the top,  by passing LCD screen pins.
 
void setup() {
  lcd.begin(16, 2); // starting lcd screeen
  Wire.begin(); // Start I2C communication

  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);  //adding input pullup to JoystickButtonPress pin
  attachInterrupt(digitalPinToInterrupt(ANALOG_BUTTON_PIN), joyPressed, FALLING); //ISR adding at falling edge so ISR function joyPressed function called after press completes.

  pinMode(Motor_L_dir_pin, OUTPUT); // Left motor direction pin as output
  pinMode(Motor_R_dir_pin, OUTPUT); // Right motor direction pin as output
  pinMode(Motor_L_pwm_pin, OUTPUT); // Left motor power(speed) pin as output
  pinMode(Motor_R_pwm_pin, OUTPUT); // Right motor power(speed) pin as output

  pinMode(2, INPUT_PULLUP);//encoder pin initialize
  attachInterrupt(digitalPinToInterrupt(2), countEncoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countEncoder_right, RISING);
  Serial.begin(9600);// starting serial monitor
}

void loop() {
  if (Serial.available() > 0){
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);
    int pos_s = message.indexOf("Move:");
    int pos_r = message.indexOf("degree:");
    int pos_l = message.indexOf("lcd:");
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
      Serial.println("Command = Degree");
      LCDcommandText = message;
      pos_r = message.indexOf(":");
        if(pos_r >-1){
          String sometexthere = message.substring(pos_r+1);
          
            char rotationDir;
            travelPlan[0][3] = getRotationAngle(sometexthere.toInt(), rotationDir);
            travelPlan[0][4] = rotationDir; // store direction in travelPlan
            travelPlan[0][1]=0;
          buttonPressedFlag = true;
        }
    }

    if(pos_l > -1){
      Serial.println("Command = LCD");
      LCDcommandText = message;
      pos_l = message.indexOf(":");
        if(pos_l >-1){
          String sometexthere = message.substring(pos_l+1);
          
          travelPlan[0][1]=0;
          travelPlan[0][3]=0;
          //buttonPressedFlag = true;
        }
    }
  }
  if (buttonPressedFlag) {
    buttonPressedFlag = false;
    currentTravelSection = 0;
    encoderCount_left = 0;
    encoderCount_right = 0;
    currentTargetPulsCount = targetPulsCountCalc(travelPlan[currentTravelSection][1]);
    runMotors(travelPlan[currentTravelSection][0],travelPlan[currentTravelSection][2],travelPlan[currentTravelSection][3],travelPlan[currentTravelSection][4]);
    motorRunning = true;
  }

  // --- Motor Movement Control ---
  if (motorRunning && encoderCount_left >= currentTargetPulsCount) {
    
    // Stop the motor immediately after meeting the pulse count goal
    stopMotors(); 

    currentTravelSection++; // Advance to the next section index

    // Check if the plan is complete (Total sections is 3, indices are 0, 1, 2)
    // If the index is 3 or more, the plan is finished.
    if (currentTravelSection >= totalSectionsInTravelPlan) { 
      
      Serial.println("--- Travel Plan Completed and Stopped ---");
    } else {
      // If there are more sections, start the next segment
      encoderCount_left = 0;
      encoderCount_right = 0;
      
      // Load the new target pulse count
      currentTargetPulsCount = targetPulsCountCalc(travelPlan[currentTravelSection][1]);
      
      // Start the motors for the new section
      runMotors(travelPlan[currentTravelSection][0],
                travelPlan[currentTravelSection][2],
                travelPlan[currentTravelSection][3],
                travelPlan[currentTravelSection][4]);
      
      motorRunning = true; // runMotors() doesn't set this, so we set it here.
    }
  }
  
  // --- Non-Motor Tasks ---
  bearingDegrees = readBearing16Bit();

  updateScreen();

  delay(20);
}




// -------- Functions --------

int readAnalogAxisLevel(int pin) {
  return map(analogRead(pin), 0, 1023, 0, 255);
}

bool isAnalogButtonPressed(int pin) {
  return digitalRead(pin) == LOW;
}

int* joystickPercentages(int x, int y){
  int xDifferece = x-maxX[1];// checking difference from middle(1 element is the middle of maxX array)
  int yDifferece = y-maxY[1];// checking difference from middle(1 element is the middle of maxY array)


  static int joystickPercentageValues[2]; // must be static to return pointer 


  //calculating percentages comparing current x value and max values
  if(xDifferece > 0){ 
    joystickPercentageValues[0] = (int)(100.0 * xDifferece / (maxX[2]-maxX[1]));
  }else{
    joystickPercentageValues[0] = (int)(100.0 * xDifferece / (maxX[1]-maxX[0]));
  }

 //calculating percentages comparing current y value and max values
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

// ISR to fire when joystick button presses
// joystick button press flag UP by elinating debounce logically by comaring time
void joyPressed(){
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    lastInterruptTime = interruptTime;
    buttonPressedFlag = true;
  }
}


// Run motors to disired direction at desired speed (forward =f , else backward)
void runMotors(int speedPercent, char direction, int rotationAngle,char rotationDirection) {
  turnToBearing(targetBearingCal(rotationAngle,rotationDirection),rotationDirection); // make sure car faces correct direction first
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


// Turn to any spesific direction
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

    analogWrite(Motor_R_pwm_pin, motorSpeed(20));
    analogWrite(Motor_L_pwm_pin, motorSpeed(20));

    bearingDegrees = readBearing16Bit(); // keep updating
    delay(50);
    updateScreen();
  }

  stopMotors(); // stop when done
}



int targetBearingCal(int rotationAngle,char rotationDirection){
  int targetBearing = 0;
  int currentBearing = int(bearingDegrees);

    if(rotationDirection=='r'){
      targetBearing = currentBearing + rotationAngle;
      if(targetBearing>360){
        targetBearing-=360;
      }
    }else if(rotationDirection=='l'){
      targetBearing = currentBearing - rotationAngle;
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

  bool result = abs(diff) <= 4; // 4° tolerance
 
  return result;
}




// function to stop motors
void stopMotors(){
  analogWrite(Motor_R_pwm_pin, 0);
  analogWrite(Motor_L_pwm_pin, 0);
  motorRunning = false;
  
}

void countEncoder_left() {
  encoderCount_left++;
  //Serial.println(encoderCount_left); // ISR increments count
}

void countEncoder_right() {
  encoderCount_right++;
  //Serial.println(encoderCount_right); // ISR increments count
}


// --- Function to read and process the 16-bit bearing ---
float readBearing16Bit() {
  
  // 1. Set the starting register to 2 (16-bit High Byte)
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(2);
  Wire.endTransmission(); // Stop and release the bus

  // 2. Request 2 bytes (Register 2, then Register 3)
  if (Wire.requestFrom(CMPS14_ADDRESS, 2) == 2) {
    
    // Read the bytes in the order they are received (High then Low)
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    
    // Combine the two bytes into a 16-bit integer (raw value 0-3599)
    // We explicitly cast to int/unsigned int for proper bit shifting
    int rawBearing = ((unsigned int)highByte << 8) | lowByte;
    
    // Convert the raw value (tenths of a degree) to a float in degrees
    return (float)rawBearing / 10.0;
  }
  
  // Return 0.0 on read failure
  return 0.0;
}


void updateScreen(){
// Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bearing :");
  lcd.print(bearingDegrees,1);

  // lcd.setCursor(0, 4);
  // lcd.print("Direction :");
  // lcd.println(getDirection(int(bearingDegrees)));

  lcd.setCursor(0, 1);
  lcd.print("Command :");
  lcd.print(LCDcommandText);

}
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

int rotationAngleCalc(int heading){

}


// Function to calculate rotation angle and direction
// Arguments: current bearing, target bearing
// Returns: rotation angle (0–180)
// Output parameter: rotation direction ('l' or 'r')
int getRotationAngle(int targetBearing, char &rotationDirection) {
  int diff = targetBearing - bearingDegrees;

  // Normalize difference to range -180 .. 180
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  // Determine rotation direction
  if (diff > 0) {
    rotationDirection = 'r';  // Need to turn right
  } else {
    rotationDirection = 'l';  // Need to turn left
  }

  // Return absolute angle difference
  return abs(diff);
}


