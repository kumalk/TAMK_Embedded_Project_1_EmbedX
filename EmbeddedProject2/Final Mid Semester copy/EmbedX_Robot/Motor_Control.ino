// ====================================================================
// --- Motor Control Functions ---
// ====================================================================

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

void toNorth(){
  char rotationDir;
  travelPlan[0][3] = getRotationAngle(NorthDir, rotationDir);
  travelPlan[0][4] = rotationDir; 
  travelPlan[0][1]=0;
  buttonPressedFlag = true;
}


void calibrateEncoders() {
  stopMotors();
  delay(1000);
  
  float startDist = getAvgDistance(5);
  float targetDist = startDist - 20.0; // Moving 20cm forward (distance to wall decreases)
  
  encoderCount_left = 0;
  encoderCount_right = 0;
  
  Serial.println("Driving 20cm for calibration...");

  // Drive forward until LiDAR shows we moved 20cm
  while (getAvgDistance(2) > targetDist) {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
    analogWrite(Motor_L_pwm_pin, motorSpeed(25)); // Steady slow speed
    analogWrite(Motor_R_pwm_pin, motorSpeed(25));
    
    // Safety check: if distance is too small, stop
    if (getAvgDistance(1) < 10) break; 
  }
  
  stopMotors();
  delay(500);
  
  float actualDistMoved = (startDist - getAvgDistance(10)) * 10.0; // convert to mm
  long totalPulses = encoderCount_left; // Use left encoder as reference
  
  if (totalPulses > 0) {
    // Calibration formula: pulses / mm
    distperpuls = (float)totalPulses / actualDistMoved;
    
    // Save to EEPROM
    EEPROM.put(DIST_PER_PULSE_ADDR, distperpuls);
    
    Serial.print("Calibration Complete!");
    Serial.print(" Pulses: "); Serial.print(totalPulses);
    Serial.print(" | Dist: "); Serial.print(actualDistMoved); Serial.println("mm");
    Serial.print("New distperpuls saved: "); Serial.println(distperpuls);
  } else {
    Serial.println("Calibration Failed: No pulses detected.");
  }
}