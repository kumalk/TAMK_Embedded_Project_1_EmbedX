// ====================================================================
// --- Navigation and sensor handling Functions ---
// ====================================================================


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
  return -1.0f;
  }
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
