// ====================================================================
// --- UI methods handlig fuctions ---
// ====================================================================


//Process Command Function to handle all movement in a resuable way , internally and externally

void processCommand(String message) {
  int pos_s = message.indexOf("Move:"); //To staight drive forward or backward 
  int pos_r = message.indexOf("Turn:"); //Turn by spesific angle
  int pos_l = message.indexOf("lcd:"); //Just LCD command to printD
  int pos_n = message.indexOf("ToNorth"); // Turn to north
  int pos_sp = message.indexOf("NewSpeed:"); // Set new default speed as percerntage 
  int pos_dir = message.indexOf("Dir:"); //turn to a spesific riection
  int pos_rm = message.indexOf("Room"); // Canlculating room volume by LIDAR
  int pos_fl = message.indexOf("Follow"); // Follow an object maintaining spesific hardcoded distance 
  int pos_pt = message.indexOf("Path"); // Go around a box by maintianing custom hardcoded distances from the walls 
  int pos_cl = message.indexOf("NCal"); //For North direction calibration
  int pos_calibrate = message.indexOf("DPCal"); // Calibrate distance per puls value by LDR data and save in EEPROM

        

   if(pos_s > -1){   //To staight drive forward or backward 
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

        if(pos_r > -1){   //Turn by spesific angle
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

        if(pos_sp > -1){ // Set new default speed as percerntage 
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

        if(pos_n > -1){ // Turn to north
          LCDcommandText = message;
          Serial.println("Command = ToNorth");
            
              char rotationDir;
              travelPlan[0][3] = getRotationAngle(NorthDir, rotationDir);
              travelPlan[0][4] = rotationDir; 
              travelPlan[0][1]=0;
              buttonPressedFlag = true;
        }

        if(pos_dir>-1){ //turn to a spesific riection
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

        if(pos_l > -1){ //Just LCD command to print
          LCDcommandText = message;
          Serial.println("Command = LCD");
        }

        if(pos_rm > -1){  // Canlculating room volume by LIDAR
          GetRoomMeasurements();
        
        }

        if(pos_fl > -1){ // Follow an object maintaining spesific hardcoded distance
          stopMotors();
          if(followFlag==false){
            followFlag=true;
            Serial.println('Follow mode turned on!');
          }else{
            followFlag=false;
            Serial.println('Follow mode is off!');
          }
        }

        if(pos_pt > -1){ // Go around a box by maintianing custom hardcoded distances from the walls 
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

        if(pos_cl > -1){
          LCDcommandText = message;
          Serial.println("Command = NCal");
          NorthDir = bearingDegrees;

          
        }

        if(pos_calibrate > -1){
          Serial.println("Starting Calibration Routine...");
          Serial.println("Command = DPCal");
          calibrateEncoders();
        }
      
      
  
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

//Toggle between modes ESP and JOY
void toggleMode(){
    if(controlMode=="ESP"){
      controlMode="JOY";
    }else{
      controlMode="ESP";
    }
}


//Handling LCD Screen
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
