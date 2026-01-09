#include <LiquidCrystal.h>

// LCD
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32; // definiing LCD screen pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Creating LiquidCrystal object from imported class from the library at the top,  by passing LCD screen pins.

void setup(){
  Serial.begin(9600);
  Serial.println("Write something to the serial monitor.");
  lcd.begin(16, 2);
}

void loop(){
  if (Serial.available() > 0){
    String message = Serial.readStringUntil('\n'); 
    Serial.print("Message received, content: ");  
    Serial.println(message);
    int pos_s = message.indexOf("Print");
    int pos_lcd = message.indexOf("LCD");

    if(pos_lcd > -1){
      Serial.println("Command = LCD");
      pos_lcd = message.indexOf(":");
        if(pos_lcd >-1){
          String sometexthere = message.substring(pos_lcd+1);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(sometexthere);
          delay(2000);
        }
    }
    if (pos_s > -1){
      Serial.println("Command = Print ");
      pos_s = message.indexOf(":");

      if (pos_s > -1){
        String stat = message.substring(pos_s + 1);
        if (stat == "Hi" || stat == "hi") {
          Serial.println("Hi!");
        }
        else if (stat == "Hello") {
          Serial.println("Hello there!");
        }
      }  
    }
    else{
	    Serial.println("No greeting found, try typing Print:Hi or Print:Hello\n");
    }
  }
}