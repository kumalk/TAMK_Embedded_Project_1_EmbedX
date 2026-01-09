#include <LiquidCrystal.h>

#define ANALOG_X_PIN A8
#define ANALOG_Y_PIN A9 
#define ANALOG_BUTTON_PIN 19 

#define ANALOG_X_CORRECTION 130 
#define ANALOG_Y_CORRECTION 123 

int pressCount = 0;
volatile unsigned long lastInterruptTime = 0;

struct Button {
  bool pressed = false;
};

struct Joystick {
  int x = 0;
  int y = 0;
  Button button;
};

int maxX[] = {-130,125};
int maxY[] = {-123,132};

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);
  
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), joyPressed, FALLING);
  Serial.begin(9600);
}

void loop() {
  Joystick joystick;
  
  joystick.x = readAnalogAxisLevel(ANALOG_X_PIN) - ANALOG_X_CORRECTION;
  joystick.y = readAnalogAxisLevel(ANALOG_Y_PIN) - ANALOG_Y_CORRECTION;
  joystick.button.pressed = isAnalogButtonPressed(ANALOG_BUTTON_PIN);
  int* joyPercents = joystickPercentages(joystick.x, joystick.y);

  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("x:");
  lcd.print(joystick.x);
  lcd.setCursor(16, 0);
  lcd.print(joyPercents[0]);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  lcd.print("y:");
  lcd.print(joystick.y);
  lcd.setCursor(16, 1);
  lcd.print(joyPercents[1]);
  lcd.print("%");

  // Debug output
  Serial.print("X: ");
  Serial.println(joystick.x);

  Serial.print("Y: ");
  Serial.println(joystick.y);

  if (joystick.button.pressed) {
    Serial.println("Button pressed");
  } else {
    Serial.println("Button not pressed");
  }

  delay(800);
}

int readAnalogAxisLevel(int pin) {
  return map(analogRead(pin), 0, 1023, 0, 255);
}

bool isAnalogButtonPressed(int pin) {
  return digitalRead(pin) == LOW;
}

int* joystickPercentages(int x, int y){
  
  static int joystickPercentageValues[2]; // must be static to return pointer

  if(x>0){
    joystickPercentageValues[0]=100*x/maxX[1];
  }else{
    joystickPercentageValues[0]=100*(-x)/(-maxX[0]); // use absolute
  }

  if(y>0){
    joystickPercentageValues[1]=100*y/maxY[1];
  }else{
    joystickPercentageValues[1]=100*(-y)/(-maxY[0]); // use absolute
  }

  // clamp to 100 just in case
  if(joystickPercentageValues[0]>100) joystickPercentageValues[0]=100;
  if(joystickPercentageValues[1]>100) joystickPercentageValues[1]=100;

  return joystickPercentageValues;
}

void joyPressed(){
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    pressCount++;
    Serial.println(pressCount);
    lastInterruptTime = interruptTime;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Push Counter:");
    lcd.setCursor(0, 1);
    lcd.print(pressCount);

  }
}