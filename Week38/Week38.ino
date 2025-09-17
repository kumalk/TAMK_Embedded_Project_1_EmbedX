#include <LiquidCrystal.h>

// Motor pins
#define Motor_forward   0
#define Motor_return    1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

// Joystick pins
#define ANALOG_X_PIN A8
#define ANALOG_Y_PIN A9 
#define ANALOG_BUTTON_PIN 19 


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

int maxX[] = {0,522,1023};
int maxY[] = {0,494,1023};
volatile bool buttonPressedFlag = false;  // ISR sets this

// LCD
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);
  
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANALOG_BUTTON_PIN), joyPressed, FALLING);

  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Check button press flag
  if (buttonPressedFlag) {
    buttonPressedFlag = false;  // reset flag
    runMotors();
  }

  Joystick joystick;
  
  joystick.x = analogRead(ANALOG_X_PIN);
  joystick.y = analogRead(ANALOG_Y_PIN);
  int* joyPercents = joystickPercentages(joystick.x, joystick.y);

 

  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("x:");
  lcd.print(joystick.x);
  lcd.setCursor(15, 0);
  lcd.print(joyPercents[0]);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  lcd.print("y:");
  lcd.print(joystick.y);
  lcd.setCursor(15, 1);
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

  delay(200);
}

// -------- Functions --------

int readAnalogAxisLevel(int pin) {
  return map(analogRead(pin), 0, 1023, 0, 255);
}

bool isAnalogButtonPressed(int pin) {
  return digitalRead(pin) == LOW;
}

int* joystickPercentages(int x, int y){
  int xDifferece = x-maxX[1];
  int yDifferece = y-maxY[1];


  static int joystickPercentageValues[2]; // must be static to return pointer

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

// ISR
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
    buttonPressedFlag = true;
  }
}




void runMotors() {
  

  // Motor backward
  digitalWrite(Motor_R_dir_pin, Motor_return);  
  digitalWrite(Motor_L_dir_pin, Motor_return);  
  analogWrite(Motor_R_pwm_pin, 100);   
  analogWrite(Motor_L_pwm_pin, 100);   
  delay(2000);

  // Motor forward
  digitalWrite(Motor_R_dir_pin, Motor_forward);  
  digitalWrite(Motor_L_dir_pin, Motor_forward);  
  analogWrite(Motor_R_pwm_pin, 100);
  analogWrite(Motor_L_pwm_pin, 100);
  delay(2000);

  // Stop motors
  analogWrite(Motor_R_pwm_pin, 0);
  analogWrite(Motor_L_pwm_pin, 0);

  
}
