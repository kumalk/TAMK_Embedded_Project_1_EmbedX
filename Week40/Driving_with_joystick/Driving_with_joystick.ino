#include <LiquidCrystal.h>

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


int pressCount = 0;// this is to display press count on the screen
volatile unsigned long lastInterruptTime = 0; // this is to elemenate boutton press debounce

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

// LCD
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32; // definiing LCD screen pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Creating LiquidCrystal object from imported class from the library at the top,  by passing LCD screen pins.
 
void setup() {
  lcd.begin(16, 2); // starting lcd screeen
  
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);  //adding input pullup to JoystickButtonPress pin
  attachInterrupt(digitalPinToInterrupt(ANALOG_BUTTON_PIN), joyPressed, FALLING); //ISR adding at falling edge so ISR function joyPressed function called after press completes.

  pinMode(Motor_L_dir_pin, OUTPUT); // Left motor direction pin as output
  pinMode(Motor_R_dir_pin, OUTPUT); // Right motor direction pin as output
  pinMode(Motor_L_pwm_pin, OUTPUT); // Left motor power(speed) pin as output
  pinMode(Motor_R_pwm_pin, OUTPUT); // Right motor power(speed) pin as output

  Serial.begin(9600);// starting serial monitor
}

void loop() {
  // Check button press flag
  if (buttonPressedFlag) { //
    buttonPressedFlag = false;  // reset flag to identify if button is pressed, becuase there was an issue in updating display inside ISR function, so we update lcd in default loop by identifying the flag at the very begininig.
    
  }

  Joystick joystick; // creating joyStick object 
  
  joystick.x = analogRead(ANALOG_X_PIN); // reading analog pin to get X value of the joy stick
  joystick.y = analogRead(ANALOG_Y_PIN); // reading analog pin to get Y value of the joy stick
  int* joyPercents = joystickPercentages(joystick.x, joystick.y); // we create joystick direction values as percentages via joystickPercentages() , this Array is to call it and store the returning values

  
 

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

  runMotors(joyPercents[0], joyPercents[1]);


  // Debug output in Serial monitor
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

// ISR to fire when joystick button presses
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



void runMotors(int xPer,int yPer) {

  if(xPer > 0 && abs(yPer) < 40){ //turn left, Right motor(our left motor) only
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_return);  //setting Left motor direction
    analogWrite(Motor_L_pwm_pin, motorSpeed(xPer));
    //analogWrite(Motor_R_pwm_pin, motorSpeed(xPer));
    delay(10); 

  }else if(xPer < 0 && abs(yPer) < 40){ //turn right, left motor(our rigth motor) only
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_forward);  //setting Left motor direction
    //analogWrite(Motor_L_pwm_pin, motorSpeed(xPer));
    analogWrite(Motor_R_pwm_pin, motorSpeed(xPer));
    delay(10);

  }else if(xPer==0 && yPer>0){ //go farward, both motors farward
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);  //setting Left motor direction
    analogWrite(Motor_L_pwm_pin, motorSpeed(yPer));
    analogWrite(Motor_R_pwm_pin, motorSpeed(yPer));
    delay(10);
  }else if(xPer==0 && yPer<0){ //go backward, both motors backward
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);  //setting Left motor direction
    analogWrite(Motor_L_pwm_pin, motorSpeed(yPer));
    analogWrite(Motor_R_pwm_pin, motorSpeed(yPer));
    delay(10);
  }else{//stop motor both val 0
    analogWrite(Motor_R_pwm_pin, 0);
    analogWrite(Motor_L_pwm_pin, 0);
  }

  
}


/*void runMotors() {
  

  // Motor backward
  digitalWrite(Motor_R_dir_pin, Motor_return);  //setting Right motor direction
  digitalWrite(Motor_L_dir_pin, Motor_return);  //setting Left motor direction
  analogWrite(Motor_R_pwm_pin, 100);   //setting Right motor speed
  analogWrite(Motor_L_pwm_pin, 100);   //setting Left motor speed
  delay(2000);//stop doing any changes for 2secons , in this time motor receives current in above set config

  // Motor forward
  digitalWrite(Motor_R_dir_pin, Motor_forward);  
  digitalWrite(Motor_L_dir_pin, Motor_forward);  
  analogWrite(Motor_R_pwm_pin, 100);
  analogWrite(Motor_L_pwm_pin, 100);
  delay(2000);

  // Stop motors
  analogWrite(Motor_R_pwm_pin, 0);
  analogWrite(Motor_L_pwm_pin, 0);

  
}*/
