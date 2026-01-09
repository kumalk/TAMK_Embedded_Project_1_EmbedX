


int valuePin = A3; 
int ledPin =3;
int val = 0;
float voltVal = 0.00;


void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  val = analogRead(valuePin);
  voltVal = 5.0 * val / 1023.0;
  Serial.println(val);
  Serial.println(voltVal);          
  
  if(voltVal<1.0){
    digitalWrite(ledPin,1);
  }else{
    digitalWrite(ledPin,0);
  }
  delay(1000);
}