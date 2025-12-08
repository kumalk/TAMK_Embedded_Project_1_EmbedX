/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <LittleFS.h>  // Use LittleFS instead of SPIFFS

void setup() {
  Serial.begin(115200);

  // Mount LittleFS
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  // Open file for reading
  File file = LittleFS.open("/hello.txt", "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println();
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void loop() {
  // Nothing needed here
}
