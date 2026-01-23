#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLIDAR; //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun
int newDistance;
int DistanceSampleSize=10;
//for Room measuremnts
float roomMeasurements[4];

void setup() {
  Serial.begin(115200);
  Wire.begin(); //Join I2C bus
  //Wire.setClock(400000);
  //check if LIDAR will acknowledge over I2C
  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("LIDAR acknowledged!");
}

void loop() {
  
 // delay(495);//Don't hammer too hard on the I2C bus
  Serial.print("Avg distance: ");
  Serial.print(getAvgDistance(7));
  Serial.println("cm");

    
}

float getAvgDistance(int sampleSize){
  float delaytime=millis();
  float DistanceTotal=0;
  if(sampleSize>0){
  for(int i=0;i< sampleSize;i++){
    DistanceTotal += myLIDAR.getDistance();
    delay(1);
  }
  delaytime=millis()-delaytime;
  Serial.println(delaytime);
  return DistanceTotal/sampleSize;
  }else{
  Serial.println("Invalid sample size!");
  return NULL;
  }
}

