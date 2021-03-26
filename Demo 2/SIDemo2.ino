#include <Wire.h>
#include <Encoder.h>

#define SLAVE_ADDRESS 0x04

byte angleRec[8];
byte data[32];
String myString;

void setup() {
  Serial.begin(250000);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
}

void loop() {
 

//  myString = String(angleRec,2);
//  for(int j = 0; j < myString.length(); j++){
//    data[j] = myString[j];
//  }



}

void receiveData(int byteCount) {
  int i = -1;
while (Wire.available()) {

angleRec[i] = Wire.read();
i++;
}
}

void sendData(){  
Wire.write(angleRec, 5);
  
}
