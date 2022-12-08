#include <SoftwareSerial.h>
#include <string.h>
SoftwareSerial mySerial(8,9);
void setup() {
  // put your setup code here, to run once:
 Serial2.begin(38400);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  String lon = "";
  if(Serial2.available()){
    delay(30);
    while(Serial2.available()){
    char inByte = Serial2.read();
    Serial2.write(inByte);
    lon += inByte;
    
    }
    Serial.println(lon);
  }
}
