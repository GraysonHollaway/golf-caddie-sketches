#include <SoftwareSerial.h>
#include <string.h>
SoftwareSerial mySerial(8,9);
void setup() {
  // put your setup code here, to run once:
 mySerial.begin(38400);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  String lon = "";
  if(mySerial.available()){
    delay(30);
    while(mySerial.available()){
    char inByte = mySerial.read();
    mySerial.write(inByte);
    lon += inByte;
    
    }
    Serial.println(lon);
  }
  
  
  
}
