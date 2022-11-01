//initialize wheels

#define wheel0 4 // Initialize pin 4 to drive a wheel
#define wheel1 5 
#define wheel2 6 
#define wheel3 7 

//initialize GPS
#include <Adafruit_GPS.h>
#define GPSECHO  true
Adafruit_GPS GPS(&Serial2);
double carGpsData [3] = {0,0,0};
double userGpsData [2] = {0,0};

//Bluetooth = Serial0
//GPS = Serial2 
//Motion Sensor
//

int state = 0; // To read state of serial data from HC05 module



void stopWheel(){
  digitalWrite(wheel0, LOW); // Turn wheels off
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, LOW);
}

void forwardWheel(){
  digitalWrite(wheel0, HIGH); // Turn wheels all on
  digitalWrite(wheel1, HIGH);
  digitalWrite(wheel2, HIGH);
  digitalWrite(wheel3, HIGH);
}

void leftTurnWheel(){
  digitalWrite(wheel0, LOW); // Turn left wheels back, right wheels forward
  digitalWrite(wheel1, HIGH);
  digitalWrite(wheel2, HIGH);
  digitalWrite(wheel3, LOW);
}

void rightTurnWheel(){
  digitalWrite(wheel0, HIGH); // Turn left wheels forward, right wheeels back
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, HIGH);
}

void oopsiesWheel() {
  digitalWrite(wheel0, LOW); // backwards
  digitalWrite(wheel1, HIGH);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, HIGH);
}

void checkStateWheel(int instr){
  if (instr == '0') {
    stopWheel();
    Serial.println("STOP"); 
  }
  else if (instr == '1') {
    forwardWheel();
    Serial.println("Forward ");
  }
  else if (instr == '2') {
    leftTurnWheel();
    Serial.println("Left");
  }
  else if (instr == '3') {
    rightTurnWheel();
    Serial.println("Right Turn");
  }
  else if (instr == '4') {
    oopsiesWheel();
    Serial.println("backward");
  }
  instr = 0;
}

void updateUserCoords(){
  if(userGPS.fix){
    double carLong = userGPS.longitudeDegrees;
    double carLat = userGPS.latitudeDegrees;
    double carAngle = userGPS.angle;
  } else{
    //TODO: throw an error or smthng idk
  }
  carGpsData[0] = GPS.latitude;
  carGpsData[1] = GPS.longitude;
  carGpsData[2] = GPS.angle;
}

void updateCarCoords(){
  if(GPS.fix){
    double usLong = GPS.longitudeDegrees;
    double usLat = GPS.latitudeDegrees;
    double usAngle = GPS.angle;
  } else{
    //TODO: throw an error or smthng idk
  }
  userGpsData[0] = usLong;
  userGpsData[1] = usLat;
  userGpsData[2] = usAngle;
}

void checkDistance(){

  double longDistance = userGpsData[0] - carGpsData[0];
  double latDistance = userGpsData[1] - carGpsData[1];
  double totDistance;
  totDistance = sqrt((longDistance*longDistance) + (latDistance*latDistance));

}

void processCoords(){

}

void setup() {
  pinMode(wheel0, OUTPUT);//Define wheel as output
  pinMode(wheel1, OUTPUT);//Define  as output
  pinMode(wheel2, OUTPUT);//Define  as output
  pinMode(wheel3, OUTPUT);//Define  as output
  digitalWrite(wheel0, LOW);
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, LOW);

  Serial.begin(9600); // Default communication rate of the Bluetooth module
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  updateCarCoords();
  updateUserCoords();
}





void loop() {
  if (Serial.available() > 0) { // Checks whether data is coming from the serial port
    char str[] phoneCoord = Serial.readString(); // Reads the data from the serial port
  }
  userGpsData[0] = std::stof(phoneCoord.substring(0, 7)); //reads latitude from string input in the form of "xx.xxxxx yy.yyyyy"
  userGpsData[1] = str::stof(phoneCoord.substring(9, 15)); //reads longitude from string input in the form of "xx.xxxxx yy.yyyyy"



  checkStateWheel(state); //determine if wheels need spin
  Serial.println();
}

