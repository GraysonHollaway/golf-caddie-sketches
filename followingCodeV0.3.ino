//definitions
  #include <math.h>
  #include <TinyGPS++.h>
  #include <DFRobot_QMC5883.h>
  #include <Wire.h>
  #include "NewPing.h"
  //#include <Adafruit_GPS.h>

  #define DEGTORAD 0.0174532925199432957f 
  #define RADTODEG 57.295779513082320876f
  #define TWOPI 6.28318530718f
  #define DECLINATIONANGLE 0.077376235F
  //= (4.0 + (26.0 / 60.0)) / (180 / PI);
  //#define GPSECHO true

  #define TRIGGER_PIN 37
  #define ECHO_PIN 36
  #define MAX_SONAR_DIST 500

  #define wheel0 46 // Initialize pins to drive wheels
  #define wheel1 51 
  #define wheel2 48 
  #define wheel3 53 

//

//declarations
  struct Coord{
    float lat;
    float lon;
  };
  struct Route{
    float dist;
    float head;
  };

  DFRobot_QMC5883 compass(&Wire, /*I2C addr*/HMC5883L_ADDRESS);
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DIST);

  TinyGPSPlus GPS;  
  Coord carLoc;
  Coord destLoc;
  Coord data;
  Route moveTo;
  float carBearing;
  char newInstr = '0';
  unsigned long sonarDist = 0;
  sVector_t mag;
  //Adafruit_GPS GPS(&Serial3);
//

//Pair Maker
  Coord makePair(float &latP, float &lonP){
    Coord pairP;
    pairP.lat = latP;
    pairP.lon = lonP;
    return pairP;
  }
//

// //GPS Updaters
  //   Coord GPScheck(){
  //     Serial.println("reading NMEA feed");
  //     bool newData = false;
  //     unsigned long time = millis(); 
  //     do{
  //         if(GPSfeed()){
  //           newData = true;
  //         }
  //     }while((millis()-time) < 1000);
    
  //     if(newData){
  //       return GPSdump(GPS);
  //     }else{
  //       return carLoc;
  //     }  
  //   }


  //   Coord GPSdump(TinyGPS &gps){
  //     float latP;
  //     float lonP;
  //     unsigned long fixAge;

  //     gps.f_get_position(&latP, &lonP, &fixAge);

  //     Coord pairP;
  //     pairP.lat = latP;
  //     pairP.lon = lonP;

  //     Serial2.print("\n");
  //     Serial2.print(pairP.lat, 7); 
  //     Serial2.print(", "); 
  //     Serial2.println(pairP.lon, 7);  

  //     return pairP;
  //   }

  //   bool GPSfeed(){
  //     bool availability = false;
  //     while(Serial3.available()){
  //       if(GPS.encode(Serial3.read()))
  //       {
  //         availability = true;
  //       }
  //       return availability; 
  //     } 
  //   }

  //void updateCarPos(){
  //  if(GPSfeed()){
  //    data = GPScheck();
  //  }else{
  //    data = carLoc;
  //  }
  //    carLoc = data;  
  //}
//  //

//
  void updateCarPos(){
    bool validgps;
    if(Serial3.available()){
      validgps = GPS.encode(Serial3.read());
    }
      if(validgps){
        carLoc.lat = GPS.location.lat();
        carLoc.lon = GPS.location.lng();
        //Serial2.println(carLoc.lat);
        //Serial2.println(carLoc.lon);
      }
  }
//

//Wheel Action Updaters
  void stopWheel(){
    digitalWrite(wheel0, LOW); // Turn wheels off
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, LOW);
  }

  void forwardWheel(){
    digitalWrite(wheel0, HIGH); // Turn wheels all on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, LOW);
  }

  void leftTurnWheel(){
    digitalWrite(wheel0, LOW); // Turn left wheels back, right wheels forward
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, HIGH);
  }

  void rightTurnWheel(){
    digitalWrite(wheel0, HIGH); // Turn left wheels forward, right wheeels back
    digitalWrite(wheel1, HIGH);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, LOW);
  }

  void backwardsWheel() {
    digitalWrite(wheel0, LOW); // backwards
    digitalWrite(wheel1, HIGH);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
  }
//

//Motion Sensor Updater
  void checkMotionSensor(){
    sonarDist = sonar.ping_cm();
    if(sonarDist < 100 && sonarDist > 10){
      fullKill();
    }
  }
//

//Compass Updater
  void bearingUpdate(){
    compass.setDeclinationAngle(DECLINATIONANGLE);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    carBearing = compass.readRaw().HeadingDegress;
  }
//

//Kill Command
  void fullKill(){
    stopWheel();
    delay(100);
    exit(0);
  }
//

//Data Processing
  char doCalcs(){

    float distance = 0.0;
    float angle = 0.0;

    distance = makeDist(carLoc, destLoc);
    angle = makeAngle(carLoc, destLoc);

    moveTo.dist = distance;
    moveTo.head = angle;

    return '0';
  }

  float makeAngle(struct Coord &b, struct Coord &a){
    float alat = a.lat * DEGTORAD;
    float blat = b.lat * DEGTORAD;
    float dlon = (a.lon - b.lon) * DEGTORAD;
    float a1 = sin(dlon) * cos(alat);
    float a2 =  sin(blat) * cos(alat) * cos(dlon);
    a2 = (cos(blat) * sin(alat)) - a2;
    float ang = atan2(a1, a2);
    if(ang < 0.0){
      ang = ang + TWOPI;
    }
    ang = ang * RADTODEG;
    return ang;
  }

  float makeDist(struct Coord a, struct Coord b){
    float dlat = (a.lat-b.lat) * DEGTORAD / 2;
    float dlon = (a.lon-b.lon) * DEGTORAD / 2;

    //sin^2(dlat) + ( sin^2(dlon) * cos(a.lat) * cos(a.lon) )
    float x = (sin(dlat)*sin(dlat)) + (sin(dlon)*sin(dlon) * cos(a.lat*DEGTORAD)*cos(b.lat*DEGTORAD));
    //arctan^2( sqrt(x) * sqrt(1-x) ) * 2
    float y = (atan2(sqrt(x), sqrt(1-x))) * 2;
    //convert dist to meters
    float z = 6372795 * y;

    return z;
  }
//

//User Input Functions
  Coord getDest(){
    Serial2.print("\nEnter GPS coordinates in the form xx.xxxxxx \n");
    float latd = 0.0;
    float lond = 0.0;

    Serial2.println("\nEnter Destination Latitude:");
    delay(12000);
    if (Serial2.available() > 0) {
      latd = Serial2.parseFloat();
    }
    Serial2.println(latd, 7);
    Serial2.println("\nEnter Destination Longitude:");
    delay(12000);
    if (Serial2.available() > 0) {
      lond = Serial2.parseFloat();
    }
    Serial2.println(lond, 7);

    Coord pairD = makePair(latd, lond);
    return pairD;
  }

  void promptForInput(){
    if(newInstr == '0'){
      Serial2.println("If you wish to update to new coordinates- press (y):");
    }  
  }

  void readInput(){
    if (Serial2.available() > 0) {
      delay(500);
      newInstr = Serial2.read();
    }
  }
  
//

//Drive Fuctions

  void goDist(){
    Serial2.println("begin forward");
    forwardWheel();
    doCalcs();
    while(moveTo.dist > 1){
      checkMotionSensor();
      updateCarPos();
      delay(10);      
      doCalcs();
      Serial2.println(moveTo.dist);
    }
    stopWheel();
    delay(100);
    Serial2.println("at destination");
  }

  int shortestTurn(float current, float target){
    int curint = (int)current;
    int tarint = (int)target;
    int dir = 0;
    float d = (tarint - curint + 540) % 360 - 180;
    if(d > 0){
      //right turn
      dir = 1;
    } //else left turn
    return dir;
  }

  void turnToAng(){
    int dir = 0;
    Serial2.println("begin turning");
    bearingUpdate();
    float angle = moveTo.head;
    dir = shortestTurn(carBearing, angle);
    while(abs(carBearing - angle) > 2.5){
      if(dir = 1){
        rightTurnWheel();
      } else {
        leftTurnWheel();
      }
      bearingUpdate();
      delay(10);
    }

    stopWheel();
    Serial2.println("angle set!");
    Serial2.println("facing "+ (String)carBearing +"degrees from north");

  }

//

/*
GPS:            Serial3
BT:             Serial2
Compass:        Serial1 
Motion Sensor:  Pins 36&37
*/

//setup
  void setup() {
    //Serial2.println("Beginning Setup");
    delay(3000);
    // BTSetup
    Serial2.begin(9600);
    delay(3000);
    Serial2.println("BT Setup");
    //GPS Setup
    Serial3.begin(9600);
    delay(1000);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
    //GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    Serial2.print("GPS Setup");


    //Compass setup
    compass.begin();
    mag = compass.readRaw();
    compass.setDeclinationAngle(DECLINATIONANGLE);
    Serial2.println("Compass Setup");

    Serial2.println("begining main loop");
  }
//

void loop() {
  delay(1000);
  updateCarPos();
  promptForInput();
  while(newInstr != 'y'){
    readInput();
    updateCarPos();
  }
  if(newInstr == 'y'){
    destLoc = getDest();
    updateCarPos();
    newInstr = doCalcs();
    Serial2.println("\nThe car must move "+ (String)moveTo.dist +" meters after adjusting its angle to "+ (String)moveTo.head +" degrees from north.\n");
    turnToAng();
    goDist();
  }
}

