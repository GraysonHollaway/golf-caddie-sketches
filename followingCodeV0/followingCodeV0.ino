//definitions
  #include <math.h>
  #include "TinyGPS.h"
  #include <DFRobot_QMC5883.h>
  #include <Wire.h>

  #define DEGTORAD 0.0174532925199432957f 
  #define RADTODEG 57.295779513082320876f
  #define TWOPI 6.28318530718f
  #define DECLINATIONANGLE 0.077376235F
  //= (4.0 + (26.0 / 60.0)) / (180 / PI);
  #define GPSECHO  true



  #define wheel0 46 // Initialize pins to drive wheels
  #define wheel1 51 
  #define wheel2 48 
  #define wheel3 53 

//

struct Coord{
  float lat;
  float lon;
};
struct Route{
  float dist;
  float head;
};

DFRobot_QMC5883 compass(&Wire, /*I2C addr*/HMC5883L_ADDRESS);

/*
     GPS:   Serial3
      BT:   Serial2
Compass?:   Serial1 
*/

TinyGPS GPS;  
Coord carLoc;
Coord destLoc;
Coord data;
Route moveTo;
float carBearing;
char newInstr = '0';


Coord GPScheck(){
  Serial.println("reading NMEA feed");
  bool newData = false;
  unsigned long time = millis(); 
  do{
      if(GPSfeed()){
        newData = true;
      }
  }while((millis()-time) < 1000);
  
  if(newData){
    return GPSdump(GPS);
  }else{
    return carLoc;
  }  
}


Coord GPSdump(TinyGPS &gps){
  float latP;
  float lonP;
  unsigned long fixAge;

  gps.f_get_position(&latP, &lonP, &fixAge);

  Coord pairP;
  pairP.lat = latP;
  pairP.lon = lonP;

  Serial2.print("\t");
  Serial2.print(pairP.lat, 7); 
  Serial2.print(", "); 
  Serial2.println(pairP.lon, 7);  

  return pairP;
}

bool GPSfeed(){
  bool availability = false;
  while(Serial3.available()){
    if(GPS.encode(Serial3.read()))
    {
      availability = true;
    }
    return availability; 
  } 
}

void updateCarPos(){
  if(GPSfeed()){
    data = GPScheck();
  }else{
    data = carLoc;
  }
  carLoc = data;
}

//haversine distance funtion
float makeDist(struct Coord &a, struct Coord &b){
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

//azimuth angle function (degrees from north)
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

Coord makePair(float &latP, float &lonP){
  Coord pairP;
  pairP.lat = latP;
  pairP.lon = lonP;
  return pairP;
}

Coord getDest(){
  Serial2.print("\nEnter GPS coordinates in the form xx.xxxxxx \n");
  float latd = 0.0;
  float lond = 0.0;

  Serial2.println("\nEnter Destination Latitude:");
  delay(7000);
   if (Serial2.available() > 0) {
     latd = Serial2.parseFloat();
  }
  Serial2.println(latd, 7);
  Serial2.println("\nEnter Destination Longitude:");
  delay(7000);
   if (Serial2.available() > 0) {
     lond = Serial2.parseFloat();
  }
  Serial2.println(lond, 7);

  Coord pairD = makePair(latd, lond);
  return pairD;
}

char doCalcs(){

  float lat1 = carLoc.lat;
  float lon1 = carLoc.lon;

  float distance = 0.0;
  float angle = 0.0;

  carLoc = makePair(lat1, lon1);
  destLoc = getDest();

  distance = makeDist(carLoc, destLoc);
  angle = makeAngle(carLoc, destLoc);

  moveTo.dist = distance;
  moveTo.head = angle;

  Serial2.println("\nThe car must move "+ (String)distance +" meters after adjusting its angle to "+ (String)angle +" degrees from north.\n");
  return '0';
}


void turnToAng(){
  compass.setDeclinationAngle(DECLINATIONANGLE);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  float angle = moveTo.head;

  while(abs(compass.readRaw().HeadingDegress - angle) > 5) { 
    Serial2.println(compass.readRaw().HeadingDegress, 5);
    rightTurnWheel();
    compass.setDeclinationAngle(DECLINATIONANGLE);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
  }
  stopWheel();
  Serial2.println("angle set!");
}

void goDist(){
  Serial2.println("begin forward");
  forwardWheel();
  doCalcs();
  while(moveTo.dist > 1){
    updateCarPos();
    delay(10);
    doCalcs();
  }
  stopWheel();
}

void fullKill(){
  stopWheel();
  delay(100);
  exit(0);
}

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




void setup() {
  // BTSetup
  Serial2.begin(9600);
  delay(3000);
  
  //GPS Setup
  Serial3.begin(9600);
  delay(1000);

  //Compass setup
  compass.begin();
  Serial2.println("begining");
}

void loop() {
  // put your main code here, to run repeatedly:

  updateCarPos();

  Serial2.println("If you wish to update to new coordinates- press (y):");
  if (Serial2.available() > 0) {
    delay(5000);
    newInstr = Serial2.read();
  }
  if(newInstr == 'y'){
    newInstr = doCalcs();
    turnToAng();
    goDist();
  }
    delay(10000);
  

}
