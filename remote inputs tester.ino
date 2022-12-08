
  #include <TinyGPS++.h>
  #include <DFRobot_QMC5883.h>
  #include <Wire.h>

  #define DEGTORAD 0.0174532925199432957f 
  #define RADTODEG 57.295779513082320876f
  #define TWOPI 6.28318530718f
  #define DECLINATIONANGLE 0.077376235F

  #define wheel0 46 // Initialize pins to drive wheels
  #define wheel1 51 
  #define wheel2 48 
  #define wheel3 53 
//  #define HMC5883L_ADDRESS &Serial1

  struct Coord{
    double lat;
    double lon;
  };

  struct Route{
    float dist;
    float head;
  };

  Coord carLoc;
  Coord destLoc;
  Route moveTo;
  
  char newInstr = '0';
  float carBearing;

  TinyGPSPlus GPS;
  DFRobot_QMC5883 compass(&Wire, /*I2C addr*/HMC5883L_ADDRESS);
  sVector_t mag;
  String rawData ="e";


//Pair Maker
  Coord makePair(double &latP, double &lonP){
    Coord pairP;
    pairP.lat = latP;
    pairP.lon = lonP;
    return pairP;
  }
//

  void bearingUpdate(){
    compass.setDeclinationAngle(DECLINATIONANGLE);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    carBearing = compass.readRaw().HeadingDegress;
  }

  void updateCarPos(){
    bool validgps;
    if(Serial3.available()){
      validgps = GPS.encode(Serial3.read());
    }
    if(validgps){
      carLoc.lat = GPS.location.lat();
      carLoc.lon = GPS.location.lng();
    }
  }

  static void smartDelay(unsigned long ms)
  {
    unsigned long start = millis();
    do 
    {
      updateCarPos();
    } while (millis() - start < ms);
  }


  Coord getDest(){
    Serial2.println("\nEnter GPS coordinates(xx.xxxxxx) in the form xx.xxxxxx");
    String holder = "0.0";
    double latd = 0.0;
    double lond = 0.0;

    Serial2.println("\nEnter Destination Latitude:");
    delay(12000);
    if (Serial2.available() > 0) {
      holder = Serial2.readString();
      latd = (holder.toDouble());
      holder = "0.0";
    }
    Serial2.println(latd, 6);
    Serial2.println("\nEnter Destination Longitude:");
    delay(12000);
    if (Serial2.available() > 0) {
      holder = Serial2.readString();
      lond = (holder.toDouble());
    }
    Serial2.println(lond, 6);

    newInstr = '0';

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
  
  Coord getDestRemote(){

    String holder = "0.1\\n0.1";
    String holderlat = "0.1";
    String holderlon = "0.1";
    int strpos = 0;
    double latd = 0.1;
    double lond = 0.1;
    rawData = "";

    if (Serial2.available() > 0) {
      delay(30);
      while(Serial2.available() > 0){
        char inbyte = Serial2.read();
        rawData = rawData + inbyte;
      }
      holder = rawData;
      //Serial.println(rawData);
      strpos = holder.indexOf(',');
      holderlat = holder.substring(0,strpos);
      holderlon = holder.substring(strpos+1, holder.length());
      
      latd = (holderlat.toDouble());
      lond = (holderlon.toDouble());
      holder = "0.0\\n0.0";

      Coord pairD = makePair(latd, lond);
      return pairD;
    }
  }

  void doCalcs(){

    float distance = 0.0;
    float angle = 0.0;

    distance = makeDist(carLoc, destLoc);
    angle = makeAngle(carLoc, destLoc);

    moveTo.dist = distance;
    moveTo.head = angle;
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
  
  void goDist(){
    Serial.println("begin forward");
    forwardWheel();
    //doCalcs();
    double tempdist = moveTo.dist;
    while(tempdist > 4){
      //checkMotionSensor();
      delay(100);
      updateCarPos();
      delay(1000);      
      doCalcs();
      delay(1000);
      Serial.println(moveTo.dist);
      Serial.println(tempdist);
      tempdist = moveTo.dist;
    }
    stopWheel();
    delay(100);
    Serial.println("at destination");
  }
//

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
    Serial.println("begin turning");
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
    Serial.println("angle set!");
    Serial.println("facing "+ (String)carBearing +"degrees from north");

  }

  void turnToAng2(){
    bearingUpdate();
    while(abs(moveTo.head - carBearing) > 25){
      leftTurnWheel();
      Serial.print("turning left");
      bearingUpdate();
      Serial.println(carBearing);
    }
    stopWheel();
  }




//


  void setup() {
    // put your setup code here, to run once:
    delay(3000);
    Serial.begin(115200);
    // BTSetup
    Serial2.begin(38400);
    delay(3000);
    Serial.println("BT Setup");
    //GPS Setup
    Serial3.begin(9600);
    delay(1000);
    Serial.print("GPS Setup");
    
    compass.begin();
    compass.setDeclinationAngle(DECLINATIONANGLE);
    mag = compass.readRaw();
    Serial.println("Compass Setup");

    smartDelay(3000);
  }

void loop() {

  updateCarPos();
  destLoc = getDestRemote();
  
  Serial.println("\nYour Current Location inputs:");
  Serial.println(carLoc.lat, 6);
  Serial.println(carLoc.lon, 6);

   Serial.println("\nYour Destination inputs:");
   Serial.println(destLoc.lat, 6);
   Serial.println(destLoc.lon, 6);
   //Serial.println(rawData);

   bearingUpdate();
   Serial.println("\nYour Current Bearing:");
   Serial.println(carBearing);


  doCalcs();
  Serial.println("\nThe car must move "+ (String)moveTo.dist +" meters after adjusting its angle to "+ (String)moveTo.head +" degrees from north.\n");

  //turnToAng();
  delay(100);
  //smartDelay(1000);
  //goDist();

  if(abs(moveTo.head - carBearing) > 20){
    Serial.println("turn car");
    turnToAng2();
    stopWheel();
      if(moveTo.dist > 8 && abs(moveTo.head - carBearing) > 10) {
        forwardWheel();
        Serial.println("move forward");
      }
    stopWheel();
  }

  smartDelay(1000);

  // put your main code here, to run repeatedly:

}
