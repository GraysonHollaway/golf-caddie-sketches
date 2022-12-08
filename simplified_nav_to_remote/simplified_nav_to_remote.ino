// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <Adafruit_GPS.h>
#include <math.h>
#include <DFRobot_QMC5883.h>
#include <Wire.h>
//#include <SoftwareSerial.h>
#include <string.h>

DFRobot_QMC5883 compass(&Wire, /*I2C addr*/HMC5883L_ADDRESS);

// what's the name of the hardware serial port?
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false


uint32_t timer = millis();

#define wheel0 46 // Initialize pins to drive wheels
#define wheel1 51 
#define wheel2 48 
#define wheel3 53

double remoteLat;
double remoteLon;
bool enable = true;

void getLocRemote(){
  enable = true;
  String lon = "";
  if(Serial2.available()){
    delay(30);
    while(Serial2.available()){
    char inByte = Serial2.read();
    if(inByte == 'D'){
      enable = false;
    }
    
    Serial2.write(inByte);
    lon += inByte;
    
    }
    if(!enable){
      return;
    }
      
    int commaIndex = lon.indexOf(',');
    String latStr = lon.substring(0, commaIndex);
    String lonStr = lon.substring(commaIndex+1, lon.length());

    remoteLat = latStr.toDouble();
    remoteLon = lonStr.toDouble();
    Serial.println(remoteLat, 10);
    Serial.println(remoteLon, 10);
  }
}


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial2.begin(38400);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  Serial2.begin(38400);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  pinMode(wheel0, OUTPUT);//Define wheel as output
  pinMode(wheel1, OUTPUT);//Define  as output
  pinMode(wheel2, OUTPUT);//Define  as output
  pinMode(wheel3, OUTPUT);//Define  as output
  compass.begin();
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 4000) {
    timer = millis(); // reset the timer
    // Serial.print("\nTime: ");
    // if (GPS.hour < 10) { Serial.print('0'); }
    // Serial.print(GPS.hour, DEC); Serial.print(':');
    // if (GPS.minute < 10) { Serial.print('0'); }
    // Serial.print(GPS.minute, DEC); Serial.print(':');
    // if (GPS.seconds < 10) { Serial.print('0'); }
    // Serial.print(GPS.seconds, DEC); Serial.print('.');
    // if (GPS.milliseconds < 10) {
    //   Serial.print("00");
    // } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    //   Serial.print("0");
    // }
    // Serial.println(GPS.milliseconds);
    // Serial.print("Date: ");
    // Serial.print(GPS.day, DEC); Serial.print('/');
    // Serial.print(GPS.month, DEC); Serial.print("/20");
    // Serial.println(GPS.year, DEC);
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      // Serial.print(", ");
      // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      // Serial.print("Angle: "); Serial.println(GPS.angle);
      // Serial.print("Altitude: "); Serial.println(GPS.altitude);
      // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      // Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      //getLocRemote();
      getLocRemote();
      if(!enable){
        digitalWrite(wheel0, LOW); // Turn wheels off
        digitalWrite(wheel1, LOW);
        digitalWrite(wheel2, LOW);
        digitalWrite(wheel3, LOW);
        
        return;
      }
        
      /*Simple doCalcs*/
      double dlat = remoteLat - GPS.latitudeDegrees;
      double dlon = remoteLon - GPS.longitudeDegrees;

      if(abs(dlat) < 0.00000001 && abs(dlon) < 0.00000001){
        digitalWrite(wheel0, LOW); // error try again
        digitalWrite(wheel1, LOW);
        digitalWrite(wheel2, LOW);
        digitalWrite(wheel3, LOW);
        Serial.println("error in calculation");
        
        return;
      }

      double angle = atan2(dlon, dlat) * 57.2958;
      if(angle < 0) 
        angle += 360;
      Serial.println(angle);


      float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
      compass.setDeclinationAngle(declinationAngle);
      sVector_t mag = compass.readRaw();
      compass.getHeadingDegrees();

      double dist = sqrt(dlat*dlat + dlon*dlon);

      //spin to approx direction
      while((abs(compass.readRaw().HeadingDegress - angle) > 10) && !(dist < 0.00001)) {
        //turn right 
        Serial.println("-------------------------------------");
        Serial.println(compass.readRaw().HeadingDegress, 5);
        Serial.println(angle);
        Serial.println("-------------------------------------");

        if(angle < 180){
          if(compass.readRaw().HeadingDegress < angle || compass.readRaw().HeadingDegress > angle + 180){
            digitalWrite(wheel0, LOW); // Turn clockwise
            digitalWrite(wheel1, LOW);
            digitalWrite(wheel2, HIGH);
            digitalWrite(wheel3, HIGH);
            Serial.println("CLOCKWISE");
          }
          else{
            digitalWrite(wheel0, HIGH); // Turn cc
            digitalWrite(wheel1, HIGH);
            digitalWrite(wheel2, LOW);
            digitalWrite(wheel3, LOW);
            Serial.println("CC");
          }
        }
        else{
          if(compass.readRaw().HeadingDegress < angle && compass.readRaw().HeadingDegress > angle - 180){
            digitalWrite(wheel0, LOW); // Turn clockwise
            digitalWrite(wheel1, LOW);
            digitalWrite(wheel2, HIGH);
            digitalWrite(wheel3, HIGH);
            Serial.println("CLOCKWISE");
          }
          else{
            digitalWrite(wheel0, HIGH); // Turn cc
            digitalWrite(wheel1, HIGH);
            digitalWrite(wheel2, LOW);
            digitalWrite(wheel3, LOW);
            Serial.println("CC");
          }
        }
        
        
        
        declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
        compass.setDeclinationAngle(declinationAngle);
        sVector_t mag = compass.readRaw();
        compass.getHeadingDegrees();
        delay(100);
      }

      digitalWrite(wheel0, LOW); // Turn wheels off
      digitalWrite(wheel1, LOW);
      digitalWrite(wheel2, LOW);
      digitalWrite(wheel3, LOW);
      delay(10);

      

      if(dist < 0.00001){
        digitalWrite(wheel0, LOW); // Turn wheels off
        digitalWrite(wheel1, LOW);
        digitalWrite(wheel2, LOW);
        digitalWrite(wheel3, LOW);
      }
      else{
        digitalWrite(wheel0, HIGH); // Turn wheels all on
        digitalWrite(wheel1, LOW);
        digitalWrite(wheel2, HIGH);
        digitalWrite(wheel3, LOW);
      }

      Serial.println(dist, 10);

      delay(100);
    }
  }
}