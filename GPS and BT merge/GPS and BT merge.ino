#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial3);
#define GPSECHO  true
int state = 0; // To read state of serial data from HC05 module


void setup() {
  Serial1.begin(9600); // Default communication rate of the Bluetooth module

  GPS.begin(9600);
  Serial1.println("GPS Test");

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);



}

double carLat = GPS.latitude;
double carLong = GPS.longitude;

void loop() {
  if (Serial1.available() > 0) { // Checks whether data is coming from the serial port
    state = Serial3.read(); // Reads the data from the serial port
  }
  Serial1.println("Lat");
    Serial1.println(carLat);
    Serial1.println("Long");
    Serial1.println(carLong);

    if(false){
    Serial1.print("\nTime: ");
    Serial1.print(GPS.hour, DEC); Serial.print(':');
    Serial1.print(GPS.minute, DEC); Serial.print(':');
    Serial1.print(GPS.seconds, DEC); Serial.print('.');
    Serial1.println(GPS.milliseconds);
    Serial1.print("Date: ");
    Serial1.print(GPS.day, DEC); Serial.print('/');
    Serial1.print(GPS.month, DEC); Serial.print("/20");
    Serial1.println(GPS.year, DEC);
    Serial1.print("Fix: "); Serial.print((int)GPS.fix);
    Serial1.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial1.print("Location: ");
      Serial1.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial1.print(", "); 
      Serial1.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial1.print("Location (in degrees, works with Google Maps): ");
      Serial1.print(GPS.latitudeDegrees, 4);
      Serial1.print(", "); 
      Serial1.println(GPS.longitudeDegrees, 4);
      
      Serial1.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial1.print("Angle: "); Serial.println(GPS.angle);
      Serial1.print("Altitude: "); Serial.println(GPS.altitude);
      Serial1.print("Satellites: "); Serial.println((int)GPS.satellites);
    }

  Serial1.print(state);
  delay(5000);
  double carLat = GPS.latitude;
  double carLong = GPS.longitude;
}
}
