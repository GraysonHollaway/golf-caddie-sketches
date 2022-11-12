#include <Adafruit_GPS.h>

  #define wheel0 50 // Initialize pins to drive wheels
  #define wheel1 51 
  #define wheel2 52 
  #define wheel3 53 

// what's the name of the hardware serial port?
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

uint32_t timer = millis();

void setup() {
  pinMode(wheel0, OUTPUT);//Define wheel as output
  pinMode(wheel1, OUTPUT);//Define  as output
  pinMode(wheel2, OUTPUT);//Define  as output
  pinMode(wheel3, OUTPUT);//Define  as output
  // put your setup code here, to run once:
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
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

}

void loop() {
  // put your main code here, to run repeatedly:

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

  Serial.println(GPS.angle);
  if (GPS.fix){
    while(abs(GPS.angle - 0) > 45.0) {
      //turn right 
      digitalWrite(wheel0, LOW); // Turn left wheels back, right wheels forward
      digitalWrite(wheel1, LOW);
      digitalWrite(wheel2, HIGH);
      digitalWrite(wheel3, HIGH);
    }
    digitalWrite(wheel0, LOW); // Turn wheels off
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, LOW);
  }
  else{
    //move back and forth to get fix
    digitalWrite(wheel0, HIGH); // Turn wheels all on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, LOW);
    delay(1000);
    digitalWrite(wheel0, LOW); // backwards
    digitalWrite(wheel1, HIGH);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
    delay(1000);
  }

  digitalWrite(wheel0, LOW); // Turn wheels off
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, LOW);

}
