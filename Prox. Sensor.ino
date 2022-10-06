//Interface a PING ultrasonic sensor which uses ultrasonic waves to measure distance
//It consists of 3 pins: GND, 5V, and SIG
//Datasheet available: https://www.parallax.com/sites/default/files/downloads/28015-PING-Sensor-Product-Guide-v2.0.pdf

#include <LiquidCrystal.h>

//variable declarations
long duration;
int distancecm;
long distancein;
const int RS = 2, EN = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7;

LiquidCrystal lcd(RS,EN,D4,D5,D6,D7);   //set Uno pins that are connected to LCD, 4-bit mode

void setup() {
  //lcd.begin(16,2);          //set 16 columns and 2 rows of 16x2 LCD
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  Serial.begin(9600);     
}

void loop() {
  //Trigger the sensor with a 2usec HIGH pulse
  digitalWrite(8, HIGH);
  delayMicroseconds(2);
  digitalWrite(8, LOW);

  pinMode(8, INPUT);              //set pin as input to read return pulse from sensor
  duration = pulseIn(8, HIGH);    //obtain length of HIGH pulse in microseconds

  //Ping sensor returns HIGH output pulse that gives the time required for the burst
  //echo to return back to the sensor 

  //Speed of sound travels at 1130ft/sec (0.00113ft/usec) or 340m/sec (0.034cm/usec)
  //Must divide by 2 since time received is time for sound wave to travel to and from object

  //Calculate distance in inches and centimeters
  distancecm = duration*0.034/2;
  distancein = distancecm*0.3937;     //1cm = 0.3937in

  //display distance to LCD
  // lcd.clear();
  // lcd.print("Distance: ");
  // lcd.print(distancecm);
  // lcd.print("cm");
  // lcd.setCursor(10,1);
  // lcd.print(distancein);
  // lcd.print("in");
  Serial.println(distancecm);
  delay(20);
  pinMode(8, OUTPUT);               //set pin 8 back to an output to trigger next distance measurement
  digitalWrite(8, LOW);

}
