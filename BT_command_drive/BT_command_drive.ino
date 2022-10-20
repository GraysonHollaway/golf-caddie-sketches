#define wheel0 4 // Initialize pin 4 to drive a wheel
#define wheel1 5 
#define wheel2 6 
#define wheel3 7 

int state = 0; // To read state of serial data from HC05 module
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
}
void loop() {
  if (Serial.available() > 0) { // Checks whether data is coming from the serial port
    state = Serial.read(); // Reads the data from the serial port
  }
  if (state == '0') {
    digitalWrite(wheel0, LOW); // Turn wheels off
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, LOW);


    Serial.println("STOP"); // Send back, to the phone, the String "LED: ON"
    state = 0;
  }
  else if (state == '1') {
    digitalWrite(wheel0, HIGH); // Turn wheels 1 and 0 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, LOW);
    Serial.println("Forward Turn 0");
    state = 0;
  }
  else if (state == '2') {
    digitalWrite(wheel0, LOW); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, LOW);
    Serial.println("Forward Turn 1");
    state = 0;
  }
  else if (state == '3') {
    digitalWrite(wheel0, LOW); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
    Serial.println("backward Turn 0");
    state = 0;
  }
  else if (state == '4') {
    digitalWrite(wheel0, LOW); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
    Serial.println("backward Turn 1");
    state = 0;
  }
  else if (state == '5') {
    digitalWrite(wheel0, LOW); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, HIGH);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, LOW);
    Serial.println("zero Turn 0");
    state = 0;
  }
  else if (state == '6') {
    digitalWrite(wheel0, HIGH); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
    Serial.println("zero Turn 1");
    state = 0;
  }else if (state == '7') {
    digitalWrite(wheel0, HIGH); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, LOW);
    digitalWrite(wheel2, HIGH);
    digitalWrite(wheel3, LOW);
    Serial.println("Forward");
    state = 0;
  }
  else if (state == '8') {
    digitalWrite(wheel0, LOW); // Turn wheels 2 and 3 on
    digitalWrite(wheel1, HIGH);
    digitalWrite(wheel2, LOW);
    digitalWrite(wheel3, HIGH);
    Serial.println("Backward");
    state = 0;
  }
}

