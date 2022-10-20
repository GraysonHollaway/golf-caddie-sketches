#define RIGHT_MOTOR_POS 4
#define RIGHT_MOTOR_NEG 5
#define LEFT_MOTOR_POS 6
#define LEFT_MOTOR_NEG 7
//int i = 1;

void move_forward(int meters);
void stop();
void turn(int angle);

void setup() {
  // put your setup code here, to run once:
  pinMode(RIGHT_MOTOR_POS, OUTPUT);
  pinMode(RIGHT_MOTOR_NEG, OUTPUT);
  pinMode(LEFT_MOTOR_POS, OUTPUT);
  pinMode(LEFT_MOTOR_NEG, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);

  move_forward(1);
  delay(2000);
  turn(1);
  delay(1000);

  stop();
  delay(10000);
}

void move_forward(int meters){
  digitalWrite(RIGHT_MOTOR_POS, HIGH);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
  digitalWrite(LEFT_MOTOR_POS, HIGH);
  digitalWrite(LEFT_MOTOR_NEG, LOW);
  delay(2000);
}

void stop(){
  digitalWrite(RIGHT_MOTOR_POS, HIGH);
  digitalWrite(RIGHT_MOTOR_NEG, HIGH);
  digitalWrite(LEFT_MOTOR_POS, HIGH);
  digitalWrite(LEFT_MOTOR_NEG, HIGH);
}

void turn(int angle){
  digitalWrite(RIGHT_MOTOR_POS, HIGH);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
  digitalWrite(LEFT_MOTOR_POS, LOW);
  digitalWrite(LEFT_MOTOR_NEG, HIGH);
  delay(2000);
}
