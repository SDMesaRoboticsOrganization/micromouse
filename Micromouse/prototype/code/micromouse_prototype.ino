#include <Encoder.h>
#include <VL6180X.h>
#include <Wire.h>

/*
 * This is the code for the initial micromouse prototype
 */


//Motors setup
int STBY = 9; //standby
//Motor A
int PWMA = 30; //Speed control
int AIN1 = 10; //Direction
int AIN2 = 11; //Direction
//Motor B
int PWMB = 4; //Speed control
int BIN1 = 6; //Direction
int BIN2 = 5; //Direction

// IR Sensor setup
VL6180X sensor1;  //left IR
VL6180X sensor2;  //right IR
int sensor1_pin = 3, sensor2_pin = 2;    // Digital Pins to set GPI00 High or Low.

//Encoder setup
Encoder knobLeft(33, 34);
Encoder knobRight(7, 8);

//Global vars

long positionLeft  = -999;
long positionRight = -999;
double ir_difference;
int right_rotations;
int left_rotations;
int deadZone = 1;


void setup()
{

  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Wire.setSDA(8);
  Wire.setSCL(7);

  pinMode(sensor1_pin, OUTPUT);                     // IR sensor calibration and activation.
  pinMode(sensor2_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  Serial.begin(9600);                               // Begin serial transmission of data.
  Wire.begin();

  // Sensor 1:
  digitalWrite(sensor1_pin, HIGH);                  // Switch on IR sensor 1.
  delay(50);                                        // 50ms second delay for sensor to come out of standby.
  sensor1.init();                                   // Initialise sensor 1.
  sensor1.configureDefault();                       // Load default configuration of sensor.
  sensor1.setTimeout(500);                          // Timeout time of sensor 0.5 seconds.
  sensor1.setAddress(0x30);                         // Set I2C address as for memory allocation as 0x30.

  // Sensor 2:
  digitalWrite(sensor2_pin, HIGH);                  // Switch on IR sensor 2.
  delay(50);                                        // 50ms second delay for sensor to come out of standby.
  sensor2.init();                                   // Initialise sensor 2.
  sensor2.configureDefault();                       // Load default configuration of sensor.
  sensor2.setTimeout(500);                          // Timeout time of sensor 0.5 seconds.
  sensor2.setAddress(0x31);                         // Set I2C address as for memory allocation as 0x31

  while (!Serial);                                  // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");                  // Set I2C adress as for memory allocation as 0x56.

  delay(10);
}


void loop()
{
  calc_dist_diff();
  turn();
}


void turn()
{
  int turn_speed = 1;
  int turn_scalar = 25;
  if(ir_difference > deadZone)
  {
    turn_left();
  }
  else if(ir_difference < -deadZone)
  {
    turn_right();
  }
}

void turn_left()
{
  if(ir_difference > 9)
  {
    turn_speed = 3;
  }
  else if (ir_difference > 4)
  {
    turn_speed = 2;
  }
  
  int motor_diff =  turn_scalar*turn_speed;
  move(1, SPEED-motor_diff, 0);
  move(0, SPEED, 0);
}

void turn_right()
{
  if(ir_difference < -9)
  {
    turn_speed = 3;
  }
  else if (ir_difference < -4)
  {
    turn_speed = 2;
  }
  
  int motor_diff =  turn_scalar*turn_speed;
  move(0, SPEED-motor_diff, 0);
  move(1, SPEED, 0);
}


/*
================IR FUNCTIONS================
*/
//Subtract left IR distance from right IR distance
double calc_dist_diff()
{
  double distLeft = getDistanceRight();  //left
  double distRight = getDistanceLeft();  //Right

  ir_difference = (distRight - distLeft);
}

//Reads value from right IR and converts to inches
double getDistanceRight()
{
  double distanceToReturn;
  distanceToReturn = sensor1.readRangeSingle() / 25.4;
  delay(10);
  
  return distanceToReturn;
}

//Read vlue from left IR and converts to inches
double getDistanceLeft()
{
  double distanceToReturn;
  distanceToReturn = sensor2.readRangeSingle() / 25.4;
  delay(10);
  
  return distanceToReturn;
}
/*
================END IR FUNCTIONS================
*/





/*
================ENCODER FUNCTIONS================
*/
//Counts rotation of right Encoder
void readRightEncoder()
{
  long newRight;
  newRight = knobRight.read();
  if (newRight != positionRight)
  {
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionRight = newRight;
  }
}

//Counts rotation of right Encoder
void readLeftEncoder()
{
  long newLeft;
  newLeft = knobLeft.read();
  if (newLeft != positionLeft)
  {
    Serial.print(", Right = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
}
/*
================END ENCODER FUNCTIONS================
*/





/*
================MOTOR FUNCTIONS================
*/
void move(int motor, int speed, int direction)
{
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1)
  {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1)
  {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else
  {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop()
{
  //enable standby
  digitalWrite(STBY, LOW);
}
/*
================END MOTOR FUNCTIONS================
*/
