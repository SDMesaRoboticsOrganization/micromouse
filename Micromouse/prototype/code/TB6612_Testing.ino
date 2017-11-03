#include <Encoder.h>
#include <VL6180X.h>
#include <Wire.h>

//motor A connected between A01 and A02
//motor B connected between B01 and B02

int STBY = 9; //standby

//Motor A
int PWMA = 30; //Speed control
int AIN1 = 10; //Direction
int AIN2 = 11; //Direction

//Motor B
int PWMB = 4; //Speed control
int BIN1 = 6; //Direction
int BIN2 = 5; //Direction



/**
   IR Sensor
*/

// IR Sensor Definitions
VL6180X sensor1;                                     // Sensor that measures from Servo to plate
VL6180X sensor2;
int sensor1_pin = 3, sensor2_pin = 2;    // Digital Pins to set GPI00 High or Low.

/**
   End IR declaration
*/


/**
   Encoders
*/

Encoder knobLeft(33, 34);
Encoder knobRight(7, 8);

/**
   End Encoder declaration
*/

void setup() {

  /**
     Motor controller
  */
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  /**
     End motor controller setup code
  */
  /**
     IR Sensor setup
  */
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


  delay(1000);
  /**
     End IR Sensor setup
  */

}

/**
   Encoders
*/
long positionLeft  = -999;
long positionRight = -999;

void loop() {

  readEncoder();

  move(1, 128, 1);      //motor 1, full speed, left
  //move(2, 255, 1);    //motor 2, full speed, left
  //move(1, 255, 2);    //motor 1, full speed, right?

  readEncoder();

  delay(1000);          //go for 1 second
  stop();               //stop
  delay(250);           //hold for 250ms until move again

  move(1, 128, 0);      //motor 1, half speed, right

  //move(2, 128, 0);    //motor 2, half speed, right

  readEncoder();

  delay(1000);
  stop();
  delay(250);
}

void moveTowardsWall() {
  
}

double getDistanceA() {
  double distanceToReturn;
  //Get Distance and report in mm
  //Serial.print("Distance measured (in) = ");
  //Serial.println( sensor1.readRangeSingle()/25.4 );

  distanceToReturn = sensor1.readRangeSingle() / 25.4;

  delay(500);
  return distanceToReturn;
}

double getDistanceB() {
  double distanceToReturn;

  distanceToReturn = sensor2.readRangeSingle() / 25.4;

  //Get Distance and report in mm
  //Serial.print("Distance measured (in) = ");
  //Serial.println( sensor2.readRangeSingle()/25.4 );

  delay(500);
  return distanceToReturn;
}

void readEncoder() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}

void move(int motor, int speed, int direction) {
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop() {
  //enable standby
  digitalWrite(STBY, LOW);
}
