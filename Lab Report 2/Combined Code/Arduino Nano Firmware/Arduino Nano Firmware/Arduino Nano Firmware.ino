//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*                                                      *//
//*  EEEBot Firmware Code for the Arduino NANO           *//
//*  UoN EEEBot 2022                                     *//
//*  Nat Dacombe                                         *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// read through the accompanying readme file - only then ask for help if you are still unsure

// DO NOT modify or edit any of this code - for Session 2, the Arduino NANO code is provided for you
// the only exception is modify the pin numbers for the motors (if the motors do not spin the correct way)

#include <Servo.h>
#include <Encoder.h>
#include <Wire.h>

#define servoPin 4

Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position

// either change your wire connections to the two encoders or change the pin numbers in the code
// to change whether the count increments or decrements when turning in a particular direction
Encoder enc1(2, 11);  // create an encoder object for encoder 1
Encoder enc2(3, 12);  // create an encoder object for encoder 2
long oldPos_enc1 = -999;  
long oldPos_enc2 = -999;
long enc1_count;
long enc2_count;

#define enA 5   // EnableA command line - should be a PWM pin
#define enB 6   // EnableB command line - should be a PWM pin

// name the motor control pins - replace the ** with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  // Channel A direction 
#define INb A1  // Channel A direction 
#define INc A2  // Channel B direction 
#define INd A3  // Channel B direction 


void setup() {
  Wire.begin(4);                // join I2C bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  myservo.attach(servoPin);  // attach our servo object to pin D4
  // the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  // configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  // initialise serial communication
  Serial.begin(9600);
  Serial.println("Arduino Nano is Running"); // sanity check
}


void loop() {
  // update the two encoder values
  enc1_count = enc1.read();
  enc2_count = enc2.read();
  if (enc1_count != oldPos_enc1) {
    oldPos_enc1 = enc1_count;
  }
  if (enc2_count != oldPos_enc2) {
    oldPos_enc2 = enc2_count;
  }
}

// this function executes when data is requested from the master device
void requestEvent(void)
{
  Wire.write(enc1_count);
  Wire.write(enc2_count);
}

// this function executes whenever data is received from the master device
void receiveEvent(int howMany)
{
  if(howMany != 6)  // for 3 16-bit numbers, the data will be 6 bytes long - anything else is an error
  {
    emptyBuffer();
    return;
  }
  
  int16_t leftMotor_speed = 0;
  int16_t rightMotor_speed = 0;
  int16_t servoAngle = 0;
  
  uint8_t leftMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t leftMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
  uint8_t rightMotor_speed16_9 = Wire.read();   // receive bits 16 to 9 of y (one byte)
  uint8_t rightMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of y (one byte)
  uint8_t servoAngle16_9 = Wire.read();   // receive bits 16 to 9 of z (one byte)
  uint8_t servoAngle8_1 = Wire.read();   // receive bits 8 to 1 of z (one byte)

  leftMotor_speed = (leftMotor_speed16_9 << 8) | leftMotor_speed8_1; // combine the two bytes into a 16 bit number
  rightMotor_speed = (rightMotor_speed16_9 << 8) | rightMotor_speed8_1; // combine the two bytes into a 16 bit number
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1; // combine the two bytes into a 16 bit number

  // verify that the correct values are received via the serial monitor
  Serial.print("Left Motor: ");
  Serial.print(leftMotor_speed);
  Serial.print("\t");
  Serial.print("Right Motor: ");
  Serial.print(rightMotor_speed);
  Serial.print("\t");
  Serial.print("Servo: ");
  Serial.println(servoAngle);

  setSteeringAngle(servoAngle);
  runMotors(leftMotor_speed, rightMotor_speed);
}


// function to clear the I2C buffer
void emptyBuffer(void)
{
  Serial.println("Error: I2C Byte Size Mismatch"); // i.e. an incorrect number of bytes has been received
  while(Wire.available())
  {
    Wire.read();
  }
}


// function to set the steering angle
void setSteeringAngle(int servoAngle){
  servoAngle = constrain(servoAngle, 0, 180); // prevents the servo being set to 'impossible' angles
  myservo.write(servoAngle);
}


// function to run the motors
void runMotors(int leftMotor_speed, int rightMotor_speed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);

  // vary the motor speeds - use the absolute value to remove the negative
  analogWrite(enA, abs(leftMotor_speed));
  analogWrite(enB, abs(rightMotor_speed));

  // if the speed value is negative, run the motor backwards
  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }
}
