#include <Wire.h>    // Include the Wire library to use I2C communication
#include <PID_v1.h>  // Include the PID library

#define NANO_ADDRESS 0x04  // Define the I2C address of the Arduino Nano

#define SENSOR1 32  // Assign ADC Pin for the left sensor
#define SENSOR2 35  // Assign ADC Pin for the left sensor
#define SENSOR3 33  // Assign ADC Pin for the middle-left sensor
#define SENSOR4 2   // Assign ADC Pin for the middle-right sensor
#define SENSOR5 0   // Assign ADC Pin for the right sensor
#define SENSOR6 4   // Assign ADC Pin for the right sensor

int baseSpeed = 80;                                                                          // Set the base speed of the motors
int leftSpeed = 0;                                                                           // Set the initial speed of the left motor
int rightSpeed = 0;                                                                          // Set the initial speed of the right motor
int servoPos = 20;                                                                           // Set the initial position of the servo
int midServo = 20;                                                                           // Set the center position of the servo
int sensor1Value, sensor2Value, sensor3Value, sensor4Value, sensor5Value, sensor6Value = 0;  // Initialize the sensor values to zero
int setpoint = 0;                                                                            // Set the initial setpoint of the PID controller to zero
int avg = 0;                                                                                 // Initialize the average sensor value to zero

float K = 0;  // Initialize the motor speed scaling factor to zero

double Setpoint, Input, Output;        // Define variables for the PID controller
double Kp = 175, Ki = 10.3, Kd = 7.8;  // Set the PID constants

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // Create the PID controller object

float weight1 = -40.0;  // Set the weight of the left sensor
float weight2 = -25.0;  // Set the weight of the left sensor
float weight3 = -8.75;  // Set the weight of the middle-left sensor
float weight4 = -8.75;  // Set the weight of the middle-right sensor
float weight5 = 25.0;   // Set the weight of the right sensor
float weight6 = 40.0;   // Set the weight of the right sensor

int min_1, min_2, min_3, min_4, min_5, min_6;  // Initialize the minimum calibration values for the sensors
int max_1, max_2, max_3, max_4, max_5, max_6;  // Initialize the maximum calibration values for the sensors

void setup() {
  pinMode(SENSOR1, INPUT);  // Set the mode of the left sensor pin
  pinMode(SENSOR2, INPUT);  // Set the mode of the left sensor pin
  pinMode(SENSOR3, INPUT);  // Set the mode of the middle-left sensor pin
  pinMode(SENSOR4, INPUT);  // Set the mode of the middle-right sensor pin
  pinMode(SENSOR5, INPUT);  // Set the mode of the right sensor pin
  pinMode(SENSOR6, INPUT);  // Set the mode of the right sensor pin

  Wire.begin();        // Start the I2C communication
  Serial.begin(9600);  // Start the serial communication

  myPID.SetMode(AUTOMATIC);  // Set the PID controller mode to automatic

  Setpoint = 0;  // Sets the value of the Setpoint variable to 0
}

void loop() {
  myPID.SetSampleTime(10);  // Set the sample time of the PID controller to 10ms

  min_1 = 1000;  // Sets the minimum value for sensor 1 to 1000
  min_2 = 1000;  // Sets the minimum value for sensor 2 to 1000
  min_3 = 1000;  // Sets the minimum value for sensor 3 to 1000
  min_4 = 1000;  // Sets the minimum value for sensor 4 to 1000
  min_5 = 1000;  // Sets the minimum value for sensor 5 to 1000
  min_6 = 1000;  // Sets the minimum value for sensor 6 to 1000

  max_1 = 4095;  // Sets the maximum value for sensor 1 to 4095
  max_2 = 4095;  // Sets the maximum value for sensor 2 to 4095
  max_3 = 4095;  // Sets the maximum value for sensor 3 to 4095
  max_4 = 4095;  // Sets the maximum value for sensor 4 to 4095
  max_5 = 4095;  // Sets the maximum value for sensor 5 to 4095
  max_6 = 4095;  // Sets the maximum value for sensor 6 to 4095


  sensor1Value = analogRead(SENSOR1);  // Reads the analog value of sensor 1 and stores it in the sensor1Value variable
  sensor2Value = analogRead(SENSOR2);  // Reads the analog value of sensor 2 and stores it in the sensor2Value variable
  sensor3Value = analogRead(SENSOR3);  // Reads the analog value of sensor 3 and stores it in the sensor3Value variable
  sensor4Value = analogRead(SENSOR4);  // Reads the analog value of sensor 4 and stores it in the sensor4Value variable
  sensor5Value = analogRead(SENSOR5);  // Reads the analog value of sensor 5 and stores it in the sensor5Value variable
  sensor6Value = analogRead(SENSOR6);  // Reads the analog value of sensor 6 and stores it in the sensor6Value variable


  sensor1Value = constrain(sensor1Value, min_1, max_1);  // Constrains the value of sensor 1 to be between the minimum and maximum values
  sensor2Value = constrain(sensor2Value, min_2, max_2);  // Constrains the value of sensor 2 to be between the minimum and maximum values
  sensor3Value = constrain(sensor3Value, min_3, max_3);  // Constrains the value of sensor 3 to be between the minimum and maximum values
  sensor4Value = constrain(sensor4Value, min_4, max_4);  // Constrains the value of sensor 4 to be between the minimum and maximum values
  sensor5Value = constrain(sensor5Value, min_5, max_5);  // Constrains the value of sensor 5 to be between the minimum and maximum values
  sensor6Value = constrain(sensor6Value, min_6, max_6);  // Constrains the value of sensor 6 to be between the minimum and maximum values


  sensor1Value = map(sensor1Value, min_1, max_1, 1, 255);  // Map sensor1Value from the range of min_1 to max_1 to a new range of 1 to 255.
  sensor2Value = map(sensor2Value, min_2, max_2, 1, 255);  // Map sensor2Value from the range of min_2 to max_2 to a new range of 1 to 255.
  sensor3Value = map(sensor3Value, min_3, max_3, 1, 255);  // Map sensor3Value from the range of min_3 to max_3 to a new range of 1 to 255.
  sensor4Value = map(sensor4Value, min_4, max_4, 1, 255);  // Map sensor4Value from the range of min_4 to max_4 to a new range of 1 to 255.
  sensor5Value = map(sensor5Value, min_5, max_5, 1, 255);  // Map sensor5Value from the range of min_5 to max_5 to a new range of 1 to 255.
  sensor6Value = map(sensor6Value, min_6, max_6, 1, 255);  // Map sensor6Value from the range of min_6 to max_6 to a new range of 1 to 255.

  sensor1Value = 255 - sensor1Value;  // Invert the value of sensor1Value.
  sensor2Value = 255 - sensor2Value;  // Invert the value of sensor2Value.
  sensor3Value = 255 - sensor3Value;  // Invert the value of sensor3Value.
  sensor4Value = 255 - sensor4Value;  // Invert the value of sensor4Value.
  sensor5Value = 255 - sensor5Value;  // Invert the value of sensor5Value.
  sensor6Value = 255 - sensor6Value;  // Invert the value of sensor6Value.

  float num = (sensor1Value * weight1 + sensor2Value * weight2 + sensor3Value * weight3 + sensor4Value * weight4 + sensor5Value * weight5 + sensor6Value * weight6);  // Calculate the weighted sum of the sensor values.
  float denum = (sensor1Value + sensor2Value + sensor3Value + sensor4Value + sensor5Value + sensor6Value);                                                            // Calculate the sum of the sensor values.

  float avg = Setpoint - (num / denum);  // Calculate the weighted average of the sensor values.

  Input = avg;  // Set Input to the weighted average.

  myPID.Compute();  // Compute the PID.

  Serial.print("Output:");  // Print "Output:" to the serial monitor.
  Serial.println(Output);   // Print the value of Output to the serial monitor.

  leftSpeed = baseSpeed + (K * Output);   // Calculate the new value of leftSpeed.
  rightSpeed = baseSpeed - (K * Output);  // Calculate the new value of rightSpeed.
  servoPos = midServo + Output;           // Calculate the new value of servoPos.

  leftSpeed = constrain(leftSpeed, 0, 255);    // Constrain leftSpeed to the range of 0 to 255.
  rightSpeed = constrain(rightSpeed, 0, 255);  // Constrain rightSpeed to the range of 0 to 255.
  servoPos = constrain(servoPos, -10, 100);    // Constrain servoPos to the range of -10 to 100.
  transmitArduino(leftSpeed, rightSpeed, servoPos); //Send the parameters to the function to send values to arduino

  // Send the values to the Arduino Nano
void transmitArduino(int leftSpeed, int rightSpeed, int servoPos) {

  Wire.beginTransmission(NANO_ADDRESS);                // begin I2C transmission to device with the address NANO_ADDRESS
  Wire.write((byte)((leftSpeed & 0x0000FF00) >> 8));   // write the first byte of leftSpeed (bits 16 to 9) to the I2C bus
  Wire.write((byte)(leftSpeed & 0x000000FF));          // write the second byte of leftSpeed (bits 8 to 1) to the I2C bus
  Wire.write((byte)((rightSpeed & 0x0000FF00) >> 8));  // write the first byte of rightSpeed (bits 16 to 9) to the I2C bus
  Wire.write((byte)(rightSpeed & 0x000000FF));         // write the second byte of rightSpeed (bits 8 to 1) to the I2C bus
  Wire.write((byte)((servoPos & 0x0000FF00) >> 8));    // write the first byte of servoPos (bits 16 to 9) to the I2C bus
  Wire.write((byte)(servoPos & 0x000000FF));           // write the second byte of servoPos (bits 8 to 1) to the I2C bus
  Wire.endTransmission();                              // end the I2C transmission
}
