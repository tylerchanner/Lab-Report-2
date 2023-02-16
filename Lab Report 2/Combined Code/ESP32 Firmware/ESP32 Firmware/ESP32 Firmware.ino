#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HCSR04.h>
#include <MPU9250.h>
#include <SPI.h>
#include <TFT_22_ILI9225.h>

//TFT Setup

#define TFT_RST 26  // IO 26
#define TFT_RS 25   // IO 25
#define TFT_CLK 14  // HSPI-SCK
//#define TFT_SDO 12  // HSPI-MISO
#define TFT_SDI 13  // HSPI-MOSI
#define TFT_CS 15   // HSPI-SS0
#define TFT_LED 0   // 0 if wired to +5V directly
SPIClass hspi(HSPI);

#define TFT_BRIGHTNESS 200  // Initial brightness of TFT backlight (optional)
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);

float output;
String distance;
String output_callback_variable;
String leftEncoderCountString;

int leftEncoderCount;
int rightEncoderCount;

String motorSpeed;
String ServoAngleNodeRED;

//Sensor setup

MPU9250 mpu;       // initialising the MPU-9250
HCSR04 hc(5, 18);  //initialising the HC-SR04
int yaw;           //initialising the yaw variable

char distanceString[8];    //creating a string for node red communication
String LeftEncoderString;  //creating a string for node red communication
String RightEncoderString;
String avgString;

int16_t leftMotorSpeed = 255;   //initiaising left motor speed
int16_t rightMotorSpeed = 255;  //initiaising right motor speed
int16_t servoAngle = 41;        //initiaising servo angle

float circumferenceOfWheels = 31.42;  // Measured circumference of wheels
float pulsesPerRevolution = 24;       // Number of pulses per revolution for rotary encoders


// Replace the next variables with your SSID/Password combination
const char* ssid = "B6-RaspberryPiFi";
const char* password = "b6channer";

// Add your MQTT Broker IP address
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

//Line Following Setup
#include <Wire.h>
#include <PID_v1.h>

// I2C address of the Arduino Nano
#define NANO_ADDRESS 0x04

// ADC Pin assignments for sensors
#define SENSOR1 32  //Left sensor
#define SENSOR2 35  //Left sensor
#define SENSOR3 33  //Middle left
#define SENSOR4 2   //Middle right
#define SENSOR5 0   //Right
#define SENSOR6 4   //Right


int baseSpeed = 100;  //EEEBot motor stable speed
int leftSpeed = 0;
int rightSpeed = 0;
int servoPos = 20;
int midServo = 20;  //EEEBot center steering
int sensor1Value, sensor2Value, sensor3Value, sensor4Value, sensor5Value, sensor6Value = 0;
int setpoint = 0;  //Setpoint located on the center of the car
int avg = 0;

float K = 0;  //Motor speed scaling factor
// PID variables
double Setpoint, Input, Output;
double Kp = 185, Ki = 0.95, Kd = 0;


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define the weights for each sensor

float weight1 = -40.0;
float weight2 = -25.0;
float weight3 = -8.75;
float weight4 = -8.75;
float weight5 = 25.0;
float weight6 = 40.0;

// Calibration variables
int min_1, min_2, min_3, min_4, min_5, min_6;
int max_1, max_2, max_3, max_4, max_5, max_6;

void callback(char* topic, byte* message, unsigned int length) {  // A callback function that is executed when a message arrives on an MQTT topic
  Serial.print("Message arrived on topic: ");                     // Prints a message to the Serial Monitor indicating that a message has arrived on a topic
  Serial.print(topic);                                            // Prints the topic of the message to the Serial Monitor
  Serial.print(". Message: ");                                    // Prints a message to the Serial Monitor indicating that the message will be printed next

  String messageTemp;  // Declares a string variable called messageTemp

  for (int i = 0; i < length; i++) {  // Loops through the message byte array to print each byte as a character to the Serial Monitor
    Serial.print((char)message[i]);   // Prints a character to the Serial Monitor by casting the byte to a char
    messageTemp += (char)message[i];  // Appends the character to the messageTemp string
  }
  Serial.println();  // Prints a new line character to the Serial Monitor to end the message

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/motorSpeed") {         // Checks if the topic of the message is "esp32/motorSpeed"
    leftMotorSpeed = (int16_t)messageTemp.toInt();   // Converts the messageTemp string to an integer and assigns it to leftMotorSpeed
    rightMotorSpeed = (int16_t)messageTemp.toInt();  // Converts the messageTemp string to an integer and assigns it to rightMotorSpeed
  } else if (String(topic) == "esp32/servoAngle") {  // Checks if the topic of the message is "esp32/servoAngle"
    servoAngle = (int16_t)messageTemp.toInt();       // Converts the messageTemp string to an integer and assigns it to servoAngle
  } else if (String(topic) == "esp32/distance") {    // Checks if the topic of the message is "esp32/distance"
    distance = messageTemp;                          // Assigns the messageTemp string to the distance variable
  } else if (String(topic) == "esp32/Kp") {          // Checks if the topic of the message is "esp32/Kp"
    Kp = messageTemp.toDouble();                     // Converts the messageTemp string to a double and assigns it to the Kp variable
  } else if (String(topic) == "esp32/Ki") {          // Checks if the topic of the message is "esp32/Ki"
    Ki = messageTemp.toDouble();                     // Converts the messageTemp string to a double and assigns it to the Ki variable
  } else if (String(topic) == "esp32/Kd") {          // Checks if the topic of the message is "esp32/Kd"
    Kd = messageTemp.toDouble();                     // Converts the messageTemp string to a double and assigns it to the Kd variable
  }
}


void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();  // Initialize the I2C bus for communication with the MPU sensor
  delay(100);
  hspi.begin();
  tft.begin(hspi);

  // Set the pin modes for the sensor pairs
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);
  pinMode(SENSOR6, INPUT);

  // Start the I2C communication
  Wire.begin();
  Serial.begin(9600);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  Setpoint = 0;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi network using the specified SSID and password
  WiFi.begin(ssid, password);

  // Wait for the connection to be established
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print a message indicating that the WiFi is connected, and the local IP address
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

      // Add your subscribe topics here
      // --
      client.subscribe("esp32/distance");
      client.subscribe("esp32/motorSpeed");
      client.subscribe("esp32/servoAngle");
      client.subscribe("esp32/Kp");
      client.subscribe("esp32/Ki");
      client.subscribe("esp32/Kd");
      // --

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  Wire.requestFrom(0x04, 2);  // Request 2 bytes of data from the Arduino Nano
  if (Wire.available() >= 2) {
    leftEncoderCount = Wire.read();
    rightEncoderCount = Wire.read();
  }
  // Do calcs n stuff

  // Convert the number of pulses to a distance for each motor
  float distanceLeftEncoder = (leftEncoderCount * circumferenceOfWheels) / pulsesPerRevolution;
  float distanceRightEncoder = (rightEncoderCount * circumferenceOfWheels) / pulsesPerRevolution;

  Serial.print("Distance Left Encoder: ");
  Serial.println(distanceLeftEncoder);
  Serial.print("Distance Right Encoder: ");
  Serial.println(distanceRightEncoder);

  //PID
  //Checking constants
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.println(Ki);
  Serial.print("Kd: ");
  Serial.println(Kd);



  myPID.SetSampleTime(10);
  //Calibrated max and min values
  min_1 = 1000;
  min_2 = 1000;
  min_3 = 1000;
  min_4 = 1000;
  min_5 = 1000;
  min_6 = 1000;

  max_1 = 4095;
  max_2 = 4095;
  max_3 = 4095;
  max_4 = 4095;
  max_5 = 4095;
  max_6 = 4095;

  // Read the values from all 6 sensors
  sensor1Value = analogRead(SENSOR1);
  sensor2Value = analogRead(SENSOR2);
  sensor3Value = analogRead(SENSOR3);
  sensor4Value = analogRead(SENSOR4);
  sensor5Value = analogRead(SENSOR5);
  sensor6Value = analogRead(SENSOR6);

  //Constrains the values to a specified range, to disallow extremes
  sensor1Value = constrain(sensor1Value, min_1, max_1);
  sensor2Value = constrain(sensor2Value, min_2, max_2);
  sensor3Value = constrain(sensor3Value, min_3, max_3);
  sensor4Value = constrain(sensor4Value, min_4, max_4);
  sensor5Value = constrain(sensor5Value, min_5, max_5);
  sensor6Value = constrain(sensor6Value, min_6, max_6);

  //Maps the values into the 0 to 255 range
  sensor1Value = map(sensor1Value, min_1, max_1, 0, 500);
  sensor2Value = map(sensor2Value, min_2, max_2, 0, 500);
  sensor3Value = map(sensor3Value, min_3, max_3, 0, 500);
  sensor4Value = map(sensor4Value, min_4, max_4, 0, 500);
  sensor5Value = map(sensor5Value, min_5, max_5, 0, 500);
  sensor6Value = map(sensor6Value, min_6, max_6, 0, 500);



  // Calculate the weighted average of the sensor values
  float num = (sensor1Value * weight1 + sensor2Value * weight2 + sensor3Value * weight3 + sensor4Value * weight4 + sensor5Value * weight5 + sensor6Value * weight6);
  float denum = (sensor1Value + sensor2Value + sensor3Value + sensor4Value + sensor5Value + sensor6Value);

  float avg = Setpoint - (num / denum);

  //Serial.print("Weighted average: ");
  //Serial.println(avg);

  Input = avg;
  myPID.Compute();

  Serial.print("Output:");
  Serial.println(Output);

  // Calculate the new speed and steering values
  //Right turns are minus values
  //Left turns are positive values
  leftMotorSpeed = baseSpeed + (K * Output);
  rightMotorSpeed = baseSpeed - (K * Output);
  servoAngle = midServo + Output;




  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float distanceToObstacle = hc.dist();  //updating the sensor distance

  if (mpu.update()) {    //checking if the mpu has updated
    yaw = mpu.getYaw();  //setting the yaw to the updated yaw value
  }

  dtostrf(distanceToObstacle, 1, 2, distanceString);
  client.publish("esp32/distance", distanceString);

  LeftEncoderString = String(leftEncoderCount);
  client.publish("esp32/leftEncoder", LeftEncoderString.c_str());
  RightEncoderString = String(rightEncoderCount);
  avgString = String(avg);
  client.publish("esp32/leftEncoder", LeftEncoderString.c_str());
  client.publish("esp32/rightEncoder", RightEncoderString.c_str());
  client.publish("esp32/weightedAverage", avgString.c_str());

  tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
  tft.setFont(Terminal12x16);
  tft.drawText(10, 70, "Distance");
  tft.drawText(10, 100, distanceString);
  tft.drawText(10, 130, "Encoder Count");
  tft.drawText(10, 160, String(distanceLeftEncoder));
  delay(20);







  //Communicate with arduino


  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);  // Limit left motor speed between -255 and 255


  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);  // Limit right motor speed between -255 and 255


  servoAngle = constrain(servoAngle, 0, 180);  // Limit servo angle between 0 and 180

  transmitArduino(leftMotorSpeed, rightSpeed, servoPos);
}

// Send the values to the Arduino Nano
void transmitArduino(int leftMotorSpeed, int rightMotorSpeed, int servoAngle) {

  Wire.beginTransmission(NANO_ADDRESS);                     // transmit to device #4
  Wire.write((byte)((leftMotorSpeed & 0x0000FF00) >> 8));   // first byte of leftMotor, containing bits 16 to 9
  Wire.write((byte)(leftMotorSpeed & 0x000000FF));          // second byte of leftMotor, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotorSpeed & 0x0000FF00) >> 8));  // first byte of rightMotor, containing bits 16 to 9
  Wire.write((byte)(rightMotorSpeed & 0x000000FF));         // second byte of rightMotor, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));       // first byte of rightMotor, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));              // second byte of rightMotor, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                                   // stop transmitting
}