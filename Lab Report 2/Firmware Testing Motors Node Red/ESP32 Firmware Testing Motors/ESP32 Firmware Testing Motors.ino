#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>


String motorSpeed;
String ServoAngleNodeRED;


int16_t leftMotorSpeed = 0;   //initiaising left motor speed
int16_t rightMotorSpeed = 0;  //initiaising right motor speed
int16_t servoAngle = 41;        //initiaising servo angle

int leftEncoderCount;
int rightEncoderCount;

float circumferenceOfWheels = 31.42;  // Measured circumference of wheels
float pulsesPerRevolution = 24;       // Number of pulses per revolution for rotary encoders


// Replace the next variables with your SSID/Password combination
const char* ssid = "B6-RaspberryPiFi";
const char* password = "b6channer";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/motorSpeed") {
    //motorSpeed = messageTemp;
  }else if (String(topic) == "esp32/servoAngle") {
    ServoAngleNodeRED = messageTemp;
  }
}


void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();  // Initialize the I2C bus for communication with the MPU sensor
  delay(100);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

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
      //client.subscribe("esp32/output");
      client.subscribe("esp32/motorSpeed");
      client.subscribe("esp32/servoAngle");
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
/*
    Serial.print("Distance Left Encoder: ");
    Serial.println(distanceLeftEncoder);
    Serial.print("Distance Right Encoder: ");
    Serial.println(distanceRightEncoder);
*/



  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  //Communicate with arduino


  leftMotorSpeed = (int16_t)motorSpeed.toInt();
  rightMotorSpeed = (int16_t)motorSpeed.toInt();
  servoAngle = (int16_t)ServoAngleNodeRED.toInt();

  //Serial.println(leftMotorSpeed);
  //Serial.println(rightMotorSpeed);

  Serial.println(servoAngle);



  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);  // Limit left motor speed between -255 and 255


  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);  // Limit right motor speed between -255 and 255


  servoAngle = constrain(servoAngle, 0, 180);  // Limit servo angle between 0 and 180

  Wire.beginTransmission(0x04);        // Prepare and send control values to the I2C slave device with address 4
  Wire.write(leftMotorSpeed >> 8);     // Send the first byte of the left motor speed control value
  Wire.write(leftMotorSpeed & 0xff);   // Send the second byte of the left motor speed control value
  Wire.write(rightMotorSpeed >> 8);    // Send the first byte of the right motor speed control value
  Wire.write(rightMotorSpeed & 0xff);  // Send the second byte of the right motor speed control value
  Wire.write(servoAngle >> 8);         // Send the first byte of the servo angle control value
  Wire.write(servoAngle & 0xff);       // Send the second byte of the servo angle control value
  Wire.endTransmission();              // End the transmission and send the data to the I2C slave device
}