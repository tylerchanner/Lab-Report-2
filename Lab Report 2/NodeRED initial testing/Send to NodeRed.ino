//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

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

//Sensor setup

MPU9250 mpu;       // initialising the MPU-9250
HCSR04 hc(5, 18);  //initialising the HC-SR04
int yaw;           //initialising the yaw variable

char distanceString[8];  //creating a string for node red communication



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
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    output_callback_variable = messageTemp;
    tft.clear();
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }else if(String(topic)=="esp32/distance"){
    
    distance = messageTemp;
    Serial.println("Distance = ");Serial.print(messageTemp);
  }
}


void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  //client.setCallback(output_callback);
  client.setCallback(callback);
  Wire.begin();  // Initialize the I2C bus for communication with the MPU sensor
  delay(100);
  hspi.begin();
  tft.begin(hspi);

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
      client.subscribe("esp32/distance");
      client.subscribe("esp32/output");
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


  tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
  tft.setFont(Terminal12x16);
  Serial.println(distance);
  Serial.println(output_callback_variable);
  tft.drawText(10, 10, "Output");
  tft.drawText(10, 40, output_callback_variable);
  tft.drawText(10, 70, "Distance");
  tft.drawText(10, 100, distance);
  tft.drawText(10, 130, "Encoder Count");
  tft.drawText(10, 160, distance);
  delay(500);
}