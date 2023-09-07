// ADITYA DUA
// Student ID: 2210994755
// SIT-210
// Task: 3.3D

// Include necessary libraries
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <WiFi101.h>
#include <ESP8266WiFi.h>
#include <WiFi.h>

// Check the board type to include the appropriate libraries
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#elif defined(ARDUINO_SAMD_MKR1000)
#elif defined(ARDUINO_ARCH_ESP8266)
#elif defined(ARDUINO_ARCH_ESP32)
#endif

// Wi-Fi credentials
char ssid[] = "DUA";   
char pass[] = "06122003";    

// Pin for controlling an LED
int light = 2;

/* To establish a secure SSL/TLS connection:
  1) Replace 'WiFiClient' with 'WiFiSSLClient'.
  2) Change the 'port' value from 1883 to 8883.
  3) Ensure the 'broker' value corresponds to a server that has a trusted SSL/TLS root certificate configured in the WiFi module.
*/

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// MQTT broker details
const char broker[] = "mqtt-dashboard.com";
int port = 1883;
const char topic[]  = "SIT210/waves";

void setup() {
  Serial.begin(9600);
  pinMode(light, OUTPUT);
  while (!Serial) {;}

  // Connect to Wi-Fi
  Serial.print("Trying to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connection established.");
  Serial.println();

  // Connect to MQTT broker
  Serial.print("Trying to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // Subscribe to a specific MQTT topic
  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  mqttClient.subscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();
}

// Function to blink the LED
void blink()
{
  digitalWrite(light, HIGH);     // LED TURN ON
  delay(200);                  
  digitalWrite(light, LOW);    // LED TURN OFF
  delay(200);
  digitalWrite(light, HIGH);   // LED TURN ON
  delay(200);                  
  digitalWrite(light, LOW);    // LED TURN OFF
  delay(200);
  digitalWrite(light, HIGH);   // LED TURN ON
  delay(200);                  
  digitalWrite(light, LOW);    // LED TURN OFF
  delay(200);
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();
    blink();  // Blink the LED 3 times whenever motion is detected with a 200-millisecond delay
    Serial.println();
  }
}