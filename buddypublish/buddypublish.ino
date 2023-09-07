// SIT210 Task 3.3D
// Name : Om Tomar
// Student Id : 2210994882

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <WiFi101.h>
#include <ESP8266WiFi.h>
#include <WiFi.h>

// Check the board type and include the appropriate libraries based on the board.
// This section checks the type of Arduino board being used and includes the necessary libraries accordingly.

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
// Include libraries for specific board types.
#elif defined(ARDUINO_SAMD_MKR1000)
// Include libraries for another board type.
#elif defined(ARDUINO_ARCH_ESP8266)
// Include libraries for ESP8266 board.
#elif defined(ARDUINO_ARCH_ESP32)
// Include libraries for ESP32 board.
#endif

// Define Wi-Fi credentials.
char ssid[] = "DUA";    // Replace with your Wi-Fi SSID
char pass[] = "06122003";  // Replace with your Wi-Fi password

// Define the pins for the ultrasonic sensor.
const int trigPin = 2;   // Trigger pin for the ultrasonic sensor
const int echoPin = 3;   // Echo pin for the ultrasonic sensor

// Variables for ultrasonic sensor.
float duration, distance;

// Initialize Wi-Fi client and MQTT client.
WiFiClient wifiClient;      // Create a Wi-Fi client object
MqttClient mqttClient(wifiClient);  // Create an MQTT client object using the Wi-Fi client

// Define MQTT broker information.
const char broker[] = "broker.mqttdashboard.com";  // MQTT broker server address
int port = 1883;  // MQTT broker port
const char topic[] = "SIT210/waves";  // MQTT topic to publish messages to

// Time interval for sending MQTT messages.
const long interval = 1000;  // Interval between MQTT messages in milliseconds
unsigned long previousMillis = 0;  // Store the previous time MQTT message was sent

// Counter for debugging purposes.
int count = 0;  // A counter for debugging, not used in the core functionality

void setup() {
  // Initialize serial communication.
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect (needed for native USB port).
  }

  // Attempt to connect to the Wi-Fi network.
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // Attempt to connect to the MQTT broker.
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1); // Infinite loop if MQTT connection fails.
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  // Call poll() regularly to allow the library to send MQTT keep-alives.
  // This prevents disconnection by the broker.
  mqttClient.poll();

  // To avoid delays in the loop, use the BlinkWithoutDelay strategy.
  unsigned long currentMillis = millis();

  // Call the detectMotion() function.
  detectMotion();
}

void detectMotion() {
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Ultrasonic sensor logic to measure distance.
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);

    // If distance is less than 12 (motion detected), send MQTT message.
    if (distance < 12) {
      mqttClient.beginMessage(topic);
      mqttClient.print("Aditya Dua : Wave is detected, ");
      mqttClient.print("Distance: ");
      mqttClient.print(distance);
      mqttClient.endMessage();
      delay(1000);
    }

    Serial.println();

    count++;
  }
}  