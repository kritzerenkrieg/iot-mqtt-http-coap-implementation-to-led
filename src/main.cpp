#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <coap-simple.h>

// ----------------------------------------------------
// WiFi
// ----------------------------------------------------
const char* WIFI_SSID = "S20ULTRA";
const char* WIFI_PASS = "susuputih";

// ----------------------------------------------------
// Pins
// ----------------------------------------------------
const int LED_MQTT = 23;
const int LED_COAP = 22;
const int LED_HTTP = 21;
const int LED_PROXIMITY = 2;   // 4th LED for proximity indicator

// Ultrasonic sensor pins
const int TRIG_PIN = 19;
const int ECHO_PIN = 18;

// Proximity threshold in cm
const long PROXIMITY_THRESHOLD_CM = 10;

// ----------------------------------------------------
// MQTT
// ----------------------------------------------------
const char* MQTT_HOST = "kritz.my.id";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC_LED = "kritz/led/mqtt";
const char* MQTT_TOPIC_SENSOR = "kritz/sensor/distance";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ----------------------------------------------------
// HTTP Polling
// ----------------------------------------------------
const char* STATES_URL = "https://kritz.my.id/states";

unsigned long lastPollMillis = 0;
const unsigned long POLL_INTERVAL_MS = 2000;

// ----------------------------------------------------
// CoAP Client Polling
// ----------------------------------------------------
const char* COAP_HOST = "kritz.my.id";
const uint16_t COAP_PORT = 5683;
WiFiUDP coapUdp;
Coap coapClient(coapUdp);

unsigned long lastCoapPollMillis = 0;
const unsigned long COAP_POLL_INTERVAL_MS = 3000;

// ----------------------------------------------------
// Ultrasonic Sensor
// ----------------------------------------------------
unsigned long lastSensorReadMillis = 0;
const unsigned long SENSOR_READ_INTERVAL_MS = 500;  // Read every 500ms

unsigned long lastSensorPublishMillis = 0;
const unsigned long SENSOR_PUBLISH_INTERVAL_MS = 2000;  // Publish every 2s

// ----------------------------------------------------
// Local HTTP Debug Server
// ----------------------------------------------------
WebServer server(9000);

// ----------------------------------------------------
// Forward declarations
// ----------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnect();
void fetchInitialMqttState();
void pollStates();
void pollCoapState();
void handleLocalHttpLed();
void coapResponse(CoapPacket &packet, IPAddress ip, int port);
long readUltrasonicCM();
void processUltrasonicReading();
void publishSensorData(long distance, bool proximity);

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_MQTT, OUTPUT);
  pinMode(LED_COAP, OUTPUT);
  pinMode(LED_HTTP, OUTPUT);
  pinMode(LED_PROXIMITY, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(LED_MQTT, LOW);
  digitalWrite(LED_COAP, LOW);
  digitalWrite(LED_HTTP, LOW);
  digitalWrite(LED_PROXIMITY, LOW);

  // -------- WiFi --------
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println();
      Serial.println("Still connecting... continuing and will retry in loop.");
      break;
    }
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  }

  // -------- MQTT --------
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // -------- Local HTTP Server --------
  server.on("/http_led", handleLocalHttpLed);
  server.begin();
  Serial.println("Local HTTP server started on port 9000");

  // -------- CoAP Client --------
  coapClient.response(coapResponse);
  coapClient.start();
  Serial.println("CoAP client initialized");
  Serial.println("  Will poll coap://kritz.my.id:5683/led");

  Serial.println("Ultrasonic sensor initialized");
  Serial.printf("  Proximity threshold: %ld cm\n", PROXIMITY_THRESHOLD_CM);

  lastPollMillis = millis();
  lastCoapPollMillis = millis();
  lastSensorReadMillis = millis();
  lastSensorPublishMillis = millis();
}

// ----------------------------------------------------
// Loop
// ----------------------------------------------------
void loop() {
  // Check free heap memory periodically
  static unsigned long lastMemCheck = 0;
  if (millis() - lastMemCheck > 10000) {
    lastMemCheck = millis();
    Serial.print("Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
  }

  // MQTT keepalive
  if (!mqttClient.connected()) {
    mqttConnect();
  } else {
    mqttClient.loop();
  }

  // HTTP polling
  if (millis() - lastPollMillis >= POLL_INTERVAL_MS) {
    lastPollMillis = millis();
    pollStates();
  }

  // CoAP polling
  if (millis() - lastCoapPollMillis >= COAP_POLL_INTERVAL_MS) {
    lastCoapPollMillis = millis();
    pollCoapState();
  }

  // Ultrasonic sensor reading
  if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMillis = millis();
    processUltrasonicReading();
  }

  // Local HTTP debug
  server.handleClient();

  // CoAP loop
  coapClient.loop();

  delay(1);
}

// ----------------------------------------------------
// Ultrasonic Sensor Functions
// ----------------------------------------------------
long readUltrasonicCM() {
  // Ensure trigger LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse (timeout ~38 ms)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 38000UL);
  
  if (duration == 0) {
    return -1; // timeout
  }
  
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void processUltrasonicReading() {
  static long lastDistance = -1;
  static bool lastProximity = false;
  
  long distance = readUltrasonicCM();
  
  if (distance > 0) {
    bool proximity = (distance <= PROXIMITY_THRESHOLD_CM);
    
    // Update proximity LED
    digitalWrite(LED_PROXIMITY, proximity ? HIGH : LOW);
    
    // Only print when value changes significantly or proximity state changes
    if (abs(distance - lastDistance) > 2 || proximity != lastProximity) {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" cm");
      if (proximity) {
        Serial.println(" [PROXIMITY TRIGGERED]");
      } else {
        Serial.println();
      }
      
      lastDistance = distance;
      lastProximity = proximity;
    }
    
    // Publish to MQTT periodically
    if (millis() - lastSensorPublishMillis >= SENSOR_PUBLISH_INTERVAL_MS) {
      lastSensorPublishMillis = millis();
      publishSensorData(distance, proximity);
    }
  } else {
    // Timeout - turn off proximity LED
    digitalWrite(LED_PROXIMITY, LOW);
  }
}

void publishSensorData(long distance, bool proximity) {
  if (!mqttClient.connected()) {
    return;
  }
  
  // Create JSON payload
  StaticJsonDocument<128> doc;
  doc["distance"] = distance;
  doc["proximity"] = proximity;
  
  char buffer[128];
  serializeJson(doc, buffer);
  
  // Publish to MQTT
  if (mqttClient.publish(MQTT_TOPIC_SENSOR, buffer)) {
    Serial.print("Published sensor data: ");
    Serial.println(buffer);
  } else {
    Serial.println("Failed to publish sensor data");
  }
}

// ----------------------------------------------------
// MQTT CALLBACK
// ----------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT msg [");
  Serial.print(topic);
  Serial.print("] ");

  if (length == 0) {
    Serial.println("empty");
    return;
  }

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);

  // Only handle LED topic
  if (strcmp(topic, MQTT_TOPIC_LED) == 0) {
    int st = msg.toInt();
    if (st == 1) digitalWrite(LED_MQTT, HIGH);
    else digitalWrite(LED_MQTT, LOW);
    Serial.printf("Set LED_MQTT (D%d) -> %d\n", LED_MQTT, st);
  }
}

// ----------------------------------------------------
// MQTT Connect
// ----------------------------------------------------
void mqttConnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skip MQTT connect");
    return;
  }

  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 2000) return;
  lastAttempt = millis();

  Serial.print("Connecting to MQTT broker ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT connected");
    mqttClient.subscribe(MQTT_TOPIC_LED);
    Serial.print("Subscribed to ");
    Serial.println(MQTT_TOPIC_LED);
    
    // Fetch initial state after connecting
    fetchInitialMqttState();
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ----------------------------------------------------
// Fetch Initial MQTT State
// ----------------------------------------------------
void fetchInitialMqttState() {
  Serial.println("Fetching initial MQTT state...");
  
  WiFiClientSecure* client = new WiFiClientSecure;
  if (!client) {
    Serial.println("Failed to create SSL client");
    return;
  }
  
  client->setInsecure();
  
  HTTPClient https;
  https.begin(*client, STATES_URL);
  https.setTimeout(5000);
  https.setReuse(false);
  int code = https.GET();

  if (code == 200) {
    String payload = https.getString();
    
    const size_t CAPACITY = JSON_OBJECT_SIZE(3) + 60;
    StaticJsonDocument<CAPACITY> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (!err && doc.containsKey("mqtt")) {
      int mqtt_state = doc["mqtt"];
      digitalWrite(LED_MQTT, mqtt_state ? HIGH : LOW);
      Serial.printf("Initial MQTT LED state set to: %d\n", mqtt_state);
    }
  } else {
    Serial.print("Failed to fetch initial state, code=");
    Serial.println(code);
  }

  https.end();
  delete client;
}

// ----------------------------------------------------
// HTTP POLL HANDLER
// ----------------------------------------------------
void pollStates() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, trying reconnect...");
    WiFi.reconnect();
    return;
  }

  WiFiClientSecure* client = new WiFiClientSecure;
  if (!client) {
    Serial.println("Failed to create SSL client");
    return;
  }
  
  client->setInsecure();
  client->setTimeout(5000);

  HTTPClient https;
  https.begin(*client, STATES_URL);
  https.setTimeout(5000);
  https.setReuse(false);
  int code = https.GET();

  if (code == 200) {
    String payload = https.getString();

    const size_t CAPACITY = JSON_OBJECT_SIZE(3) + 60;
    StaticJsonDocument<CAPACITY> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (!err) {
      if (doc.containsKey("http")) {
        int http_state = doc["http"];
        int current = digitalRead(LED_HTTP);
        if ((http_state ? HIGH : LOW) != current) {
          digitalWrite(LED_HTTP, http_state ? HIGH : LOW);
          Serial.printf("HTTP LED changed to: %d\n", http_state);
        }
      }
    } else {
      Serial.print("JSON parse failed: ");
      Serial.println(err.c_str());
    }
  } else if (code > 0) {
    Serial.print("HTTP GET failed, code=");
    Serial.println(code);
  }

  https.end();
  client->stop();
  delete client;
}

// ----------------------------------------------------
// CoAP POLL HANDLER
// ----------------------------------------------------
void pollCoapState() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  static IPAddress serverIP;
  static unsigned long lastDnsLookup = 0;
  
  if (serverIP == IPAddress(0,0,0,0) || millis() - lastDnsLookup > 300000) {
    if (!WiFi.hostByName(COAP_HOST, serverIP)) {
      Serial.println("CoAP: DNS lookup failed");
      return;
    }
    lastDnsLookup = millis();
    Serial.print("CoAP: DNS resolved to ");
    Serial.println(serverIP);
  }

  int msgid = coapClient.get(serverIP, COAP_PORT, "led");
  
  if (msgid == 0) {
    Serial.println("CoAP: Failed to send request");
  }
}

// ----------------------------------------------------
// CoAP RESPONSE HANDLER
// ----------------------------------------------------
void coapResponse(CoapPacket &packet, IPAddress ip, int port) {
  char payload[packet.payloadlen + 1];
  memcpy(payload, packet.payload, packet.payloadlen);
  payload[packet.payloadlen] = '\0';

  int state = atoi(payload);
  
  int current = digitalRead(LED_COAP);
  if ((state ? HIGH : LOW) != current) {
    digitalWrite(LED_COAP, state ? HIGH : LOW);
    Serial.printf("CoAP LED changed to: %d\n", state);
  }
}

// ----------------------------------------------------
// Local HTTP Debug
// ----------------------------------------------------
void handleLocalHttpLed() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "missing state");
    return;
  }

  String s = server.arg("state");
  int st = s.toInt();
  digitalWrite(LED_HTTP, st ? HIGH : LOW);
  server.send(200, "text/plain", "OK");
  Serial.printf("Local HTTP set LED_HTTP (D%d) -> %d\n", LED_HTTP, st);
}