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

// ----------------------------------------------------
// MQTT
// ----------------------------------------------------
const char* MQTT_HOST = "kritz.my.id";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC = "kritz/led/mqtt";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ----------------------------------------------------
// HTTP Polling
// ----------------------------------------------------
const char* STATES_URL = "https://kritz.my.id/states";

unsigned long lastPollMillis = 0;
const unsigned long POLL_INTERVAL_MS = 2000;  // Increased to reduce load

// ----------------------------------------------------
// CoAP Client Polling
// ----------------------------------------------------
const char* COAP_HOST = "kritz.my.id";
const uint16_t COAP_PORT = 5683;
WiFiUDP coapUdp;
Coap coapClient(coapUdp);

unsigned long lastCoapPollMillis = 0;
const unsigned long COAP_POLL_INTERVAL_MS = 3000;  // Increased to reduce load

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

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_MQTT, OUTPUT);
  pinMode(LED_COAP, OUTPUT);
  pinMode(LED_HTTP, OUTPUT);

  digitalWrite(LED_MQTT, LOW);
  digitalWrite(LED_COAP, LOW);
  digitalWrite(LED_HTTP, LOW);

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

  lastPollMillis = millis();
  lastCoapPollMillis = millis();
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

  // Local HTTP debug
  server.handleClient();

  // CoAP loop
  coapClient.loop();

  delay(1);
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

  int st = msg.toInt();
  if (st == 1) digitalWrite(LED_MQTT, HIGH);
  else digitalWrite(LED_MQTT, LOW);

  Serial.printf("Set LED_MQTT (D%d) -> %d\n", LED_MQTT, st);
}

// ----------------------------------------------------
// Fetch Initial MQTT State
// ----------------------------------------------------
void fetchInitialMqttState() {
  Serial.println("Fetching initial MQTT state...");
  
  // Create a fresh SSL client for this request
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
    mqttClient.subscribe(MQTT_TOPIC);
    Serial.print("Subscribed to ");
    Serial.println(MQTT_TOPIC);
    
    // Fetch initial state after connecting
    fetchInitialMqttState();
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
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

  // Create a fresh SSL client for each request
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

    // Parse JSON
    const size_t CAPACITY = JSON_OBJECT_SIZE(3) + 60;
    StaticJsonDocument<CAPACITY> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (!err) {
      // Only control HTTP LED from polling
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

  // Serial.println("Polling CoAP state...");
  
  // Resolve hostname to IP (cache this to avoid repeated DNS lookups)
  static IPAddress serverIP;
  static unsigned long lastDnsLookup = 0;
  
  // Only do DNS lookup once every 5 minutes or if IP is not set
  if (serverIP == IPAddress(0,0,0,0) || millis() - lastDnsLookup > 300000) {
    if (!WiFi.hostByName(COAP_HOST, serverIP)) {
      Serial.println("CoAP: DNS lookup failed");
      return;
    }
    lastDnsLookup = millis();
    Serial.print("CoAP: DNS resolved to ");
    Serial.println(serverIP);
  }

  // Send GET request
  int msgid = coapClient.get(serverIP, COAP_PORT, "led");
  
  if (msgid == 0) {
    Serial.println("CoAP: Failed to send request");
  }
}


// ----------------------------------------------------
// CoAP RESPONSE HANDLER
// ----------------------------------------------------
void coapResponse(CoapPacket &packet, IPAddress ip, int port) {
  // Serial.print("CoAP response from ");
  // Serial.print(ip);
  // Serial.print(":");
  // Serial.println(port);

  // Extract payload
  char payload[packet.payloadlen + 1];
  memcpy(payload, packet.payload, packet.payloadlen);
  payload[packet.payloadlen] = '\0';

  // Parse state
  int state = atoi(payload);
  
  // Only update if state changed
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