// ESP32 + DFPlayer Mini, triggered by the delivery API over wifi.
//
// The Pi (audio_node) POSTs a completion when the robot reaches its goal.
// This sketch polls the same endpoint with GET every 30 s and plays a track
// and plays a track when the newest eventId (first item of "data")
// differs from the last one seen. The first poll after boot only records
// the current eventId, so an old event never plays.
//
// Wiring (Serial2):
//   ESP32 GPIO17 (TX2) -> 1k resistor -> DFPlayer RX
//   ESP32 GPIO16 (RX2) <- DFPlayer TX
//   DFPlayer VCC 5V, GND common. Speaker on SPK1/SPK2.
//   SD card (FAT32) with files 0001.mp3, 0002.mp3, ... in the root or /mp3.
//
// Library: "DFRobotDFPlayerMini". Any ESP32 board works (wifi only).

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DFRobotDFPlayerMini.h>

const char* WIFI_SSID = "Ngoi nha vui ve";
const char* WIFI_PASS = "06011997";

const char* API_URL =
  "https://34.172.236.168:8081/api/v1/robots/MEDIBOT-01/deliveries/completions";
const char* API_TOKEN = "";  // sent as "Authorization: Bearer ..." if not empty

const unsigned long POLL_MS = 000;  // 30 s
const int TRACK = 1;
const int VOLUME = 25;  // 0..30

DFRobotDFPlayerMini player;
bool playerReady = false;
String lastEventId;
bool initialized = false;
unsigned long lastPoll = 0;

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.print("\nIP: ");
  Serial.println(WiFi.localIP());
}

// First "eventId" value of the body (the newest event), "" if none
String firstEventId(const String& body) {
  const String key = "\"eventId\":\"";
  int start = body.indexOf(key);
  if (start < 0) return "";
  start += key.length();
  int end = body.indexOf('"', start);
  if (end < 0) return "";
  return body.substring(start, end);
}

void poll() {
  WiFiClientSecure client;
  client.setInsecure();  // self signed cert on a bare IP: no verification

  HTTPClient http;
  http.setTimeout(4000);
  if (!http.begin(client, API_URL)) return;
  if (strlen(API_TOKEN) > 0) {
    http.addHeader("Authorization", String("Bearer ") + API_TOKEN);
  }

  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    String eventId = firstEventId(body);
    if (eventId.length() == 0) {
      Serial.println("no eventId: " + body);
    } else if (!initialized) {
      lastEventId = eventId;  // init: remember, do not play
      initialized = true;
      Serial.println("init eventId " + eventId);
    } else if (eventId != lastEventId) {
      lastEventId = eventId;
      Serial.println("new event " + eventId + " -> play");
      if (playerReady) player.play(TRACK);
    } else {
      Serial.println("same event: " + body);
    }
  } else {
    Serial.println("poll failed, HTTP " + String(code));
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2=16, TX2=17

  playerReady = player.begin(Serial2);
  if (playerReady) {
    player.volume(VOLUME);
  } else {
    Serial.println("DFPlayer init failed (check wiring / SD card)");
  }

  connectWifi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(1000);
    return;
  }
  if (lastPoll == 0 || millis() - lastPoll >= POLL_MS) {
    lastPoll = millis() | 1;  // never 0, so the first poll is immediate
    poll();
  }
}
