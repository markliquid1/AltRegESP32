#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

// ESP32 Address 
// 192.168.8.205 


// Replace with your actual WiFi credentials
const char *ssid = "MN2G";
const char *password = "5FENYC8PDW";

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  // Mount LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }

  // Serve index.html at root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  // Start server
  server.begin();
}

void loop() {
  // Nothing needed here
}
