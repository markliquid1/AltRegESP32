/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-input-data-html-form/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "MN2G";
const char* password = "5FENYC8PDW";

const char* TLimit = "TemperatureLimitF";
const char* FieldCurrentSetp = "FieldCurrentSetpointAmps";
const char* FullChargeV = "FullChargeVoltage";

// HTML web page to handle 3 input fields (inputString, FieldCurrentSetpointAmps, FullChargeVoltage)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    TemperatureLimitF (current value %TemperatureLimitF%): <input type="text" name="TemperatureLimitF">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    FieldCurrentSetpointAmps (current value %FieldCurrentSetpointAmps%): <input type="number " name="FieldCurrentSetpointAmps">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    FullChargeVoltage (current value %FullChargeVoltage%): <input type="number " name="FullChargeVoltage">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TemperatureLimitF"){
    return readFile(SPIFFS, "/TemperatureLimitF.txt");
  }
  else if(var == "FieldCurrentSetpointAmps"){
    return readFile(SPIFFS, "/FieldCurrentSetpointAmps.txt");
  }
  else if(var == "FullChargeVoltage"){
    return readFile(SPIFFS, "/FullChargeVoltage.txt");
  }
  return String();
}

void setup() {
  Serial.begin(115200);
  // Initialize SPIFFS
  #ifdef ESP32
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI (lower absolute value is better!): ");
  Serial.println(WiFi.RSSI());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputString value on <ESP_IP>/get?inputString=<inputMessage>
    if (request->hasParam(TLimit)) {
      inputMessage = request->getParam(TLimit)->value();
      writeFile(SPIFFS, "/TemperatureLimitF.txt", inputMessage.c_str());
    }
    // GET FieldCurrentSetpointAmps value on <ESP_IP>/get?FieldCurrentSetpointAmps=<inputMessage>
    else if (request->hasParam(FieldCurrentSetp)) {
      inputMessage = request->getParam(FieldCurrentSetp)->value();
      writeFile(SPIFFS, "/FieldCurrentSetpointAmps.txt", inputMessage.c_str());
    }
    // GET FullChargeVoltage value on <ESP_IP>/get?FullChargeVoltage=<inputMessage>
    else if (request->hasParam(FullChargeV)) {
      inputMessage = request->getParam(FullChargeV)->value();
      writeFile(SPIFFS, "/FullChargeVoltage.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  // To access your stored values 
  String yourTemperatureLimitF = readFile(SPIFFS, "/TemperatureLimitF.txt");
  Serial.print("*** Your inputString: ");
  Serial.println(yourTemperatureLimitF);
  
  int yourFieldCurrentSetpointAmps = readFile(SPIFFS, "/FieldCurrentSetpointAmps.txt").toInt();
  Serial.print("*** Your FieldCurrentSetpointAmps: ");
  Serial.println(yourFieldCurrentSetpointAmps);
  
  float yourFullChargeVoltage = readFile(SPIFFS, "/FullChargeVoltage.txt").toFloat();
  Serial.print("*** Your FullChargeVoltage: ");
  Serial.println(yourFullChargeVoltage);
  delay(5000);
}
