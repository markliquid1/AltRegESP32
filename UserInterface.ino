/*********
  This is the beginning of a user interface to configure and display sensor readings from the Regulator.
  The display readings is done with ServerSentEvents
  The configuration parameters are uploaded with HTTP_GET requests.  They are also stored permanently in SPIFFS so they remain after a power cycle

  Code skeleton came from the below source:
  Rui Santos Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-input-data-html-form/
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

// Create an Event Source on /events
AsyncEventSource events("/events");

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 500;

float AlternatorTemperature;
float FieldDutyCycle;
float BatteryVoltage;

const char* TLimit = "TemperatureLimitF";
const char* FieldCurrentSetp = "FieldCurrentSetpointAmps";
const char* FullChargeV = "FullChargeVoltage";

// HTML web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Alt Regulator by X Engineering</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 1000);   
    }
  </script></head><body>
  <h1><b>Alternator Status and Controls</b></h1>
  <hr/>

    <h2>Configuration Settings</h2>

  <form action="/get" target="hidden-form">
    Alternator Temperature Limit (F) (current value %TemperatureLimitF%): <input type="number " name="TemperatureLimitF">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Field Current Target (A) (current value %FieldCurrentSetpointAmps%): <input type="number " name="FieldCurrentSetpointAmps">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Fully Charged Voltage (V) (current value %FullChargeVoltage%): <input type="number " name="FullChargeVoltage">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
<hr/>

    <h2>Sensor Data and Other Outputs</h2>
 <div class="content">
    <div class="cards">
      <div class="card">
        <p><i class="fas fa-thermometer-half" style="color:#059e8a;"></i> ALTERNATOR TEMPERATURE</p><p><span class="reading"><span id="AltTempID">%ALTERNATORTEMPERATURE%</span> &deg;C</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#00add6;"></i> FIELD DUTY CYCLE</p><p><span class="reading"><span id="FieldDutyCycleID">%FIELDDUTYCYCLE%</span> &percnt;</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-battery-three-quarters" style="color:#e1e437;"></i> BATTERY VOLTAGE</p><p><span class="reading"><span id="BatteryVoltageID">%BATTERYVOLTAGE%</span> V</span></p>
      </div>
    </div>
  </div>
  <hr/>

<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');

 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);

 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);

 source.addEventListener('AlternatorTemperature', function(e) {
  console.log("AlternatorTemperature", e.data);
  document.getElementById("AltTempID").innerHTML = e.data;
 }, false);

 source.addEventListener('FieldDutyCycle', function(e) {
  console.log("FieldDutyCycle", e.data);
  document.getElementById("FieldDutyCycleID").innerHTML = e.data;
 }, false);

 source.addEventListener('BatteryVoltage', function(e) {
  console.log("BatteryVoltage", e.data);
  document.getElementById("BatteryVoltageID").innerHTML = e.data;
 }, false);
}
</script>
</body>
</html>)rawliteral";

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI (lower absolute value is better!): ");
  Serial.println(WiFi.RSSI());
}

void getSensorReadings() {

  AlternatorTemperature = analogRead(34);
  FieldDutyCycle = analogRead(35);
  BatteryVoltage = analogRead(36);
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path) {
 // Serial.printf("Reading file: %s\r\n", path); //UNCOMMENT FOR DEBUGGING
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
 // Serial.println("- read from file:");
  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
 // Serial.println(fileContent);//UNCOMMENT FOR DEBUGGING
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
 // Serial.printf("Writing file: %s\r\n", path);//UNCOMMENT FOR DEBUGGING
  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
  //  Serial.println("- file written");//UNCOMMENT FOR DEBUGGING
  } else {
 //   Serial.println("- write failed");//UNCOMMENT FOR DEBUGGING
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var) {
  getSensorReadings();

  if (var == "ALTERNATORTEMPERATURE") {
    return String(AlternatorTemperature);
  }
  else if (var == "FIELDDUTYCYCLE") {
    return String(FieldDutyCycle);
  }
  else if (var == "BATTERYVOLTAGE") {
    return String(BatteryVoltage);
  }

  else if (var == "TemperatureLimitF") {
    return readFile(SPIFFS, "/TemperatureLimitF.txt");
  }
  else if (var == "FieldCurrentSetpointAmps") {
    return readFile(SPIFFS, "/FieldCurrentSetpointAmps.txt");
  }
  else if (var == "FullChargeVoltage") {
    return readFile(SPIFFS, "/FullChargeVoltage.txt");
  }
  return String();
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
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
   // Serial.println(inputMessage);  //UNCOMMENT FOR DEBUGGING
   
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient * client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void loop() {
  // To access your stored values
  int yourTemperatureLimitF = readFile(SPIFFS, "/TemperatureLimitF.txt").toInt();
  int yourFieldCurrentSetpointAmps = readFile(SPIFFS, "/FieldCurrentSetpointAmps.txt").toInt();
  float yourFullChargeVoltage = readFile(SPIFFS, "/FullChargeVoltage.txt").toFloat();
  if ((millis() - lastTime) > timerDelay) {
    getSensorReadings();
    // Send Events to the Web Client with the Sensor Readings
    events.send("ping", NULL, millis());
    events.send(String(AlternatorTemperature).c_str(), "AlternatorTemperature", millis());
    events.send(String(FieldDutyCycle).c_str(), "FieldDutyCycle", millis());
    events.send(String(BatteryVoltage).c_str(), "BatteryVoltage", millis());

    lastTime = millis();


    // delay(100);
  }

}
