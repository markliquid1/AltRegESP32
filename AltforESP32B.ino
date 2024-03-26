// ***IMPORTANT REMINDER    BATTERIES WILL NOT CHARGE IF ALT Temp <20F or Battery less than 8 Volts***

//Next tasks: 
//Add If statemenets for different sources of the required inputs (ie does battery voltage come from ADS1115, VictronVEDirect, NMEA2K, etc)
//PID control for approaching temperature limit/ psuedocode for overall control strategy
//Test CAN
//Web based interface for editing parameters and displaying sensor data (gauges)


#include <TinyGPSPlus.h> // for NMEA0183 parsing, probably can remove as it has no relevance to alternator
#include "VeDirectFrameHandler.h" // for victron communication
#include <Arduino.h>
#include <Wire.h>
#include "ADS1X15.h"
#include "SSD1306Wire.h" // OLED display                                 
#include <SPI.h> // probably can remove (?) because switched to internal CAN controller
#include <OneWire.h> // temp sensors
#include <DallasTemperature.h> // temp sensors
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "SPIFFS.h" // this adds ability to read configuration setpoints from a text file in FLASH , probably remove this later
#include <WiFi.h> // allow updates over WIFI
#include <esp_wifi.h> // this one is needed for turning wifi "sleep mode" off permanently for better Wifi reliability 
#include <AsyncTCP.h> //allow updates over WIFI
#include <ESPAsyncWebServer.h> . //allow updates over WIFI
#include <ElegantOTA.h> .  //allow updates over WIFI
#include <RTCx.h> // real time clock
#include <INA3221.h> // 3 channel analog input card specialized for BMS      Note that if something not working, i installed 2 versions of this, and both seem to use the same line of code here . "RT" seems to have alarms, regular one no 
#include "SiC45x.h" // Vishay buck converter (heart of system!)

// User Adjustable Variables

// Replace with your network credentials
const char* ssid = "MN2G";
const char* password = "5FENYC8PDW";

int Tthreshold = 165; //over temp threshold (F)
int lowamps = 20; // Amps target on slow charge setting
int highamps = 45; // Amps target on fast charge setting
int Athreshold = 45; // store the selected Amp target ( Clean this up later)
float VthresholdH = 14.3; // over volt threshold
float VthresholdL = 8.0; // under volt threshold
float Vthreshold = 14.3; // battery volts when fully charged   ( Clean this up later)


//Variables used by program.  Some of these can be made local variables at some point if memory an issue
float VictronVoltage = 0;
float VictronCurrent = 0;
float TemperatureF = 0;
float curntsensorValue = 0; // could this be an int?
float Volts = 0;
int voltsensorValue = 0;  // variable to store the value coming from the voltage sensor
int sensorVal = 1; // //ToggleSwitch value for fast/slow charging    Rename later

//CAN pin definitions
#define ESP32_CAN_TX_PIN GPIO_NUM_16 // If you use ESP32 and do not have TX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_17 // If you use ESP32 and do not have RX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
//NMEA0183 setup
TinyGPSPlus gps;  //Object for NMEA0183 reading
// In this section, define custom NMEA sentences
//from B&G Wind
TinyGPSCustom ApparentWindMagnitude(gps, "IIVWR", 3); // $IIVWR sentence, 3rd element
TinyGPSCustom ApparentWindSide(gps, "IIVWR", 2);
TinyGPSCustom ApparentWindAngle(gps, "IIVWR", 1);
//from Simrad Autopilot
TinyGPSCustom Heading(gps, "YDHDG", 1);
TinyGPSCustom RudderAngle(gps, "YDRSA", 1);
//from GPS
TinyGPSCustom SpeedOverGround(gps, "GPRMC", 7);
TinyGPSCustom CourseOverGround(gps, "GPRMC", 8);
//VictronEnergy
VeDirectFrameHandler myve;
SSD1306Wire display(0x3C, SDA, SCL); //give address of the OLED display for i2c
// onewire    Data wire is conntec to the Arduino digital pin 14
#define ONE_WIRE_BUS 14
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
// ADS1115 4 channel analog input card 5V maximum
ADS1115 ADS(0x48);

// Section below for Wifi Updates
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);



void setup()
{
  Serial.begin(115200); //The usual serial monitor
  //Victron
  Serial1.begin(19200, SERIAL_8N1, 16, -1, 1); // ... note the "1" for inversion of logic.  This is the reading of the combined NMEA0183 data from YachtDevices
  Serial2.begin(19200, SERIAL_8N1, 17, -1, 0);  // This is the reading of Victron VEDirect
  Serial2.flush();
  //onewire
  sensors.begin();
  //DISPLAY
  display.init();

  //stuff for loading setpoints from text file
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS, does the text file exist in the right folder?");
    return;
  }
  File file = SPIFFS.open("/XEngAlternatorConfigFile.txt");
  if (!file) {
    Serial.println("Failed to open file for reading");
    Serial.println("Amps Setpoints were HardCoded (Low/High): ");
    Serial.print(lowamps);
    Serial.print('/');
    Serial.println(highamps);
    return;
    while (file.available()) {
      lowamps = file.parseInt();
      Serial.print("LowAmps Setpoint From Text File: ");
      Serial.println(lowamps);
      highamps = file.parseInt();
      Serial.print("HighAmps Setpoint From Text File: ");
      Serial.println(highamps);
    }
    file.close();
  }

  //stuff for updates over WIFI
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE); // disable sleep mode (power save) for wifi
  WiFi.begin(ssid, password);
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Hi! I am ESP32 by X Engineering.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA--- this is the part that enables over the air updates
  server.begin();
  Serial.println("HTTP server started");

}

void loop() {
  ReadNMEA0183(); //continuous NMEA0183 readings from serial port 1
  ReadVEData(); // continuous Victron readings from serial port 2
  ReadSensors(); // less frequently (?) read other sensors
  //PrintData(); //This prints all victron data received to the serial monitor
  ReadDigTempSensor(); // every 5 seconds, take a temperature reading
  SafetyChecks(); // Function to check if all inputs are present and reasonable
  AdjustCurrent(); // Function to adjust altenrator ouput current
  EverySecond(); // Function that happens 1x per second- so far just updating display in normal operation
}

void AdjustCurrent() {
  // Pseudocode
  //Is alternator temperature good?
  // Battery temperature good?
  // RPM a limiting factor?
  // Voltage good?
  //If YES, then set the current target to the Max allowed by user setpoint
  //If NO, set it to a lower value (will need to be chosen individually per the deviation), and notify the display.
}

void EverySecond() {
  static unsigned long prev_millis;

  if (millis() - prev_millis > 1000) {
    //  PrintData(); // this function prints everything available to serial monitor
    prev_millis = millis();

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    //Victron VEDirect
    display.drawString(0, 0, "V-Voltage:");
    display.drawString(90, 0, String(VictronVoltage, 2)); //String(val, decimalPlaces)
    Serial.print("VictronVoltage");
    Serial.print("=  ");
    Serial.println(VictronVoltage);
    display.drawString(0, 11, "V-Current:");
    display.drawString(90, 11, String(VictronCurrent, 1)); //String(val, decimalPlaces)
    display.display();
    //Serial.print("VictronCurrent");
    // Serial.print("=  ");
    // Serial.println(VictronCurrent);

    //Onewire Temp Sensor
    //Serial.print(" - Fahrenheit temperature: ");
    //Serial.println(sensors.getTempFByIndex(0));
    //Serial.print("\AltTemp: "); Serial.print(TemperatureF, 0);
    //Serial.println();

  }
}

void SafetyChecks() {
  if (TemperatureF > Tthreshold) {
    //display.print("OVERHEAT");
    // Serial.println("OVERHEATING");
    //delay(60000); // pause control by 1 minute to allow cooling
  }

  if (TemperatureF < 20) {    // There is probably an issue with Thermistor
    //display.print("TOO COLD");
    // Serial.println("TOO COLD");
    //delay(60000); // pause control by 1 minute then try again
  }

  if (Volts > Vthreshold) {
    //display.print("HIGH VOLTS");
    //Serial.println("VOLTS HIGH");
    //delay(600000); // pause control by 10 minutes
  }

  if (Volts < 8) {
    //display.print("LOW VOLTS");
    //Serial.println("Volts Low");
    //delay(600000); // pause control by 10 minutes
  }

  if (curntsensorValue < -20) {
    //display.print("NO AMPS");    // current sensor is probably broken
    //Serial.println("BROKEN CURRENT SENSOR");
    //delay(60000); // pause control by 1 minutes
  }

}

void ReadDigTempSensor() {
  static unsigned long prev_millis2;

  if (millis() - prev_millis2 > 5000) {
    //  PrintData(); // this function prints everything available to serial monitor
    prev_millis2 = millis();
    //Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
    sensors.requestTemperatures();
    float TemperatureF = sensors.getTempFByIndex(0);
  }
}

void ReadSensors() {      // This section needs some internal IF statements depending on which sensors are available
  // Check which charge mode is selected, either fast or slow
  int sensorVal = digitalRead(A1);
  if (sensorVal == 0) {
    Athreshold = highamps;
  }
  if (sensorVal == 1) {
    Athreshold = lowamps;
  }
  // Read some Sensor Inputs
  int16_t val_1 = ADS.readADC(1);  //Battery Voltage
  float f = ADS.toVoltage(1);  // voltage factor    IS THIS NECESSARY?
  Volts = val_1 * f * 4.0305;    // convert to engineering units
  //Serial.print("\tBattery Voltage: "); Serial.print(Volts, 2);

  int16_t val_2 = ADS.readADC(2);  //Alternator Current
  curntsensorValue = -1 * ((val_2 * f * 100) - 165); // convert to engineering units
  //Serial.print("\tAlternator Current: "); Serial.print(curntsensorValue, 1);

  digitalWrite(3, HIGH); // Turn on voltage supply to Thermistor.  This is done briefly to avoid self-heating
  delay(100);   // Need to adjust this later based on experimentation
  int16_t val_0 = ADS.readADC(0);  // Thermistor
  digitalWrite(3, LOW); // turn off the voltage supply to Thermistor to avoid self heating
  TemperatureF = (((val_0 * f / 5 * 1023 * 64 / 617) + 40) * 9 / 5) + 32; // convert to engineering units
  // first transfer volts back to a simulated "ADC Value" then convert volts to Celcius, then Celcius to F
  // Serial.print("\tThermistor: ");
  //Serial.print(TemperatureF, 1);
  // Serial.println();
}

void ReadNMEA0183() {

  // Every time anything is updated, print everything.
  if (ApparentWindMagnitude.isUpdated() || ApparentWindSide.isUpdated() || ApparentWindAngle.isUpdated() ||
      Heading.isUpdated() || RudderAngle.isUpdated() || SpeedOverGround.isUpdated() ||
      CourseOverGround.isUpdated())
  { //These can all get converted into Floats and then displayed in the OncePerSecond Loop someday

    //    Serial.print(F(" ApparentWindMagnitude=")); Serial.println(ApparentWindMagnitude.value());
    //    Serial.print(F(" ApparentWindSide=")); Serial.println(ApparentWindSide.value());
    //    Serial.print(F(" ApparentWindAngle=")); Serial.println(ApparentWindAngle.value());
    //    Serial.print(F(" Heading=")); Serial.println(Heading.value());
    //    Serial.print(F(" RudderAngle=")); Serial.println(RudderAngle.value());
    //    Serial.print(F(" SpeedOverGround=")); Serial.println(SpeedOverGround.value());
    //    Serial.print(F(" CourseOverGround=")); Serial.println(CourseOverGround.value());

    //OLED Display---All this can get turned on when having 2 good serial ports again
    //    display.clear();
    //    display.setTextAlignment(TEXT_ALIGN_LEFT);
    //    display.setFont(ArialMT_Plain_10);
    //
    //    display.drawString(0, 0, "Wind Speed:");
    //    display.drawString(70, 0, ApparentWindMagnitude.value());
    //
    //    display.drawString(0, 11, "Wind Angle:");
    //    display.drawString(70, 11, ApparentWindAngle.value());
    //    display.drawString(90, 11, ApparentWindSide.value());
    //
    //    display.drawString(0, 22, "Heading:");
    //    display.drawString(70, 22, Heading.value());
    //
    //    display.drawString(0, 33, "COG:");
    //    display.drawString(70, 33, CourseOverGround.value());
    //
    //    display.drawString(0, 44, "Rudder Angle:");
    //    display.drawString(70, 44, RudderAngle.value());
    //
    //    display.drawString(0, 55, "Speed:");
    //    display.drawString(70, 55, SpeedOverGround.value());
    //
    //    display.display();
    //    delay(10);
  }
}

void PrintData() {
  for ( int i = 0; i < myve.veEnd; i++ ) {
    Serial.print(myve.veName[i]);
    Serial.print(i);
    Serial.print("= ");
    Serial.println(myve.veValue[i]);
  }

}

void ReadVEData() {

  while ( Serial2.available() ) {
    myve.rxData(Serial2.read());
    for ( int i = 0; i < myve.veEnd; i++ )
    {
      if (strcmp(myve.veName[i], "V") == 0 )
      {
        VictronVoltage = (atof(myve.veValue[i]) / 1000);
      }
      if (strcmp(myve.veName[i], "I") == 0 )
      {
        VictronCurrent = (atof(myve.veValue[i]) / 1000);
      }
    }
    yield();
  }
}
