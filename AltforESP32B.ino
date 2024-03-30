// Open Source Alternator Regulator by X Engineering
// contact: Mark@Xengineering.net
// https://github.com/markliquid1/AltRegESP32
// wwww.xengineering.net
// March 30, 2024
// GPL-3.0 License

// ***IMPORTANT REMINDER    BATTERIES WILL NOT CHARGE IF ALT Temp <20F or Battery less than 8 Volts ***

//Next tasks:
//PID control for approaching temperature limit/ psuedocode for overall control strategy

//---------------LIBRARIES------------------------------------------------------------------------------------------------------------------------
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
#include "SPIFFS.h" // this adds ability to read configuration setpoints from a text file in FLASH 
#include <WiFi.h> // allow updates over WIFI
#include <esp_wifi.h> // this one is needed for turning wifi "sleep mode" off permanently for better Wifi reliability 
#include <AsyncTCP.h> //allow updates over WIFI
#include <ESPAsyncWebServer.h> . //allow updates over WIFI
//#include <ElegantOTA.h> .  //allow program re-flashing over WIFI-- currently not working due to program being >50% size of available memory.  Plus this is a big library.
#include <RTCx.h> // real time clock
#include <INA3221.h> // 3 channel analog input card specialized for BMS      Note that if something not working, i installed 2 versions of this, and both seem to use the same line of code here . "RT" seems to have alarms, regular one no 
#include "SiC45x.h" // Vishay buck converter (heart of system!)

//------------User Adjustable Constants---------------------------------------------------------------------------------------------------------------------

// Network Credentials        update to match your local wifi network
const char* ssid = "MN2G";
const char* password = "5FENYC8PDW";

int Tthreshold = 165; //Temperature at which the alternator will scale back (F)
int lowamps = 20; // Amps target on slow charge setting
int highamps = 45; // Amps target on fast charge setting
int Athreshold = 45; // store the selected Amp target ( Clean this up later)
float VthresholdH = 14.3; // over volt threshold
float VthresholdL = 8.0; // under volt threshold

bool VeDataOn = 0; // If you are using VeDirect set this to 1
bool VeDataforVoltageOn = 0; //and want to use it as the source of Battery Voltage
bool VeDataforCurrentOn = 0; //and want to use it as the source of Battery Current

bool NMEA2KOn = 0; // If you are using a NMEA2K network  set to 1
bool NMEA2KVoltageOn = 0; //and want to use it as the source of Battery Voltage
bool NMEA2KVCurrentOn = 0; //and want to use it as the source of Battery Current

//----------------Variables used by program.  Some of these can be made local variables at some point if memory an issue--------------------------------------------

//Battery Voltage
float VictronVoltage = 0; // battery reading from VeDirect
float INA3221Volts = 0; // battery reading from INA3221
float ADS1115Volts = 0; // battery reading from ADS1115
float BatteryVoltage = 0; //This will be the chosen one we control off of

//Alternator Current and RPM
float VictronCurrent = 0;// current as measured by victron
float ADS1115Current = 0; // current as measured by ADS1115
float curntValue = 0; //  This will be the chosen one we control off of
int AltRPM=0; // alternator RPM

//Battery Current
float INA3221Shunt = 0;

//Temperature Related
float AlternatorTemperature = 0;

//Various Switches
bool SwitchSource = 0; // Set to zero give physical hardware switches control.  Set to 1 to give web interface control
//HardwareInputs
bool HiLowMode = 0; 
bool LimpHomeMode=0;
bool ForceFloat=0;
bool KillCharge=0;
bool KillChargeFromExternalBMS=0;

//-------------//Misc pre-setup code that don't need frequent changes----------------------------------------------------------------------------------------------------
//CAN Tx/Rx pin #'s on ESP32 GPIO
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

//Wifi and User Interface related:
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 1000; // update the web display 1x per second



//-------------The usual setup() and loop() functions----------------------------------------------------------------------------------------------------
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
  esp_wifi_set_ps(WIFI_PS_NONE); // make sure wifi doesn't go into low power mode
}

void loop() {

  ReadSensors(); // 
  GetSetUp(); // misc things that need to be done 
  SafetyChecks(); // Check if all inputs are present and reasonable
  AdjustCurrent(); // Adjust altenrator ouput current setpoint based on a number of factors
  UpdateDisplay(); // 
}

//--------------------FUNCTIONS--------------------------------------------------------------------------------------------------------
void ReadSensors() {        
  ReadNMEA2K();//read data from NMEA2K network CAN connection
  ReadVEData();//read Data from Victron VeDirect
  ReadNMEA0183(); //read NMEA0183 readings from serial port 1
  ReadDigTempSensor();//read theOneWire sensor
  ReadVEData(); // read Victron readings from serial port 2
  ReadINA3221(); // reac voltage from INA3221
  ReadADS1115(); // read voltage readings from ADS1115
  ReadSwitches();// poll the various on/off switches
  ReadRPM();// read engine speed from W post on alternator (Stator)
}
void GetSetUp() {
  //This function takes care of figuring out some preliminary info
  if (HiLowMode == 0) {
    Athreshold = highamps;
  }
  if (HiLowMode == 1) {
    Athreshold = lowamps;
  }

  BatteryVoltage = 14; //This will be the one we control off of

}
void SafetyChecks() {
  if (AlternatorTemperature > Tthreshold) {
    //display.print("OVERHEAT");
    // Serial.println("OVERHEATING");
    //delay(60000); // pause control by 1 minute to allow cooling
  }

  if (AlternatorTemperature < 20) {    // There is probably an issue with Thermistor
    //display.print("TOO COLD");
    // Serial.println("TOO COLD");
    //delay(60000); // pause control by 1 minute then try again
  }

  if (BatteryVoltage > VthresholdH) {
    //display.print("HIGH VOLTS");
    //Serial.println("VOLTS HIGH");
    //delay(600000); // pause control by 10 minutes
  }

  if (BatteryVoltage < VthresholdL) {
    //display.print("LOW VOLTS");
    //Serial.println("Volts Low");
    //delay(600000); // pause control by 10 minutes
  }

  if (curntValue < -20) {
    //display.print("NO AMPS");    // current sensor is probably broken
    //Serial.println("BROKEN CURRENT SENSOR");
    //delay(60000); // pause control by 1 minutes
  }
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
void UpdateDisplay() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {
    //  PrintData(); // this function prints everything available to serial monitor
    prev_millis = millis();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "V-Voltage:");
    display.drawString(90, 0, String(VictronVoltage, 2)); //String(val, decimalPlaces)
    Serial.print("BatteryV");
    Serial.print("=  ");
    Serial.println(BatteryVoltage);
    display.drawString(0, 11, "ALternatorC:");
    display.drawString(90, 11, String(curntValue, 1)); //String(val, decimalPlaces)
    display.display();
  }
}
void ReadNMEA2K() {
  if (NMEA2KOn == 1) {
    //Insert Code Here
  }
}
void ReadVEData() {
  if (VeDataOn == 1) {
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
    //PrintData(); //This prints all victron data received to the serial monitor, put this in "Loop" for debugging if needed
  }
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
void ReadDigTempSensor() {
  static unsigned long prev_millis2;

  if (millis() - prev_millis2 > 5000) {
    //  PrintData(); // this function prints everything available to serial monitor
    prev_millis2 = millis();
    //Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
    sensors.requestTemperatures();
    float AlternatorTemperature = sensors.getTempFByIndex(0);
  }
}
void ReadINA3221() {
  INA3221Volts = 0; // battery reading from INA3221
  INA3221Shunt = 0; // shunt current reading
}
void ReadADS1115() {
  ADS1115Volts = 0; // battery reading from ADS1115
  ADS1115Current = 0;
}
void ReadSwitches() {
    HiLowMode = digitalRead(A1); // Check which charge mode is selected, either fast or slow
    LimpHomeMode=0;
    ForceFloat=0;
    KillCharge=0;
    KillChargeFromExternalBMS=0;
}
void ReadRPM() {
  AltRPM=0;
}
void PrintData() {
  for ( int i = 0; i < myve.veEnd; i++ ) {
    Serial.print(myve.veName[i]);
    Serial.print(i);
    Serial.print("= ");
    Serial.println(myve.veValue[i]);
  }

}





///
