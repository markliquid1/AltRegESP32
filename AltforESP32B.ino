// Open Source Alternator Regulator by X Engineering
// contact: Mark@Xengineering.net
// https://github.com/markliquid1/AltRegESP32
// wwww.xengineering.net
// March 30, 2024
// GPL-3.0 License

// ***IMPORTANT REMINDER    BATTERIES WILL NOT CHARGE IF Alt Temp <20F or Battery less than 8 Volts without Override ***

//Next tasks:
//PID Control 

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
//#include <ElegantOTA.h> .  //allow program re-flashing over WIFI-- currently not implemented but was working at one point in a differnet file
#include <RTCx.h> // real time clock
#include <INA3221.h> // 3 channel analog input card specialized for BMS      Note that if something not working, i installed 2 versions of this, and both seem to use the same line of code here . "RT" seems to have alarms, regular one no 
#include "SiC45x.h" // Vishay buck converter (heart of system!)

//------------User Adjustable Constants---------------------------------------------------------------------------------------------------------------------

// Network Credentials        update to match your local wifi network
const char* ssid = "MN2G";
const char* password = "5FENYC8PDW";

int Tthreshold = 165; //Temperature at which the alternator will scale back (F)
int lowamps = 20; // Amps target on "slow" charge setting
int highamps = 45; // Amps target on "fast" charge setting

float VthresholdH = 14.3; // over volt threshold, stop charging above here
float VthresholdL = 8.0; // under volt threshold, prevent charging below here

bool VeDataOn = 0; // If you are using VeDirect for anything set this to 1
bool VeDataforVoltageOn = 0; //and want to use it as the source of Battery Voltage
bool VeDataforCurrentOn = 0; //and want to use it as the source of Battery Current

bool NMEA2KOn = 0; // If you are using NMEA2K for anhything set this to 1
bool NMEA2KBatteryVoltageOn = 0; //If you are using a NMEA2K network and want to use it as the source of Battery Voltage
bool NMEA2KCurrentOn = 0; //If you are using a NMEA2K network and want to use it as the source of Battery Current

bool INA3221BatteryCurrentOn = 0; // If you want to make use of a battery shunt connected to the ESP32/INA3221 chip

bool LM2907RPMOn  = 0; // if you want to use alternator calculated RPM for any reason (such as broken belt detector or RPM based charge table), set to 1
bool NMEA2KRPMOn = 0; // same deal but for RPM from NMEA2K network

bool SwitchSource = 0; // Set to zero give physical hardware switches control.  Set to 1 to give web interface control


//----------------Variables used by program.  Some of these can be made local variables at some point if memory an issue--------------------------------------------

//Battery Voltage
float VictronVoltage = 0; // battery reading from VeDirect
float INA3221Volts = 0; // battery reading from INA3221
float ADS1115Volts = 0; // battery reading from ADS1115
float NMEA2KBatteryVoltage = 0; // battery reading from NMEA2K
float BatteryVoltage = 0; //This will be the chosen one we control off of

//Alternator Current and RPM
int AmpTarget; // store the selected Amp target- after all checks, this is the final target setpoint
float VictronCurrent = 0;// battery current as measured by victron
float ADS1115Current = 0; // Alternator current as measured by ADS1115
float NMEA2KCurrent = 0; // Battery current as measured by NMEA2K
float INA3221BatteryCurrent=0; // Battery current as measured by ESP32/INA3221 chip
float curntValue = 0; //  This will be the chosen alternator current we control off of
int LM2907RPM = 0; //  RPM from built in frequency converter
int NMEA2KRPM = 0; //  RPM from NMEA2K
int RPM = 0; // primary RPM reading we control off of

//Battery Current
float BatteryCurrent = 0; // final one used for control in the case of a battery based goal


//Temperature Related
float AlternatorTemperature = 0;


//Used to make sure engine is spinning if going to keep supplying field current by function EngienOfforSlipBeltCheck()
unsigned long start_millis;
bool ThisIsTheFirstTime = 1;


//HardwareSwitches
bool HiLowMode = 0; // Select high or low amps to target.  High can be used for maximum charge rate, while low can be general use when Solar is generall enough
bool LimpHomeMode = 0; // DANGER: This mode will override having a valid temperature sensor, voltage reading, current reading, etc, and do the best it can to charge at a low field current of your choice (hard coded)
bool ForceFloat = 0; // In this mode, the alterantor will target Zero amps (or a value of your choice, hard-coded) at the battery shunt
bool KillCharge = 0; // This switch is used to manually turn off the field (stop charging)
bool KillChargeFromExternalBMS = 0; // This is the same functionality as above, but originating from an outside source such as 3rd party BMS

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
  AreTheReadingsRealistic(); // Check if all desired inputs are present and reasonable
  EngienOfforSlipBeltCheck();
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
    AmpTarget = highamps;
  }
  if (HiLowMode == 1) {
    AmpTarget = lowamps;
  }
  if (VeDataforVoltageOn == 1) {
    BatteryVoltage = VictronVoltage;
  }
  else if (NMEA2KBatteryVoltageOn) {
    BatteryVoltage = NMEA2KBatteryVoltage;
  }
  else {
    BatteryVoltage = INA3221Volts;
  }
  if (VeDataforCurrentOn == 1) {
    curntValue = VictronCurrent;
  }
  else if (NMEA2KCurrentOn == 1) {
    curntValue = NMEA2KCurrent;
  }
  else {
    curntValue = ADS1115Current;
  }
  if (NMEA2KRPMOn == 1) {
    RPM = NMEA2KRPM;
  }
  else {
    RPM = LM2907RPM;
  }
  if (NMEA2KCurrentOn == 1) {
    BatteryCurrent = NMEA2KCurrent;
  }
  if (INA3221BatteryCurrentOn == 1) {
    BatteryCurrent = INA3221BatteryCurrent;
  }
}
void AreTheReadingsRealistic() {    //*may need to prevent excessive display updates in this section with a "display error message" function?
  if (AlternatorTemperature > 350 || AlternatorTemperature < 20) {
    //Temperature out of range
  }
  if (BatteryVoltage > VthresholdH || BatteryVoltage < VthresholdL) {
    //
  }
  if (curntValue < -20 || curntValue > 500) {
    //
  }
  if (BatteryCurrent < -500 || BatteryCurrent > 500) {
    //
  }
}
void EngienOfforSlipBeltCheck() {
  if (LM2907RPMOn == 1 && NMEA2KRPMOn == 1) {// % if the sensors are said to be present
    if (abs(LM2907RPM - NMEA2KRPM) > 500) { // if they differ by more than 500 RPM
      // Sound alarm 1x every 5 seconds
      // Alert display that belt is slipping
    }
    if (LM2907RPM < 300) {
      //Alert display that the engine is not spinning according to LM2907 chip
      AmpTarget = 0; // refuse to charge
    }
    if (NMEA2KRPM < 300) {
      //Alert display that the engine is not spinning according to NMEA2K
      AmpTarget = 0; // refuse to charge
    }
  }
  if (LM2907RPMOn == 0 && NMEA2KRPMOn == 1) {
    if (NMEA2KRPM < 300) {
      //Alert display that the engine is not spinning according to NMEA2K
      AmpTarget = 0; // refuse to charge
    }
  }
  if (LM2907RPMOn == 1 && NMEA2KRPMOn == 0) { // if the sensors are said to be present
    if (LM2907RPM < 300) {
      //Alert display that the engine is not spinning according to LM2907 chip
      AmpTarget = 0; // refuse to charge
    }
  }
  // In all cases, if there is a field and no measured amps for more than 100ms, then the engine is not spinning, and a 10 second delay is warranted
  if (AmpTarget > 10 && curntValue < 5) {
    if (ThisIsTheFirstTime == 1) { //if this is the first time finding a strange lack of charging
      static unsigned long start_millis = millis(); // start timer
      ThisIsTheFirstTime = 0; // the timer has been started, it's not the first time, we're counting time now
    }
    if ( millis() - start_millis > 100) { // if the condition does not self-correct within 100 milliseconds (ie current goes up, or setpoint goes down)
      //report to display              //*may need to prevent this from happening too frequently with a "display error message" function?
      AmpTarget = 0; // refuse to charge
    }
    if (millis() - start_millis > 10000) { // if it has been more than 10 seconds
      AmpTarget = 15; //try charging again
      ThisIsTheFirstTime = 1; // and get ready to re-check for output
    }
  }
  else {   // if the condition corrected itself, prepare to restart the timer next time the discrepancy is noticed again.
    ThisIsTheFirstTime = 1;
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

//Called from function "ReadSensors()"
void ReadNMEA2K() {
  if (NMEA2KOn == 1) {
    if (NMEA2KRPMOn == 1 ) {
      //Code
    }
    if (NMEA2KBatteryVoltageOn == 1 ) {
      //Code
    }
    if (NMEA2KCurrentOn == 1) {
      //Code
    }
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
  INA3221Volts = 0; // battery voltage reading from INA3221
  INA3221BatteryCurrent = 0; // shunt current reading (battery current)
}
void ReadADS1115() {
  ADS1115Volts = 0; // battery reading from ADS1115
  ADS1115Current = 0;
}
void ReadSwitches() {
  if (SwitchSource == 0) { // if this is set to 1, you can only make changes thru the Web Interface, besides Limp Home Mode, which is always allowed
    HiLowMode = digitalRead(A1); // Check which charge mode is selected, either fast or slow
    ForceFloat = digitalRead(A3);
    KillCharge = digitalRead(A4);
    KillChargeFromExternalBMS = digitalRead(A5);
  }
  LimpHomeMode = digitalRead(A2);
}
void ReadRPM() {
  RPM = 0;
}
void PrintData() { //This prints all victron data received to the serial monitor, put this in "Loop" for debugging if needed
  for ( int i = 0; i < myve.veEnd; i++ ) {
    Serial.print(myve.veName[i]);
    Serial.print(i);
    Serial.print("= ");
    Serial.println(myve.veValue[i]);
  }

}





///
