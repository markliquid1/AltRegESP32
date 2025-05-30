// X Engineering Alternator Regulator
//     Copyright (C) 2025  Mark Nickerson

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//  See <https://www.gnu.org/licenses/> for GNU General Public License

// Contact me at mark@xengineering.net

#include <OneWire.h>            // temp sensors
#include <DallasTemperature.h>  // temp sensors
#include <SPI.h>                // display
#include <Wire.h>               // unknown if needed, but probably
#include <Adafruit_GFX.h>       // display
#include <Adafruit_SSD1306.h>   // display
#include <ADS1115_lite.h>       // measuring 4 analog inputs
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);
#include "VeDirectFrameHandler.h"  // for victron communication
#include "INA228.h"
INA228 INA(0x40);
//DONT MOVE THE NEXT 6 LINES AROUND, MUST STAY IN THIS ORDER
#include <Arduino.h>                  // maybe not needed, was in NMEA2K example I copied
#define ESP32_CAN_RX_PIN GPIO_NUM_16  //
#define ESP32_CAN_TX_PIN GPIO_NUM_17  //
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>  // questionably needed
#include <WiFi.h>
#include <AsyncTCP.h>                    // for wifi stuff, important, don't ever update, use mathieucarbou github repository
#include <LittleFS.h>                    // for wifi stuff
#include <ESPAsyncWebServer.h>           // for wifi stuff, important, don't ever update, use mathieucarbou github repository
#include <DNSServer.h>                   // For captive portal (Wifi Network Provisioning) functionality
#include <ESPmDNS.h>                     // helps with wifi provisioning (to save users trouble of looking up ESP32's IP address)
#include "esp_heap_caps.h"               // needed for tracking heap usage
#include "freertos/FreeRTOS.h"           // for stack usage
#include "freertos/task.h"               // for stack usage
#define configGENERATE_RUN_TIME_STATS 1  // for CPU use tracking
#include <mbedtls/md.h>                  // security
#include <vector>                        // Console message queue system
#include <String>                        // Console message queue system
#include "esp_task_wdt.h"                //Watch dog to prevent hung up code from wreaking havoc
#include "esp_log.h"                     // get rid of spam in serial monitor


// Settings - these will be moved to LittleFS
const char *default_ssid = "MN2G";            // Default SSID if no saved credentials
const char *default_password = "5FENYC8ABC";  // Default password if no saved credentials // 5FENYC8ABC
// WiFi connection timeout when trying to avoid Access Point Mode (and connect to ship's wifi on reboot)
const unsigned long WIFI_TIMEOUT = 20000;  // 20 seconds

//Security
char requiredPassword[32] = "admin";  // Max password length = 31 chars     Password for access to change settings from browser
char storedPasswordHash[65] = { 0 };

// ===== HEAP MONITORING =====
int rawFreeHeap = 0;      // in bytes
int FreeHeap = 0;         // in KB
int MinFreeHeap = 0;      // in KB
int FreeInternalRam = 0;  // in KB
int Heapfrag = 0;         // 0–100 %, integer only

// ===== TASK STACK MONITORING =====
const int MAX_TASKS = 20;  // Adjust if you're running lots of tasks
TaskStatus_t taskArray[MAX_TASKS];
// Optional: reuse these across calls if needed
int numTasks = 0;
int tasksCaptured = 0;
int stackBytes = 0;
int core = 0;

//CPU
// ===== CPU LOAD TRACKING =====
unsigned long lastIdle0Time = 0;  // Previous IDLE0 task runtime counter
unsigned long lastIdle1Time = 0;  // Previous IDLE1 task runtime counter
unsigned long lastCheckTime = 0;  // Last time CPU load was measured
int cpuLoadCore0 = 0;             // CPU load percentage for Core 0
int cpuLoadCore1 = 0;             // CPU load percentage for Core 1

//Console
std::vector<String> consoleMessageQueue;
unsigned long lastConsoleMessageTime = 0;
const unsigned long CONSOLE_MESSAGE_INTERVAL = 2000;  // 60 seconds

// DNS Server for captive portal
DNSServer dnsServer;
const byte DNS_PORT = 53;

// WiFi modes
enum WiFiMode {
  AWIFI_MODE_CLIENT,
  AWIFI_MODE_AP
};

WiFiMode currentWiFiMode = AWIFI_MODE_CLIENT;

//delete this later ?
int INADisconnected = 0;
int WifiHeartBeat = 0;
int previousMillisBLINK = 0;  // used for heartbeat blinking LED
int intervalBLINK = 1000;     // used for heartbeat blinking LED interval
bool ledState = false;        // used for heartbeat blinking LED state
int VeTime = 0;
int SendWifiTime = 0;
int AnalogReadTime = 0;   // this is the present
int AnalogReadTime2 = 0;  // this is the maximum ever

//Input Settings
int TargetAmps = 40;   //Normal alternator output, for best performance, set to something that just barely won't overheat
int TargetAmpLA = 25;  //Alternator output in Lo mode
int uTargetAmps = 0;   // the one that gets set to either the normal setting or the low setting then used.

float TargetFloatVoltage = 13.9;
float TargetBulkVoltage = 14.5;
float ChargingVoltageTarget = 0;  // This is what the code really uses. It gets set to TargetFloatVoltage or TargetBulkVoltage later on

float dutyCycle = 5;
float dutyStep = 0.8;  // duty step to adjust field target by, each time the loop runs.  Larger numbers = faster response, less stability
float MaxDuty = 99.0;
float MinDuty = 18.0;
int ManualDutyTarget = 4;  // example manual override value


float FieldAdjustmentInterval = 500;      // The regulator field output is updated once every this many milliseconds
float MinimumFieldVoltage = 1;            // A min value here ensures that engine speed can be measured even with no alternator output commanded.  (This is only enforced when Ignition input is high)
float AlternatorTemperatureLimitF = 150;  // the offset appears to be +40 to +50 to get true max alternator external metal temp, depending on temp sensor installation, so 150 here will produce a metal temp ~200F
int ManualFieldToggle = 1;                // set to 1 to enable manual control of regulator field output, helpful for debugging
//float ManualVoltageTarget = 5;            // voltage target corresponding to the toggle above
int SwitchControlOverride = 1;  // set to 1 for web interface switches to override physical switch panel
int ForceFloat = 0;             // set to 1 to force the float voltage to be the charging target
int OnOff = 0;                  // 0 is charger off, 1 is charger On (corresponds to Alternator Enable in Basic Settings)
int Ignition = 1;               // This will eventually be an over-rideable digital input
int HiLow = 1;                  // 0 will be a low setting, 1 a high setting
int LimpHome = 0;               // 1 will set to limp home mode, whatever that gets set up to be
int resolution = 12;            // for OneWire temp sensor measurement
int VeData = 0;                 // Set to 1 if VE serial data exists
int NMEA0183Data = 0;           // Set to 1 if NMEA serial data exists doesn't do anything yet
int NMEA2KData = 0;             // doesn't do anything yet
//Field PWM stuff
float interval = 0.8;   // larger value = faster response but more unstable
float vout = 1;         // needs deleting
const int pwmPin = 32;  // field PWM
//const int pwmChannel = 0;      //0–7 available for high-speed PWM  (ESP32)
const int pwmResolution = 16;  // 16-bit resolution (0–65535)
float fffr = 500;              // this is the switching frequency in Hz, uses Float for wifi reasons
int InvertAltAmps = 1;         // change sign of alternator amp reading
int InvertBattAmps = 0;        // change sign of battery amp reading
uint32_t Freq = 0;  // ESP32 switching Frequency in case we want to report it for debugging

//Variables to store measurements
float ShuntVoltage_mV;                  // Battery shunt voltage from INA228
float Bcur;                             // battery shunt current from INA228
float IBV;                              // Ina 228 battery voltage
float IBVMax = NAN;                     // used to track maximum battery voltage
float DutyCycle;                        // Field outout %--- this is just what's transmitted over Wifi (case sensitive)
float FieldResistance = 2;              // Field resistance in Ohms usually between 2 and 6 Ω, changes 10-20% with temp
float vvout;                            // Calculated field volts (approximate)
float iiout;                            // Calculated field amps (approximate)
float AlternatorTemperatureF = NAN;     // alternator temperature
float MaxAlternatorTemperatureF = NAN;  // maximum alternator temperature
TaskHandle_t tempTaskHandle = NULL;     // make a separate cpu task for temp reading because it's so slow
float VictronVoltage = 0;               // battery reading from VeDirect
float HeadingNMEA = 0;                  // Just here to test NMEA functionality

// ADS1115
int16_t Raw = 0;
float Channel0V, Channel1V, Channel2V, Channel3V;
float BatteryV, MeasuredAmps, RPM;  //Readings from ADS1115
//float voltage;                      // This is the one that gets populated by dropdown menu selection (battery voltage source)
float MeasuredAmpsMax;  // used to track maximum alternator output
float RPMMax;           // used to track maximum RPM
int ADS1115Disconnected = 0;

// Battery SOC Monitoring Variables
int BatteryCapacity_Ah = 300;         // Battery capacity in Amp-hours
int SoC_percent = 7500;               // State of Charge percentage (0-100) but have to multiply by 100 for annoying reasons, but go with it
int ManualSOCPoint = 25;              // Used to set it manually
int CoulombCount_Ah_scaled = 7500;    // Current energy in battery (Ah × 100 for precision)
bool FullChargeDetected = false;      // Flag for full charge detection
unsigned long FullChargeTimer = 600;  // Timer for full charge detection, 10 minutes
// Timing variables
unsigned long currentTime = 0;
unsigned long elapsedMillis = 0;
unsigned long lastSOCUpdateTime = 0;      // Last time SOC was updated
unsigned long lastEngineMonitorTime = 0;  // Last time engine metrics were updated
unsigned long lastDataSaveTime = 0;       // Last time data was saved to LittleFS
int SOCUpdateInterval = 2000;             // Update SOC every 2 seconds.   Don't make this smaller than 1 without study
int DataSaveInterval = 300000;            // Save data every 5 minutes (300,000 ms).  Flash will hit 20K cycle end of life in 70 years, good enough!
// Accumulators for runtime tracking
unsigned long engineRunAccumulator = 0;     // Milliseconds accumulator for engine runtime
unsigned long alternatorOnAccumulator = 0;  // Milliseconds accumulator for alternator runtime


//Momentary Buttons
int FactorySettings = 0;  // Reset Button
int AlarmTest = 0;

//More Settings
// SOC Parameters
int CurrentThreshold_scaled = 100;  // Ignore currents below this (A × 100)
int PeukertExponent_scaled = 105;   // Peukert exponent × 100 (112 = 1.12)
int ChargeEfficiency_scaled = 99;   // Charging efficiency % (0-100)
int ChargedVoltage_scaled = 1450;   // Voltage threshold for "charged" (V × 100)
int TailCurrent_scaled = 2000;      // Current threshold for "charged" (% of capacity × 100)
int ShuntResistanceMicroOhm = 100;  // Shunt resistance in microohms
int ChargedDetectionTime = 3600;    // Time at charged state to consider 100% (seconds)
int IgnoreTemperature = 0;          // If no temp sensor, set to 1
int BMSlogic = 0;                   // if BMS is asked to turn the alternator on and off
int BMSLogicLevelOff = 0;           // set to 0 if the BMS gives a low signal (<3V?) when no charging is desired
bool chargingEnabled;               // defined from other variables
bool bmsSignalActive;               // Read from digital input pin 36
int AlarmActivate = 0;              // set to 1 to enable alarm conditions
int TempAlarm = 0;                  // above this value, sound alarm
int VoltageAlarmHigh = 0;           // above this value, sound alarm
int VoltageAlarmLow = 0;            // below this value, sound alarm
int CurrentAlarmHigh = 0;           // above this value, sound alarm
int MaximumAllowedBatteryAmps;      // safety for battery, optional
int FourWay = 0;                    // 0 voltage data source = INA228 , 1 voltage source = ADS1115, 2 voltage source = NMEA2k, 3 voltage source = Victron VeDirect
int RPMScalingFactor = 2000;        // self explanatory, adjust until it matches your trusted tachometer
float AlternatorCOffset = 0;          // tare for alt current
float BatteryCOffset=0;               // tare or batt current

//Pointless Flags delete later
int ResetTemp;              // reset the maximum alternator temperature tracker
int ResetVoltage;           // reset the maximum battery voltage measured
int ResetCurrent;           // reset the maximmum alternator output current
int ResetEngineRunTime;     // reset engine run time tracker
int ResetAlternatorOnTime;  //reset AlternatorOnTime
int ResetEnergy;            // ???
int ResetDischargedEnergy;  // total discharged from battery
int ResetFuelUsed;          // fuel used by alternator
int ResetAlternatorChargedEnergy;
int ResetEngineCycles;
int ResetRPMMax;

int Voltage_scaled = 0;            // Battery voltage scaled (V × 100)
int BatteryCurrent_scaled = 0;     // A × 100
int AlternatorCurrent_scaled = 0;  // Alternator current scaled (A × 100)
int BatteryPower_scaled = 0;       // Battery power (W × 100)
int EnergyDelta_scaled = 0;        // Energy change (Wh × 100)
int AlternatorPower_scaled = 0;    // Alternator power (W × 100)
int AltEnergyDelta_scaled = 0;     // Alternator energy change (Wh × 100)
int joulesOut = 0;
int fuelEnergyUsed_J = 0;
int AlternatorFuelUsed = 0;   // Total fuel used by alternator (mL) - INTEGER (note: mL not L)
bool alternatorIsOn = false;  // Current alternator state
// Energy Tracking Variables

int ChargedEnergy = 0;            // Total charged energy from battery (Wh) - INTEGER
int DischargedEnergy = 0;         // Total discharged energy from battery (Wh) - INTEGER
int AlternatorChargedEnergy = 0;  // Total energy from alternator (Wh) - INTEGER
int FuelEfficiency_scaled = 250;  // Engine efficiency: Wh per mL of fuel (× 100)
// Engine & Alternator Runtime Tracking
int EngineRunTime = 0;          // Time engine has been spinning (minutes)
int EngineCycles = 0;           // Average RPM * Minutes of run time
int AlternatorOnTime = 0;       // Time alternator has been producing current (minutes)
bool engineWasRunning = false;  // Engine state in previous check
bool alternatorWasOn = false;   // Alternator state in previous check


// variables used to show how long each loop takes
uint64_t starttime;
uint64_t endtime;
int LoopTime;             // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int WifiStrength;         // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int MaximumLoopTime;      // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int prev_millis7888 = 0;  // used to reset the meximum loop time

//"Blink without delay" style timer variables used to control how often differnet parts of code execute
static unsigned long prev_millis4;   //  Not Needed?
static unsigned long prev_millis66;  //used to delay the updating of the display
static unsigned long prev_millis22;  // used to delay field adjustment
static unsigned long prev_millis3;   // used to delay sampling of ADS1115 to every 2 seconds for example
//static unsigned long prev_millis2;    // used to delay sampling of temp sensor to every 2 seconds for example
static unsigned long prev_millis33;    // used to delay sampling of Serial Data (ve direct)
static unsigned long prev_millis743;   // used to read NMEA2K Network Every X seconds
static unsigned long prev_millis5;     // used to initiate wifi data exchange
static unsigned long lastINARead = 0;  // don't read the INA228 needlessly often
// Global variable to track ESP32 restart time
unsigned long lastRestartTime = 0;
const unsigned long RESTART_INTERVAL = 3600000;  // 1 hour in milliseconds

int BatteryVoltageSource = 0;  // select  "0">INA228    value="1">ADS1115     value="2">VictronVeDirect     value="3">NMEA0183     value="4">NMEA2K
int AmpControlByRPM = 0;       // this is the toggle
// In lieu of a table for RPM based charging....
int RPM1 = 0;
int RPM2 = 0;
int RPM3 = 0;
int RPM4 = 0;
int Amps1 = 0;
int Amps2 = 0;
int Amps3 = 0;
int Amps4 = 0;
int RPM5 = 0;
int RPM6 = 0;
int RPM7 = 0;
int Amps5 = 0;
int Amps6 = 0;
int Amps7 = 0;
int RPMThreshold = -20000;  //below this, there will be no field output in auto mode (Update this if we have RPM at low speeds and no field, otherwise, depend on Ignition)

int maxPoints;  //number of points plotted per plot (X axis length)



// pre-setup stuff
// onewire    Data wire is connetec to the Arduino digital pin 13
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;


//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_SSD1306 display(128, 64, 23, 18, 19, -1, 5);

//VictronEnergy
VeDirectFrameHandler myve;

// WIFI STUFF
AsyncWebServer server(80);               // Create AsyncWebServer object on port 80
AsyncEventSource events("/events");      // Create an Event Source on /events
unsigned long webgaugesinterval = 1000;  // delay in ms between sensor updates on webpage

// WiFi provisioning settings
const char *WIFI_SSID_FILE = "/ssid.txt";
const char *WIFI_PASS_FILE = "/pass.txt";


typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  { 126992L, &SystemTime },
  { 127245L, &Rudder },
  { 127250L, &Heading },
  { 127257L, &Attitude },
  { 127506L, &DCStatus },
  { 127513L, &BatteryConfigurationStatus },
  { 128259L, &Speed },
  { 128267L, &WaterDepth },
  { 129026L, &COGSOG },
  { 129029L, &GNSS },
  { 129540L, &GNSSSatsInView },
  { 0, 0 }
};

Stream *OutputStream;

//suspect i don't need this
//void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

//ADS1115 more pre-setup crap
enum ADS1115_State {
  ADS_IDLE,
  ADS_WAITING_FOR_CONVERSION
};

ADS1115_State adsState = ADS_IDLE;
uint8_t adsCurrentChannel = 0;
unsigned long adsStartTime = 0;
const unsigned long ADSConversionDelay = 155;  // 125 ms for 8 SPS but... Recommendation:Raise ADSConversionDelay to 150 or 160 ms — this gives ~20–30% margin without hurting performance much.     Can change this back later

// Forward declarations for WiFi functions
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void setupWiFi();
bool connectToWiFi(const char *ssid, const char *password, unsigned long timeout);
void setupAccessPoint();
void setupWiFiConfigServer();
void dnsHandleRequest();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// HTML for the WiFi configuration page
const char WIFI_CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>WiFi Setup</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <style>
    :root {
      --primary: #333333;
      --accent: #ff6600;
      --bg-light: #f5f5f5;
      --text-dark: #333333;
      --card-light: #ffffff;
      --radius: 4px;
      --input-border: #999999;
    }
    body {
      font-family: Arial, Helvetica, sans-serif;
      background-color: var(--bg-light);
      color: var(--text-dark);
      padding: 20px;
      line-height: 1.6;
      font-size: 14px;
    }
    h2 {
      color: var(--text-dark);
      border-bottom: 2px solid var(--accent);
      padding-bottom: 0.25rem;
      margin-top: 1rem;
      margin-bottom: 0.75rem;
      font-size: 18px;
    }
    .card {
      background: var(--card-light);
      padding: 16px;
      border-left: 2px solid var(--accent);
      border-radius: var(--radius);
      box-shadow: 0 1px 2px rgba(0,0,0,0.1);
      max-width: 400px;
      margin: 0 auto;
    }
    label {
      display: block;
      margin-bottom: 6px;
      font-weight: bold;
    }
    input[type="text"], input[type="password"] {
      width: 100%;
      padding: 8px;
      margin-bottom: 12px;
      border: 1px solid var(--input-border);
      border-radius: var(--radius);
      font-size: 14px;
      background-color: #fff;
      box-sizing: border-box;
    }
    .submit-row {
      text-align: center;
      margin-top: 16px;
    }
    input[type="submit"] {
      background-color: var(--accent);
      color: var(--text-light);
      padding: 10px 20px;
      border: none;
      border-radius: var(--radius);
      cursor: pointer;
      font-weight: bold;
      font-size: 14px;
    }
    input[type="submit"]:hover {
      background-color: #e65c00;
    }
  </style>
</head>
<body>
  <div class="card">
    <h2>Configure WiFi</h2>
    <p>Enter the ship's WiFi network credentials</p>
    <form action="/wifi" method="POST">
      <label for="ssid">Network Name (SSID):</label>
      <input type="text" id="ssid" name="ssid" required>

      <label for="password">Password:</label>
      <input type="password" id="password" name="password" required>

      <div class="submit-row">
        <input type="submit" value="Save">
      </div>
    </form>
  </div>
</body>
</html>
)rawliteral";


void setup() {
  esp_log_level_set("esp32-hal-i2c-ng", ESP_LOG_WARN);  // get rid of spam in serial monitor
  queueConsoleMessage("System starting up...");
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  delay(500);  // not sure if this is needed

  pinMode(4, OUTPUT);  // This pin is used to provide a high signal to Field Enable pin
  pinMode(2, OUTPUT);  // This pin is used to provide a heartbeat (pin 2 of ESP32 is the LED)
  // In ESP32 v3.x, this single function replaces both ledcSetup and ledcAttachPin
  ledcAttach(pwmPin, fffr, pwmResolution);


  // Initialize LittleFS first
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    queueConsoleMessage("WARNING: An Error has occurred while mounting LittleFS");
    // Continue anyway since we might be able to format and use it later (?)
  }

  InitPersistentVariables();  // load all persistent variables from LittleFS.  If no files exist, create them.
  InitSystemSettings();       // load all settings from LittleFS.  If no files exist, create them.

  // Setup WiFi (this will either connect to a saved network or create an AP)
  setupWiFi();
  setupServer();
  loadPasswordHash();

  //NMEA2K
  OutputStream = &Serial;
  //   while (!Serial)
  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50); // was commented
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);  // was false
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->print("NMEA2K Running...");

  //Victron VeDirect
  Serial1.begin(19200, SERIAL_8N1, 25, -1, 0);  // ... note the "0" at end for normal logic.  This is the reading of the combined NMEA0183 data from YachtDevices
  Serial2.begin(19200, SERIAL_8N1, 26, -1, 1);  // This is the reading of Victron VEDirect
  Serial2.flush();

  if (!INA.begin()) {
    Serial.println("Could not connect INA. Fix and Reboot");
    queueConsoleMessage("WARNING: Could not connect INA228 Battery Voltage/Amp measuring chip");


    INADisconnected = 1;
    // while (1)
    ;
  } else {
    INADisconnected = 0;
  }

  // at least 529ms for an update with these settings for average and conversion time
  INA.setMode(11);                       // Bh = Continuous shunt and bus voltage
  INA.setAverage(4);                     //0h = 1, 1h = 4, 2h = 16, 3h = 64, 4h = 128, 5h = 256, 6h = 512, 7h = 1024     Applies to all channels
  INA.setBusVoltageConversionTime(7);    // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs
  INA.setShuntVoltageConversionTime(7);  // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs

  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 dipslay allocation failed"));
    queueConsoleMessage("WARNING: SSD1306 OLED display allocation failed");
    for (;;)
      ;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();

  ChargingVoltageTarget = TargetFloatVoltage;

  //ADS1115
  //Connection check
  if (!adc.testConnection()) {
    Serial.println("ADS1115 Connection failed and would have triggered a return if it wasn't commented out");
    queueConsoleMessage("WARNING: ADS1115 Analog Input chip failed");

    ADS1115Disconnected = 1;
    // return;
  } else {
    ADS1115Disconnected = 0;
  }
  //Gain parameter.
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
  // ADS1115_REG_CONFIG_PGA_6_144V(0x0000)  // +/-6.144V range = Gain 2/3
  // ADS1115_REG_CONFIG_PGA_4_096V(0x0200)  // +/-4.096V range = Gain 1
  // ADS1115_REG_CONFIG_PGA_2_048V(0x0400)  // +/-2.048V range = Gain 2 (default)
  // ADS1115_REG_CONFIG_PGA_1_024V(0x0600)  // +/-1.024V range = Gain 4
  // ADS1115_REG_CONFIG_PGA_0_512V(0x0800)  // +/-0.512V range = Gain 8
  // ADS1115_REG_CONFIG_PGA_0_256V(0x0A00)  // +/-0.256V range = Gain 16
  //Sample rate parameter
  adc.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS);  //Set the slowest and most accurate sample rate, 8
  // ADS1115_REG_CONFIG_DR_8SPS(0x0000)              // 8 SPS(Sample per Second), or a sample every 125ms
  // ADS1115_REG_CONFIG_DR_16SPS(0x0020)             // 16 SPS, or every 62.5ms
  // ADS1115_REG_CONFIG_DR_32SPS(0x0040)             // 32 SPS, or every 31.3ms
  // ADS1115_REG_CONFIG_DR_64SPS(0x0060)             // 64 SPS, or every 15.6ms
  // ADS1115_REG_CONFIG_DR_128SPS(0x0080)            // 128 SPS, or every 7.8ms  (default)
  // ADS1115_REG_CONFIG_DR_250SPS(0x00A0)            // 250 SPS, or every 4ms, note that noise free resolution is reduced to ~14.75-16bits, see table 2 in datasheet
  // ADS1115_REG_CONFIG_DR_475SPS(0x00C0)            // 475 SPS, or every 2.1ms, note that noise free resolution is reduced to ~14.3-15.5bits, see table 2 in datasheet
  // ADS1115_REG_CONFIG_DR_860SPS(0x00E0)            // 860 SPS, or every 1.16ms, note that noise free resolution is reduced to ~13.8-15bits, see table 2 in datasheet

  //onewire
  sensors.begin();
  sensors.setResolution(12);
  sensors.getAddress(tempDeviceAddress, 0);
  if (sensors.getDeviceCount() == 0) {
    Serial.println("WARNING: No DS18B20 sensors found on the bus.");
    queueConsoleMessage("WARNING: No DS18B20 sensors found on the bus");

    sensors.setWaitForConversion(false);  // this is critical!
  }


  // Enable watchdog with new API for ESP32 Arduino 3.x
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 8000,    // 8 seconds timeout
    .idle_core_mask = 0,   // Don't watch idle cores
    .trigger_panic = true  // Restart on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);  // Add current task to watchdog

  // Check if we recovered from a watchdog reset
  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason == ESP_RST_TASK_WDT) {
    queueConsoleMessage("CRITICAL: System recovered from watchdog reset - code was hung");
  } else if (reset_reason == ESP_RST_SW) {
    queueConsoleMessage("System restarted - scheduled maintenance");
  }

  xTaskCreatePinnedToCore(
    TempTask,
    "TempTask",
    4096,
    NULL,
    0,  // Priority lower than normal (execute if nothing else to do, all the 1's are idle)
    &tempTaskHandle,
    0  // Run on Core 0, which is the one doing Wifi and system tasks, and theoretically has more idle points than Core 1 and "loop()"
  );
}

void loop() {
  starttime = esp_timer_get_time();  // Record start time for Loop
  currentTime = millis();
  esp_task_wdt_reset();  // Feed the watchdog

  // SOC and runtime update every 2 seconds
  if (currentTime - lastSOCUpdateTime >= SOCUpdateInterval) {
    elapsedMillis = currentTime - lastSOCUpdateTime;
    lastSOCUpdateTime = currentTime;

    UpdateEngineRuntime(elapsedMillis);
    UpdateBatterySOC(elapsedMillis);
  }

  // Periodic Data Save Logic - run every DataSaveInterval
  if (currentTime - lastDataSaveTime >= DataSaveInterval) {  // have to figure out an appropriate rate for this to not wear out Flash memory
    lastDataSaveTime = currentTime;
    SaveAllData();
  }

  // Handle DNS requests if in AP mode
  if (currentWiFiMode == AWIFI_MODE_AP) {
    dnsHandleRequest();
  } else {
    // This is all the normal stuff .... only done in Client Mode
    ReadAnalogInputs();
    if (VeData == 1) {
      ReadVEData();  //read Data from Victron VeDirect
    }
    if (NMEA2KData == 1) {
      NMEA2000.ParseMessages();  // read data from NMEA2K
    }

    // UpdateDisplay();   // turn this back on later
    AdjustField();  // This may need to get moved if it takes any power, but have to be careful we don't get stuck with Field On!

    // Handle ignition-based power management for saving power
    if (Ignition == 0) {
      setCpuFrequencyMhz(10);
      WiFi.mode(WIFI_OFF);
      Serial.println("wifi has been turned off");
    } else {
      if (WiFi.status() != WL_CONNECTED) {             // Only setup WiFi if not already connected
        setCpuFrequencyMhz(240);                       // turn it back on - ignition is on so enter full power mode
        setupWiFi();                                   // Use our new WiFi setup function
        Serial.println("Wifi is reinitialized");       // too many ignition on/off cycles could break this, something to harden later?
        queueConsoleMessage("Wifi is reinitialized");  // too many ignition on/off cycles could break this, something to harden later?
      }
      // Send WiFi data to client
      SendWifiData();
      // Check WiFi connection status if in client mode - only try to reconnect when ignition is on
      if (currentWiFiMode == AWIFI_MODE_CLIENT && WiFi.status() != WL_CONNECTED) {
        static unsigned long lastWiFiCheckTime = 0;
        // Try to reconnect every 5 seconds
        if (millis() - lastWiFiCheckTime > 5000) {
          lastWiFiCheckTime = millis();
          Serial.println("WiFi connection lost. Attempting to reconnect...");
          String saved_ssid = readFile(LittleFS, WIFI_SSID_FILE);
          String saved_password = readFile(LittleFS, WIFI_PASS_FILE);
          if (connectToWiFi(saved_ssid.c_str(), saved_password.c_str(), 3000)) {
            Serial.println("Reconnected to WiFi!");
            queueConsoleMessage("Reconnected to WiFi!");
            // mDNS will be reinitialized in the connectToWiFi function
          } else {
            Serial.println("Failed to reconnect. Will try again in 5 seconds.");
          }
        }
      }
      Freq = getCpuFrequencyMhz();  // unused at this time
    }
  }

  //Blink LED on and off every X seconds - works in both power modes for status indication
  // optimize for power consumpiton later
  //AP Mode: Fast triple blink pattern
  //WiFi Disconnected: Medium blink with 100ms pulse
  //WiFi Connected: Normal toggle
  if (millis() - previousMillisBLINK >= intervalBLINK) {
    // Use different blink patterns to indicate WiFi status
    if (currentWiFiMode == AWIFI_MODE_AP) {
      // Fast blink in AP mode (toggle twice)
      digitalWrite(2, HIGH);
      delay(50);
      digitalWrite(2, LOW);
      delay(50);
      digitalWrite(2, HIGH);
      delay(50);
      digitalWrite(2, (ledState = !ledState));
    } else if (WiFi.status() != WL_CONNECTED) {
      // Medium blink when WiFi is disconnected
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, (ledState = !ledState));
    } else {
      // Normal blink when connected
      digitalWrite(2, (ledState = !ledState));
    }
    previousMillisBLINK = millis();
  }
  if (millis() - prev_millis7888 > 3000) {  // every 3 seconds reset the maximum loop time
    MaximumLoopTime = 0;
    prev_millis7888 = millis();
  }
  endtime = esp_timer_get_time();  //Record end of Loop
  LoopTime = (endtime - starttime);

  if (LoopTime > MaximumLoopTime) {
    MaximumLoopTime = LoopTime;
  }
  checkAndRestart();  // Handle scheduled maintenance restarts
}

template<typename T> void PrintLabelValWithConversionCheckUnDef(const char *label, T val, double (*ConvFunc)(double val) = 0, bool AddLf = false, int8_t Desim = -1) {
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if (Desim < 0) {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val));
      } else {
        OutputStream->print(val);
      }
    } else {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val), Desim);
      } else {
        OutputStream->print(val, Desim);
      }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
}
//*****************************************************************************
