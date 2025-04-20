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
#include "SiC45x.h"  // heart of system, DC/DC converter chip
SiC45x sic45x(0x1D);
//DONT MOVE THE NEXT 6 LINES AROUND, MUST STAY IN THIS ORDER
#include <Arduino.h>                  // maybe not needed, was in NMEA2K example I copied
#define ESP32_CAN_RX_PIN GPIO_NUM_16  //
#define ESP32_CAN_TX_PIN GPIO_NUM_17  //
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>  // questionably needed
#include <WiFi.h>
#include <AsyncTCP.h>           // for wifi stuff, important, don't ever update, use mathieucarbou github repository
#include <LittleFS.h>           // for wifi stuff
#include <ESPAsyncWebServer.h>  // for wifi stuff, important, don't ever update, use mathieucarbou github repository


// Settings
// Replace next 2 lines with your own WiFi network credentials
const char *ssid = "MN2G";
const char *password = "5FENYC8PDW";

// extern WebServer server;  // declared in user's main file

// const char* ap_ssid = "RegulatorSetup";
// const char* ap_password = "changeme123";

// String sta_ssid = "";
// String sta_password = "";





//delete this later ?
int powersavemode = 0;
int INADisconnected = 0;
int WifiHeartBeat = 0;
int VeTime = 0;
int SendWifiTime = 0;
int AnalogReadTime = 0;   // this is the present
int AnalogReadTime2 = 0;  // this is the maximum ever

//Input Settings
int TargetAmps = 55;  //Normal alternator output, for best performance, set ot something that just barely won't overheat
float TargetFloatVoltage = 13.9;
float TargetBulkVoltage = 14.5;
float ChargingVoltageTarget = 0;          // This is what the code really uses. It gets set to TargetFloatVoltage or TargetBulkVoltage later on
float interval = 0.1;                     // voltage step to adjust field target by, each time the loop runs.  Larger numbers = faster response, less stability
float FieldAdjustmentInterval = 500;      // The regulator field output is updated once every this many milliseconds
float MinimumFieldVoltage = 1;            // A min value here ensures that engine speed can be measured even with no alternator output commanded.  (This is only enforced when Ignition input is high)
float AlternatorTemperatureLimitF = 150;  // the offset appears to be +40 to +50 to get true max alternator external metal temp, depending on temp sensor installation, so 150 here will produce a metal temp ~200F
int ManualFieldToggle = 1;                // set to 1 to enable manual control of regulator field output, helpful for debugging
float ManualVoltageTarget = 5;            // voltage target corresponding to the toggle above
int SwitchControlOverride = 1;            // set to 1 for web interface switches to override physical switch panel
int ForceFloat = 0;                       // set to 1 to force the float voltage to be the charging target
int OnOff = 1;                            // 0 is charger off, 1 is charger On
int Ignition = 1;                         // This will eventually be an over-rideable digital input
int HiLow = 1;                            // 0 will be a low setting, 1 a high setting
int LimpHome = 0;                         // 1 will set to limp home mode, whatever that gets set up to be
float vout = 1;                           // default field output voltage
int FaultCheckToggle = 0;                 // Set to 1 to get a serial print out over 20 seconds of error data, delete later
int resolution = 12;                      // for OneWire temp sensor measurement
float fffr = 1200;                        // this is the switching frequency for SIC450 in khz units
int VeData = 0;                           // Set to 1 if VE serial data exists
int NMEA0183Data = 0;                     // Set to 1 if NMEA serial data exists doesn't do anything yet
int NMEA2KData = 1;                       // doesn't do anything yet

int previousMillisZZ = 0;  // Temporary, for getting power consumption down
int intervalZZ = 60000;    // Turn WiFi on and off every 60 seconds (until there's an Ignition signal controlling it)
uint32_t Freq = 0;         // ESP32 switching Frequency in case we want to report it for debugging
int previousMillisBLINK;   // used for heartbeat blinking LED test can delete later
int intervalBLINK = 1000;  // used for heartbeat blinking LED test can delete later
bool ledState;             // used for heartbeat blinking LED test can delete later

//Variables to store measurements
float ShuntVoltage_mV;                  // Battery shunt voltage from INA228
float Bcur;                             // battery shunt current from INA228
float IBV;                              // Ina 228 battery voltage
float IBVMax = NAN;                     // used to track maximum battery voltage
float DutyCycle;                        // SIC outout %
float vvout;                            // SIC output volts
float iiout;                            // SIC output current
float AlternatorTemperatureF = NAN;     // alternator temperature
float AlternatorTemperatureFMax = NAN;  // used to track maximum alternator temperature
TaskHandle_t tempTaskHandle = NULL;     // make a separate cpu task for temp reading because it's so slow
float VictronVoltage = 0;               // battery reading from VeDirect
float HeadingNMEA = 0;                  // Just here to test NMEA functionality

// ADS1115
int16_t Raw = 0;
float Channel0V, Channel1V, Channel2V, Channel3V;
float BatteryV, MeasuredAmps, RPM;
float MeasuredAmpsMax;  // used to track maximum alternator output
float RPMMax;           // used to track maximum RPM
int ADS1115Disconnected = 0;

//Engine Run Time Related Tracking
int EngineRunTime = 0;     // time engine has been spinning
int EngineCycles = 0;      // average RPM * Minutes of run time
int AlternatorOnTime = 0;  // might be useful for belt replacement or other maintenance

//Battery Monitor style variables
int SOC;                      // Battery State of charge
int ChargedEnergy;            // Total Charged Energy from Battery
int DischargedEnergy;         // Total Discharged Energy from Battery
int AlternatorChargedEnergy;  // Total Energy from Alternator
int AlternatorFuelUsed;       // Estimate of fuel used by Alternator
int SolarFuelSaved;           //Charged Energy - AlternatorChargedEnergy * constant

//Controls For LittleFS
int ResetTemp;              // reset the maximum alternator temperature tracker
int ResetVotage;            // reset the maximum battery voltage measured
int ResetCurrent;           // reset the maximmum alternator output current
int ResetEngineRunTime;     // reset engine run time tracker
int ResetAlternatorOnTime;  //reset AlternatorOnTime
int ResetEnergy;            // reset Alternator/other Charged energy, and Discharged Energy, and Fuel used

// variables used to show how long each loop takes
uint64_t starttime;
uint64_t endtime;
int LoopTime;             // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int WifiStrength;         // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int MaximumLoopTime;      // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int prev_millis7888 = 0;  // used to reset the meximum loop time
int FreeHeap;             // as it says, free memory tracker

//"Blink without delay" style timer variables used to control how often differnet parts of code execute
static unsigned long prev_millis4;   // used to delay checking of faults in the SiC450
static unsigned long prev_millis66;  //used to delay the updating of the display
static unsigned long prev_millis22;  // used to delay sampling of sic450
static unsigned long prev_millis3;   // used to delay sampling of ADS1115 to every 2 seconds for example
//static unsigned long prev_millis2;    // used to delay sampling of temp sensor to every 2 seconds for example
static unsigned long prev_millis33;    // used to delay sampling of Serial Data (ve direct)
static unsigned long prev_millis743;   // used to read NMEA2K Network Every X seconds
static unsigned long prev_millis5;     // used to initiate wifi data exchange
static unsigned long lastINARead = 0;  // don't read the INA228 needlessly often
// Global variable to track ESP32 restart time
unsigned long lastRestartTime = 0;
const unsigned long RESTART_INTERVAL = 3600000;  // 1 hour in milliseconds

// pre-setup stuff
// onewire    Data wire is connetec to the Arduino digital pin 13
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;


//SIC450 control
bool SIC450Enabler = 1;

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);´
Adafruit_SSD1306 display(128, 64, 23, 18, 19, -1, 5);

//VictronEnergy
VeDirectFrameHandler myve;

// WIFI STUFF
AsyncWebServer server(80);               // Create AsyncWebServer object on port 80
AsyncEventSource events("/events");      // Create an Event Source on /events
unsigned long webgaugesinterval = 1000;  // delay in ms between sensor updates on webpage

const char *TLimit = "TemperatureLimitF";  //TLimit is a pointer to an immutable String "TemperatureLimitF"
const char *ManualV = "ManualVoltage";
const char *FullChargeV = "FullChargeVoltage";
const char *TargetA = "TargetAmpz";
const char *FrequencyP = "SwitchingFrequency";
const char *TFV = "TargetFloatVoltage1";
const char *Intt = "interval1";
const char *FAI = "FieldAdjustmentInterval1";
const char *MFT = "ManualFieldToggle1";
const char *SCO = "SwitchControlOverride1";
const char *FF = "ForceFLoat1";
const char *OO = "OnOff1";
const char *HL = "HiLow1";
const char *LH = "LimpHome1";
const char *VD = "VeData1";
const char *N0 = "NMEA0183Data1";
const char *N2 = "NMEA2KData1";


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




void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  delay(500);          // not sure if this is needed
  pinMode(4, OUTPUT);  // This pin is used to provide a high signal to SiC450 Enable pin
  pinMode(2, OUTPUT);  // This pin is used to provide a heartbeat (pin 2 of ESP32 is the LED)

  initWiFi();  // just leave it here




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
    INADisconnected = 1;
    // while (1)
    ;
  } else {
    INADisconnected = 0;
  }

  // at least 529ms for an update with these settings for average and conversion time
  INA.setMode(11);                       // Bh = Continuous shunt and bus voltage
  INA.setAverage(4);                     //0h = 1, 1h = 4, 2h = 16, 3h = 64, 4h = 128, 5h = 256, 6h = 512, 7h = 1024     Applies to all channels
  INA.setBusVoltageConversionTime(5);    // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs
  INA.setShuntVoltageConversionTime(5);  // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs

  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 dipslay allocation failed"));
    for (;;)
      ;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();

  ChargingVoltageTarget = TargetFloatVoltage;

  //SIC450
  //The range of settings allowed for each register varies stupidly with this chip, must stay within the below limits:
  //The VIN_OV_FAULT_LIMIT,VIN_UV_WARN_LIMIT  range is 1 V to 80 V, resolution is 0.5 V
  //output voltage’s range is 0.3 V to 14 V
  ///same for vout, margin high, margin low,VOUT_OV_FAULT_LIMIT,VOUT_OV_WARN_LIMIT...
  //The POWER_GOOD_ON and POWER_GOOD_OFF range is 0.24 V to 14 V
  //0V to 14V for VOUT_UV_WARN_LIMIT ,VOUT_UV_FAULT_LIMIT
  //The VIN_ON and VIN_OFF range is 1 V to 80 V, resolution is 0.5 V
  sic45x.begin();
  sic45x.sendClearFaults();
  sic45x.setVinOn(6);                    // The VIN_ON command sets the value of the input voltage, in volt, at which the PMBus unit should start power conversion
  sic45x.setVinOff(5);                   // The VIN_OFF command sets the value of the input voltage, in volt, at which the PMBus unit, once operation has started, should stop power conversion
  sic45x.setVoutMax(14);                 // The VOUT_MAX command sets an upper limit on the output voltage the unit cancommand regardless of any other commands or combinations 1.953 mV resolution 0.3 to 14 range
  sic45x.setVoutTransitionRate(0.0625);  // this is millivolts per microsecond and the lowest speed allowed. Highest would be .125 and resolution is .0625
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);
  sic45x.setVoutOvFaultResponse(SIC45X_VOUT_OV_FAULT_RESPONSE_OVRSP_CONTINUE);  // The PMBus device continues operation without interruption
  sic45x.setVoutUvFaultResponse(SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_CONTINUE);  // The device continues operation without interruption
  sic45x.setIoutOcFaultResponse(SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_CONTINUE);  // The device continues operation without interruption
  sic45x.setOtFaultResponse(SIC45X_OT_FAULT_RESPONSE_OTRSP_CONTINUE);           // The device continues operation without interruption
  sic45x.setVinOvFaultResponse(SIC45X_VIN_OV_FAULT_RESPONSE_OVRSP_CONTINUE);    // The device continues operation without interruption
  sic45x.setVinOvFaultLimit(79);                                                // this should do nothing anyway, was 15 by default
  sic45x.setOnOffConfiguration(SIC45X_ON_OFF_CONFIGURATION_PU_COMMAND);         // Regulator does not power up until commanded by the CONTROLEN pin and OPERATION command
  sic45x.setOnOffConfiguration(SIC45X_ON_OFF_CONFIGURATION_CMD_RESPOND);        // Regulator responds the "on" bit in the OPERATION command
  sic45x.setOperation(SIC45X_OPERATION_OFF_B_IMMEDIATE);                        // Output is turned off immediately and power off sequence commands ignored
  sic45x.setOperation(SIC45X_OPERATION_MARGIN_COMMAND);                         // Output voltage is set by the PMBus VOUT_COMMAND data
  sic45x.setOperation(SIC45X_OPERATION_ON_OFF_ENABLED);                         // Enable output, the might be wrong order with next line, not sure
  sic45x.setOperation(SIC45X_OPERATION_MRGNFLT_IGNORE);                         // Faults caused by selecting VOUT_MARGIN_HIGH or VOUT_MARGIN_LOW as the nominal output voltage source are ignored
  sic45x.setOnOffConfiguration(SIC45X_ON_OFF_CONFIGURATION_EN_REQUIRE);         // Regulator requires the EN pin to be asserted to start the unit
  sic45x.setOnOffConfiguration(SIC45X_ON_OFF_CONFIGURATION_ENPOL_HIGH);         // EN signal is active high
  sic45x.setOnOffConfiguration(SIC45X_ON_OFF_CONFIGURATION_OFFB1_IMMEDIATE);    // Regulator turns off immediately
  //Temporary setup specific stuff
  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_0V3_1V8);  // get ready to output 0.5V for now
  sic45x.setFrequencySwitch(fffr);                          //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
  sic45x.setPowerGoodOn(vout * 0.9);                        // .9    Try deleting this later
  sic45x.setPowerGoodOff(vout * 0.85);                      // .85   Try deleting this later
  sic45x.setVoutOvFaultLimit(vout * 1.15);                  //    I think this one is required
  sic45x.setVoutOvWarnLimit(vout * 1.1);                    // 110  Try deleting this later
  sic45x.setVoutUvWarnLimit(vout * 0.9);                    // .9 Try delting this later
  sic45x.setVoutUvFaultLimit(vout * 0.8);                   // .8 Try deleting this later
  sic45x.setVoutMarginLow(vout * 0.95);                     //.95   Try delting this later
  sic45x.setVoutMarginHigh(vout * 1.05);                    //105 Try deleting this later
  sic45x.setVoutCommand(vout);                              // Update the field voltage, during setup, it's 0.5V


  //ADS1115
  //Connection check
  if (!adc.testConnection()) {
    Serial.println("ADS1115 Connection failed and would have triggered a return if it wasn't commented out");
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
    sensors.setWaitForConversion(false);  // this is critical!
  }


  //WIFI STUFF
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed and triggered a return!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS and triggered a return");
    return;
  }


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html", false, processor);
  });


  // Send a GET request to <ESP_IP>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(TLimit)) {
      inputMessage = request->getParam(TLimit)->value();
      writeFile(LittleFS, "/TemperatureLimitF.txt", inputMessage.c_str());  //Converts the contents of a String as a C-style null-terminated string.
      AlternatorTemperatureLimitF = inputMessage.toInt();
    } else if (request->hasParam(ManualV)) {
      inputMessage = request->getParam(ManualV)->value();
      writeFile(LittleFS, "/ManualVoltage.txt", inputMessage.c_str());
      ManualVoltageTarget = inputMessage.toFloat();
    } else if (request->hasParam(FullChargeV)) {
      inputMessage = request->getParam(FullChargeV)->value();
      writeFile(LittleFS, "/FullChargeVoltage.txt", inputMessage.c_str());
      ChargingVoltageTarget = inputMessage.toFloat();
    } else if (request->hasParam(TargetA)) {
      inputMessage = request->getParam(TargetA)->value();
      writeFile(LittleFS, "/TargetAmpz.txt", inputMessage.c_str());  //Take the string stored in inputMessage, and write it to a file called TargetAmpz.txt on the LittleFS filesystem inside the ESP32's flash.
      TargetAmps = inputMessage.toInt();
    } else if (request->hasParam(FrequencyP)) {
      inputMessage = request->getParam(FrequencyP)->value();
      writeFile(LittleFS, "/SwitchingFrequency.txt", inputMessage.c_str());
      fffr = inputMessage.toInt();
    } else if (request->hasParam(TFV)) {
      inputMessage = request->getParam(TFV)->value();
      writeFile(LittleFS, "/TargetFloatVoltage1.txt", inputMessage.c_str());
      TargetFloatVoltage = inputMessage.toFloat();
    } else if (request->hasParam(Intt)) {
      inputMessage = request->getParam(Intt)->value();
      writeFile(LittleFS, "/interval1.txt", inputMessage.c_str());
      interval = inputMessage.toFloat();
    } else if (request->hasParam(FAI)) {
      inputMessage = request->getParam(FAI)->value();
      writeFile(LittleFS, "/FieldAdjustmentInterval1.txt", inputMessage.c_str());
      FieldAdjustmentInterval = inputMessage.toFloat();
    } else if (request->hasParam(MFT)) {
      inputMessage = request->getParam(MFT)->value();
      writeFile(LittleFS, "/ManualFieldToggle1.txt", inputMessage.c_str());
      ManualFieldToggle = inputMessage.toInt();
    } else if (request->hasParam(SCO)) {
      inputMessage = request->getParam(SCO)->value();
      writeFile(LittleFS, "/SwitchControlOverride1.txt", inputMessage.c_str());
      SwitchControlOverride = inputMessage.toInt();
    } else if (request->hasParam(FF)) {
      inputMessage = request->getParam(FF)->value();
      writeFile(LittleFS, "/ForceFloat1.txt", inputMessage.c_str());
      ForceFloat = inputMessage.toInt();
    } else if (request->hasParam(OO)) {
      inputMessage = request->getParam(OO)->value();
      writeFile(LittleFS, "/OnOff1.txt", inputMessage.c_str());
      OnOff = inputMessage.toInt();
    } else if (request->hasParam(HL)) {
      inputMessage = request->getParam(HL)->value();
      writeFile(LittleFS, "/HiLow1.txt", inputMessage.c_str());
      HiLow = inputMessage.toInt();
    } else if (request->hasParam(LH)) {
      inputMessage = request->getParam(LH)->value();
      writeFile(LittleFS, "/LimpHome1.txt", inputMessage.c_str());
      LimpHome = inputMessage.toInt();
    } else if (request->hasParam(VD)) {
      inputMessage = request->getParam(VD)->value();
      writeFile(LittleFS, "/VeData1.txt", inputMessage.c_str());
      VeData = inputMessage.toInt();
    } else if (request->hasParam(N0)) {
      inputMessage = request->getParam(N0)->value();
      writeFile(LittleFS, "/NMEA0183Data1.txt", inputMessage.c_str());
      NMEA0183Data = inputMessage.toInt();
    } else if (request->hasParam(N2)) {
      inputMessage = request->getParam(N2)->value();
      writeFile(LittleFS, "/NMEA2KData1.txt", inputMessage.c_str());
      NMEA2KData = inputMessage.toInt();
    }

    else {
      inputMessage = "No message sent";
    }

    // Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  //server.onNotFound(notFound);

  server.onNotFound([](AsyncWebServerRequest *request) {
    String path = request->url();
    Serial.print("Request for: ");
    Serial.println(path);

    if (LittleFS.exists(path)) {
      Serial.println("File exists, serving...");

      // Determine content type based on file extension
      String contentType = "text/html";
      if (path.endsWith(".css")) contentType = "text/css";
      else if (path.endsWith(".js")) contentType = "application/javascript";
      else if (path.endsWith(".json")) contentType = "application/json";
      else if (path.endsWith(".png")) contentType = "image/png";
      else if (path.endsWith(".jpg")) contentType = "image/jpeg";

      request->send(LittleFS, path, contentType);
    } else {
      Serial.print("File not found: ");
      Serial.println(path);
      request->send(404, "text/plain", "File Not Found");
    }
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();

  // If there are not settings in Flash memoery (files don't exist yet), populate them with the hardcoded values
  bool TempLimitfileexists = LittleFS.exists("/TemperatureLimitF.txt");
  if (!TempLimitfileexists) {
    //String stringOne = String(13);                        // Convert an Integer to a String
    //String stringOne = String(5.69, 2);                      // Convert a float to a string with 2 decimal places
    String stringAlternatorTemperatureLimitF = String(AlternatorTemperatureLimitF);
    writeFile(LittleFS, "/TemperatureLimitF.txt", stringAlternatorTemperatureLimitF.c_str());
  }
  bool ManualVfileexists = LittleFS.exists("/ManualVoltage.txt");
  if (!ManualVfileexists) {
    String stringManualVV = String(ManualVoltageTarget, 2);
    writeFile(LittleFS, "/ManualVoltage.txt", stringManualVV.c_str());
  }
  bool FCVfileexists = LittleFS.exists("/FullChargeVoltage.txt");
  if (!FCVfileexists) {
    String stringFCV = String(ChargingVoltageTarget, 2);
    writeFile(LittleFS, "/FullChargeVoltage.txt", stringFCV.c_str());
  }
  bool TAfileexists = LittleFS.exists("/TargetAmpz.txt");
  if (!TAfileexists) {
    String stringTA = String(TargetAmps);
    writeFile(LittleFS, "/TargetAmpz.txt", stringTA.c_str());
  }
  bool Freqfileexists = LittleFS.exists("/SwitchingFrequency.txt");
  if (!Freqfileexists) {
    String stringFreq = String(fffr);
    writeFile(LittleFS, "/SwitchingFrequency.txt", stringFreq.c_str());
  }
  bool TFVfileexists = LittleFS.exists("/TargetFloatVoltage1.txt");
  if (!TFVfileexists) {
    String stringTFV = String(TargetFloatVoltage);
    writeFile(LittleFS, "/TargetFloatVoltage1.txt", stringTFV.c_str());
  }
  bool intervalfileexists = LittleFS.exists("/interval1.txt");
  if (!intervalfileexists) {
    String stringInterval = String(interval);
    writeFile(LittleFS, "/interval1.txt", stringInterval.c_str());
  }
  bool Fintervalfileexists = LittleFS.exists("/FieldAdjustmentInterval1.txt");
  if (!Fintervalfileexists) {
    String stringFAI = String(FieldAdjustmentInterval);
    writeFile(LittleFS, "/FieldAdjustmentInterval1.txt", stringFAI.c_str());
  }
  bool MFTexists = LittleFS.exists("/ManualFieldToggle1.txt");
  if (!MFTexists) {
    String stringMFT = String(ManualFieldToggle);
    writeFile(LittleFS, "/ManualFieldToggle1.txt", stringMFT.c_str());
  }
  bool SCOexists = LittleFS.exists("/SwitchControlOverride1.txt");
  if (!SCOexists) {
    String stringSCO = String(SwitchControlOverride);
    writeFile(LittleFS, "/SwitchControlOverride1.txt", stringSCO.c_str());
  }
  bool FFexists = LittleFS.exists("/ForceFloat1.txt");
  if (!FFexists) {
    String stringFF = String(ForceFloat);
    writeFile(LittleFS, "/ForceFloat1.txt", stringFF.c_str());
  }
  bool OOexists = LittleFS.exists("/OnOff1.txt");
  if (!OOexists) {
    String stringOO = String(OnOff);
    writeFile(LittleFS, "/OnOff1.txt", stringOO.c_str());
  }
  bool HLexists = LittleFS.exists("/HiLow1.txt");
  if (!HLexists) {
    String stringHL = String(HiLow);
    writeFile(LittleFS, "/HiLow1.txt", stringHL.c_str());
  }
  bool LHexists = LittleFS.exists("/LimpHome1.txt");
  if (!LHexists) {
    String stringLH = String(LimpHome);
    writeFile(LittleFS, "/LimpHome1.txt", stringLH.c_str());
  }
  bool VDexists = LittleFS.exists("/VeData1.txt");
  if (!VDexists) {
    String stringVD = String(VeData);
    writeFile(LittleFS, "/VeData1.txt", stringVD.c_str());
  }
  bool N0exists = LittleFS.exists("/NMEA0183Data1.txt");
  if (!N0exists) {
    String stringN0 = String(NMEA0183Data);
    writeFile(LittleFS, "/NMEA0183Data1.txt", stringN0.c_str());
  }
  bool N2exists = LittleFS.exists("/NMEA2KData1.txt");
  if (!N2exists) {
    String stringN2 = String(NMEA2KData);
    writeFile(LittleFS, "/NMEA2KData1.txt", stringN2.c_str());
  }

  //Update some variables with values from ESP32 Flash memory
  AlternatorTemperatureLimitF = readFile(LittleFS, "/TemperatureLimitF.txt").toInt();
  ManualVoltageTarget = readFile(LittleFS, "/ManualVoltage.txt").toFloat();
  ChargingVoltageTarget = readFile(LittleFS, "/FullChargeVoltage.txt").toFloat();
  TargetAmps = readFile(LittleFS, "/TargetAmpz.txt").toInt();
  fffr = readFile(LittleFS, "/SwitchingFrequency.txt").toInt();
  TargetFloatVoltage = readFile(LittleFS, "/TargetFloatVoltage1.txt").toFloat();
  interval = readFile(LittleFS, "/interval1.txt").toFloat();
  FieldAdjustmentInterval = readFile(LittleFS, "/FieldAdjustmentInterval1.txt").toFloat();
  ManualFieldToggle = readFile(LittleFS, "/ManualFieldToggle1.txt").toInt();
  SwitchControlOverride = readFile(LittleFS, "/SwitchControlOverride1.txt").toInt();
  ForceFloat = readFile(LittleFS, "/ForceFloat1.txt").toInt();
  OnOff = readFile(LittleFS, "/OnOff1.txt").toInt();
  HiLow = readFile(LittleFS, "/HiLow1.txt").toInt();
  LimpHome = readFile(LittleFS, "/LimpHome1.txt").toInt();
  VeData = readFile(LittleFS, "/VeData1.txt").toInt();
  NMEA0183Data = readFile(LittleFS, "/NMEA0183Data1.txt").toInt();
  NMEA2KData = readFile(LittleFS, "/NMEA2KData1.txt").toInt();

  // This one worked (~7ms max loop time with analog inputs off) )but I'm trying core 0 to see if it's better
  // xTaskCreatePinnedToCore(
  //   TempTask,
  //   "TempTask",
  //   4096,
  //   NULL,
  //   1,
  //   &tempTaskHandle,
  //   1  // run on core 1 (user app core)
  // );

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

  starttime = esp_timer_get_time();  //Record a start time for demonstration
  //yield();
  //ReadAnalogInputs();
  // yield();
  // Serial.print("ManualFieldToggle: ");
  //Serial.println(ManualFieldToggle);
  //Serial.print("HiLow: ");
  //Serial.println(HiLow);
  //Serial.print("NMEA0183Data: ");
  // Serial.println(NMEA0183Data);

  //Serial.print("AlternatorTemperatureLimitF: ");
  // Serial.println(AlternatorTemperatureLimitF);

  // ReadVEData();  //read Data from Victron VeDirect

  //AdjustSic450();
  // UpdateDisplay();
  // FaultCheck();


  if (powersavemode == 1) {
    /// Replace this later with control from Ignition Signal
    //This turns wifi off every 60 seconds just to prove that it will be a power savings
    if (millis() - previousMillisZZ >= intervalZZ) {
      previousMillisZZ = millis();
      if (getCpuFrequencyMhz() == 240) {
        // try to shut downn wifi
        setCpuFrequencyMhz(10);
        WiFi.mode(WIFI_OFF);
        Serial.println("wifi has been turned off");
      } else {
        // turn it back on
        setCpuFrequencyMhz(240);
        initWiFi();  // 2nd time using this, first is in setup
        //Serial.println("Wifi is reinitialized");
      }
      Freq = getCpuFrequencyMhz();
      Serial.print("CPU Freq = ");
      Serial.println(Freq);
      Serial.println();
      Serial.println();
    }
  }

  SendWifiData();                          // break this out later to a different timed blink without delay thing

  if (millis() - prev_millis743 > 5000) {  // every 5 seconds check CAN network (this might need adjustment)
    if (NMEA2KData == 1) {
      NMEA2000.ParseMessages();
    }
    prev_millis743 = millis();
  }

//Blink LED on and off every X seconds
  if (millis() - previousMillisBLINK >= intervalBLINK) {
  digitalWrite(2, (ledState = !ledState));
  previousMillisBLINK = millis();
}


  endtime = esp_timer_get_time();  //Record a start time for demonstration
  LoopTime = (endtime - starttime);
  //Serial.println(LoopTime);
  if (LoopTime > MaximumLoopTime) {
    MaximumLoopTime = LoopTime;
  }

  if (millis() - prev_millis7888 > 3000) {  // every 2 seconds reset the maximum loop time
    MaximumLoopTime = 0;
    prev_millis7888 = millis();
  }
  //Serial.println(MaximumLoopTime);
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
