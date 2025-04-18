
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

//delete this later ?
int powersavemode = 0;
int INADisconnected = 0;
int WifiHeartBeat = 0;
int VeTime = 0;
int SendWifiTime = 0;
int AnalogReadTime = 0;   // this is the present
int AnalogReadTime2 = 0;  // this is the maximum ever


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
int NMEA2KData = 0;                       // doesn't do anything yet

int previousMillisZZ = 0;  // Temporary, for getting power consumption down
int intervalZZ = 60000;    // Turn WiFi on and off every 60 seconds (until there's an Ignition signal controlling it)
uint32_t Freq = 0;         // ESP32 switching Frequency in case we want to report it for debugging
int previousMillisBLINK;   // used for heartbeat blinking LED test can delete later
int intervalBLINK = 1000;  // used for heartbeat blinking LED test can delete later
bool ledState;             // used for heartbeat blinking LED test can delete later

//Variables to store measurements
float ShuntVoltage_mV;  // Battery shunt voltage from INA228
float Bcur;             // battery shunt current from INA228
float IBV;              // Ina 228 battery voltage
float DutyCycle;        // SIC outout %
float vvout;            // SIC output volts
float iiout;            // SIC output current
float AlternatorTemperatureF = NAN;
TaskHandle_t tempTaskHandle = NULL;  // make a separate cpu task for temp reading because it's so slow
float VictronVoltage = 0;            // battery reading from VeDirect
float HeadingNMEA = 0;               // Just here to test NMEA functionality

// ADS1115
int16_t Raw = 0;
float Channel0V, Channel1V, Channel2V, Channel3V;
float BatteryV, MeasuredAmps, RPM;
int ADS1115Disconnected = 0;

// variables used to show how long each loop takes
uint64_t starttime;
uint64_t endtime;
int LoopTime;             // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int WifiStrength;         // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int MaximumLoopTime;      // must not use unsigned long becasue cant run String() on an unsigned long and that's done by the wifi code
int prev_millis7888 = 0;  // used to reset the meximum loop time

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

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

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

  // A debug LittleFS contents
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed. Files won't be accessible.");
    return;
  }
  // List all files
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  if (!file) {
    Serial.println("No files found in LittleFS!");
  } else {
    Serial.println("Files found in LittleFS:");
    while (file) {
      String fileName = file.name();
      size_t fileSize = file.size();
      Serial.print("  ");
      Serial.print(fileName);
      Serial.print(" (");
      Serial.print(fileSize);
      Serial.println(" bytes)");
      file = root.openNextFile();
    }
  }
  // Try to open specific files we need
  if (LittleFS.exists("/index.html")) {
    Serial.println("index.html exists!");
  } else {
    Serial.println("index.html NOT found!");
  }
  if (LittleFS.exists("/uPlot.min.css")) {
    Serial.println("uPlot.min.css exists!");
  } else {
    Serial.println("uPlot.min.css NOT found!");
  }
  if (LittleFS.exists("/uPlot.iife.min.js")) {
    Serial.println("uPlot.iife.min.js exists!");
  } else {
    Serial.println("uPlot.iife.min.js NOT found!");
  }


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
    writeFile2("/TemperatureLimitF.txt", stringAlternatorTemperatureLimitF.c_str());
  }
  bool ManualVfileexists = LittleFS.exists("/ManualVoltage.txt");
  if (!ManualVfileexists) {
    String stringManualVV = String(ManualVoltageTarget, 2);
    writeFile2("/ManualVoltage.txt", stringManualVV.c_str());
  }
  bool FCVfileexists = LittleFS.exists("/FullChargeVoltage.txt");
  if (!FCVfileexists) {
    String stringFCV = String(ChargingVoltageTarget, 2);
    writeFile2("/FullChargeVoltage.txt", stringFCV.c_str());
  }
  bool TAfileexists = LittleFS.exists("/TargetAmpz.txt");
  if (!TAfileexists) {
    String stringTA = String(TargetAmps);
    writeFile2("/TargetAmpz.txt", stringTA.c_str());
  }
  bool Freqfileexists = LittleFS.exists("/SwitchingFrequency.txt");
  if (!Freqfileexists) {
    String stringFreq = String(fffr);
    writeFile2("/SwitchingFrequency.txt", stringFreq.c_str());
  }
  bool TFVfileexists = LittleFS.exists("/TargetFloatVoltage1.txt");
  if (!TFVfileexists) {
    String stringTFV = String(TargetFloatVoltage);
    writeFile2("/TargetFloatVoltage1.txt", stringTFV.c_str());
  }
  bool intervalfileexists = LittleFS.exists("/interval1.txt");
  if (!intervalfileexists) {
    String stringInterval = String(interval);
    writeFile2("/interval1.txt", stringInterval.c_str());
  }
  bool Fintervalfileexists = LittleFS.exists("/FieldAdjustmentInterval1.txt");
  if (!Fintervalfileexists) {
    String stringFAI = String(FieldAdjustmentInterval);
    writeFile2("/FieldAdjustmentInterval1.txt", stringFAI.c_str());
  }
  bool MFTexists = LittleFS.exists("/ManualFieldToggle1.txt");
  if (!MFTexists) {
    String stringMFT = String(ManualFieldToggle);
    writeFile2("/ManualFieldToggle1.txt", stringMFT.c_str());
  }
  bool SCOexists = LittleFS.exists("/SwitchControlOverride1.txt");
  if (!SCOexists) {
    String stringSCO = String(SwitchControlOverride);
    writeFile2("/SwitchControlOverride1.txt", stringSCO.c_str());
  }
  bool FFexists = LittleFS.exists("/ForceFloat1.txt");
  if (!FFexists) {
    String stringFF = String(ForceFloat);
    writeFile2("/ForceFloat1.txt", stringFF.c_str());
  }
  bool OOexists = LittleFS.exists("/OnOff1.txt");
  if (!OOexists) {
    String stringOO = String(OnOff);
    writeFile2("/OnOff1.txt", stringOO.c_str());
  }
  bool HLexists = LittleFS.exists("/HiLow1.txt");
  if (!HLexists) {
    String stringHL = String(HiLow);
    writeFile2("/HiLow1.txt", stringHL.c_str());
  }
  bool LHexists = LittleFS.exists("/LimpHome1.txt");
  if (!LHexists) {
    String stringLH = String(LimpHome);
    writeFile2("/LimpHome1.txt", stringLH.c_str());
  }
  bool VDexists = LittleFS.exists("/VeData1.txt");
  if (!VDexists) {
    String stringVD = String(VeData);
    writeFile2("/VeData1.txt", stringVD.c_str());
  }
  bool N0exists = LittleFS.exists("/NMEA0183Data1.txt");
  if (!N0exists) {
    String stringN0 = String(NMEA0183Data);
    writeFile2("/NMEA0183Data1.txt", stringN0.c_str());
  }
  bool N2exists = LittleFS.exists("/NMEA2KData1.txt");
  if (!N2exists) {
    String stringN2 = String(NMEA2KData);
    writeFile2("/NMEA2KData1.txt", stringN2.c_str());
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
    NMEA2000.ParseMessages();
    prev_millis743 = millis();
  }
  if (millis() - previousMillisBLINK >= intervalBLINK) {  // every 1 seconds, turn LED on or off
    if (ledState == LOW) {
      ledState = HIGH;

    } else {
      ledState = LOW;
    }
    digitalWrite(2, ledState);
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

void AdjustSic450() {
  if (Ignition == 1 && OnOff == 1) {

    if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust SIC450 every half second
      digitalWrite(4, SIC450Enabler);                          // Enable the SIC450

      //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
      if (ManualFieldToggle == 0) {
        if (MeasuredAmps < TargetAmps && vout < (14 - interval)) {  // if amps are low, add field (if field limit not exceeded
          vout = vout + interval;
        }
        if (MeasuredAmps > TargetAmps && vout > (MinimumFieldVoltage + interval)) {  // if amps are high and field isn't too low, drop field
          vout = vout - interval;
        }
        // HAVE TO MAKE SURE THESE VALUES DON'T GET TOO LOW FOR SIC450 COMMAND VALIDITY.   THIS LOGIC IS ALSO NOT GREAT IF INTERVAL GETS BIG FOR ANY REASON
        if (AlternatorTemperatureF > AlternatorTemperatureLimitF && vout > (MinimumFieldVoltage + 2 * interval)) {
          vout = vout - (2 * interval);  // if no *2, the adjustments offset, so this makes the Temp correction win
        }
        if (BatteryV > ChargingVoltageTarget && vout > (MinimumFieldVoltage + 3 * interval)) {
          vout = vout - (3 * interval);  // if no *3, the adjustments offset, so this makes the Temp correction win
        }
        prev_millis22 = millis();
      } else {
        vout = ManualVoltageTarget;
      }

      // adjust limits, not discussed in datasheeet but at least some of these (VOUT_SCALE_LOOP, OV_FAULT_LIMIT are necessary for stability
      // sic45x.setPowerGoodOn(vout * 0.9);        // .9    Try deleting this later
      // sic45x.setPowerGoodOff(vout * 0.85);      // .85   Try deleting this later
      sic45x.setVoutOvFaultLimit(vout * 1.15);  //    I think this one is required
                                                //  sic45x.setVoutOvWarnLimit(vout * 1.1);    // 110  Try deleting this later
                                                // sic45x.setVoutUvWarnLimit(vout * 0.9);    // .9 Try delting this later
                                                //  sic45x.setVoutUvFaultLimit(vout * 0.8);   // .8 Try deleting this later
                                                //  sic45x.setVoutMarginLow(vout * 0.95);     //.95   Try delting this later
                                                //  sic45x.setVoutMarginHigh(vout * 1.05);    //105 Try deleting this later

      //VOUT_SCALE_LOOP, according to Vishay, this matters.   Most of my early testing was between 5 and 12V, so I was possibly just in blissful ignorance before learning this
      if (vout < 1.8) {
        sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_0V3_1V8);
      }
      if (vout >= 1.8 && vout < 3.3) {
        sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_1V8_3V3);
      }
      if (vout >= 3.3 && vout < 5) {
        sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_3V3_5V0);
      }
      if (vout >= 5) {
        sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
      }

      sic45x.setFrequencySwitch(fffr);  //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
      sic45x.setVoutCommand(vout);      // Update the field voltage
      sic45x.sendClearFaults();         // may or may not be a good idea, testing will determine
    }
  } else {
    sic45x.setOperation(SIC45X_OPERATION_ON_OFF_DISABLED);  // Output is disabled
    vout = MinimumFieldVoltage;                             // start over from a low field voltage when it comes time to turn back on
  }
}
void FaultCheck() {
  if (millis() - prev_millis4 > 20000 && FaultCheckToggle == 1) {  // every 20 seconds check faults
    // Serial.print("getReadVin:");
    // Serial.print(sic45x.getReadVin());
    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadIin:");
    // Serial.print(sic45x.getReadIin());
    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadVout:");
    // Serial.print(sic45x.getReadVout());

    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadIout:");
    // Serial.print(sic45x.getReadIout());

    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadTemperature:");
    // Serial.print(sic45x.getReadTemperature());
    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadDutyCycle:");
    // Serial.print(sic45x.getReadDutyCycle());

    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadPout:");
    // Serial.print(sic45x.getReadPout());
    // Serial.print("\t");  // prints a tab
    // Serial.print("getReadPin:");
    // Serial.print(sic45x.getReadPin());
    // Serial.print("\t");  // prints a tab

    sic45x.printStatusByte();
    sic45x.printStatusWord();
    sic45x.printStatusVout();
    sic45x.printStatusIout();
    sic45x.printStatusInput();
    sic45x.printStatusTemperature();
    sic45x.printStatusCml();
    sic45x.printStatusMfrSpecific();
    Serial.println();
    prev_millis4 = millis();
  }
}


void ReadAnalogInputs() {


  if (millis() - lastINARead >= 900) {  // could go down to 600 here
    if (INADisconnected == 0) {
      //Serial.println();
      //Serial.print("INA228 Battery Voltage: ");
      int start33 = micros();  // Start timing analog input reading

      lastINARead = millis();
      IBV = INA.getBusVoltage();
      // Serial.println(IBV);
      // Serial.print("INA228 Battery Bcur (Amps): ");
      ShuntVoltage_mV = INA.getShuntVoltage_mV();
      Bcur = ShuntVoltage_mV * 10;
      //Serial.print(Bcur);
      //Serial.println();

      int end33 = micros();               // End timing
      AnalogReadTime2 = end33 - start33;  // Store elapsed time
      if (AnalogReadTime2 > AnalogReadTime) {
        AnalogReadTime = AnalogReadTime2;
      }
    }
  }

  //ADS1115 reading is based on trigger→wait→read   so as to not waste time.  That is way the below is so complicated
  if (ADS1115Disconnected != 0) return;
  Serial.println("theADS1115 was not connected and triggered a return");

  unsigned long now = millis();

  switch (adsState) {
    case ADS_IDLE:
      switch (adsCurrentChannel) {
        case 0: adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); break;
        case 1: adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); break;
        case 2: adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); break;
        case 3: adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); break;
      }
      adc.triggerConversion();
      adsStartTime = now;
      adsState = ADS_WAITING_FOR_CONVERSION;
      break;

    case ADS_WAITING_FOR_CONVERSION:
      if (now - adsStartTime >= ADSConversionDelay) {
        Raw = adc.getConversion();

        switch (adsCurrentChannel) {
          case 0:
            Channel0V = Raw / 32768.0 * 6.144 * 20.24291;
            BatteryV = Channel0V;
            if (BatteryV > 14.5) {
              ChargingVoltageTarget = TargetFloatVoltage;
            }
            break;
          case 1:
            Channel1V = Raw / 32768.0 * 6.144 * 2;
            MeasuredAmps = (2.5 - Channel1V) * 80;
            break;
          case 2:
            Channel2V = Raw / 32768.0 * 6.144 * 2133.2 * 2;
            RPM = Channel2V;
            break;
          case 3:
            Channel3V = Raw / 32768.0 * 6.144 * 833;
            break;
        }

        adsCurrentChannel = (adsCurrentChannel + 1) % 4;
        adsState = ADS_IDLE;

        if (adsCurrentChannel == 0) {
          prev_millis3 = now;  // finished full cycle
        }
      }
      break;
  }
}

void TempTask(void *parameter) {
  for (;;) {
    // Step 1: Trigger a conversion
    sensors.requestTemperaturesByAddress(tempDeviceAddress);

    // Step 2: Wait for conversion to complete while other things run
    vTaskDelay(pdMS_TO_TICKS(9000));  // This is the spacing between reads

    // Step 3: Read the completed result
    uint8_t scratchPad[9];
    if (sensors.readScratchPad(tempDeviceAddress, scratchPad)) {
      int16_t raw = (scratchPad[1] << 8) | scratchPad[0];
      float tempC = raw / 16.0;
      AlternatorTemperatureF = tempC * 1.8 + 32.0;
    } else {
      AlternatorTemperatureF = NAN;
      Serial.println("Temp read failed");
    }
    Serial.printf("Temp: %.2f °F at %lu ms\n", AlternatorTemperatureF, millis());


    // Immediately loop again — next conversion starts right now
  }
}



void UpdateDisplay() {
  if (millis() - prev_millis66 > 3000) {  // update display every 3 seconds
    //  PrintData(); // this function prints everything available to serial monitor
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Vlts: ");
    display.setCursor(35, 0);
    display.println(BatteryV, 2);

    display.setCursor(79, 0);
    display.println("R: ");
    display.setCursor(90, 0);
    display.println(RPM, 0);

    display.setCursor(0, 11);
    display.println("Acur:");
    display.setCursor(35, 11);
    display.println(MeasuredAmps, 1);

    display.setCursor(79, 11);
    display.println("VV: ");
    display.setCursor(90, 11);
    display.println(VictronVoltage, 2);

    display.setCursor(0, 22);
    display.println("Temp: ");
    display.setCursor(35, 22);
    display.println(AlternatorTemperatureF, 1);

    display.setCursor(79, 22);
    display.println("t: ");
    display.setCursor(90, 22);
    display.println("extra");

    display.setCursor(0, 33);
    display.println("PWM%:");
    display.setCursor(35, 33);
    display.println(DutyCycle, 1);  //String(val, decimalPlaces)

    display.setCursor(79, 33);
    display.println("H: ");
    display.setCursor(90, 33);
    display.println(HeadingNMEA);

    display.setCursor(0, 44);
    display.println("Vout:");
    display.setCursor(35, 44);
    display.println(vvout, 2);  //String(val, decimalPlaces)

    display.setCursor(0, 55);
    display.println("Bcur:");
    display.setCursor(35, 55);
    display.println(Bcur, 1);  //String(val, decimalPlaces)

    display.display();
    prev_millis66 = millis();
  }
}
void ReadVEData() {
  if (VeData == 1) {

    if (millis() - prev_millis33 > 1000) {  // read VE data every 1 second

      int start1 = micros();  // Start timing VeData
      while (Serial2.available()) {
        myve.rxData(Serial2.read());
        for (int i = 0; i < myve.veEnd; i++) {
          if (strcmp(myve.veName[i], "V") == 0) {
            VictronVoltage = (atof(myve.veValue[i]) / 1000);
          }
          // if (strcmp(myve.veName[i], "I") == 0) {
          //   VictronCurrent = (atof(myve.veValue[i]) / 1000);
          // }
        }
        yield();
      }
      //PrintData(); //This prints all victron data received to the serial monitor, put this in "Loop" for debugging if needed
      int end1 = micros();     // End timing
      VeTime = end1 - start1;  // Store elapsed time

      prev_millis33 = millis();
    }
  }
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
void SystemTime(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  uint16_t SystemDate;
  double SystemTime;
  tN2kTimeSource TimeSource;

  if (ParseN2kSystemTime(N2kMsg, SID, SystemDate, SystemTime, TimeSource)) {
    OutputStream->println("System time:");
    PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", SystemDate, 0, true);
    PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SystemTime, 0, true);
    OutputStream->print("  time source: ");
    PrintN2kEnumType(TimeSource, OutputStream);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void Rudder(const tN2kMsg &N2kMsg) {
  unsigned char Instance;
  tN2kRudderDirectionOrder RudderDirectionOrder;
  double RudderPosition;
  double AngleOrder;

  if (ParseN2kRudder(N2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder)) {
    PrintLabelValWithConversionCheckUnDef("Rudder: ", Instance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  position (deg): ", RudderPosition, &RadToDeg, true);
    OutputStream->print("  direction order: ");
    PrintN2kEnumType(RudderDirectionOrder, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  angle order (deg): ", AngleOrder, &RadToDeg, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void Heading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double Heading;
  double Deviation;
  double Variation;

  if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, HeadingReference)) {
    OutputStream->println("Heading:");
    PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    OutputStream->print("  reference: ");
    PrintN2kEnumType(HeadingReference, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  Heading (deg): ", Heading, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ", Deviation, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  Variation (deg): ", Variation, &RadToDeg, true);

    HeadingNMEA = Heading;  //Turn this back on!!
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void COGSOG(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double COG;
  double SOG;

  if (ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG)) {
    OutputStream->println("COG/SOG:");
    PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    OutputStream->print("  reference: ");
    PrintN2kEnumType(HeadingReference, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  COG (deg): ", COG, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  SOG (m/s): ", SOG, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void GNSS(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  uint16_t DaysSince1970;
  double SecondsSinceMidnight;
  double Latitude;
  double Longitude;
  double Altitude;
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;

  if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                   Latitude, Longitude, Altitude,
                   GNSStype, GNSSmethod,
                   nSatellites, HDOP, PDOP, GeoidalSeparation,
                   nReferenceStations, ReferenceStationType, ReferenceSationID,
                   AgeOfCorrection)) {
    OutputStream->println("GNSS info:");
    PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", DaysSince1970, 0, true);
    PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SecondsSinceMidnight, 0, true);
    PrintLabelValWithConversionCheckUnDef("  latitude: ", Latitude, 0, true, 9);
    PrintLabelValWithConversionCheckUnDef("  longitude: ", Longitude, 0, true, 9);
    PrintLabelValWithConversionCheckUnDef("  altitude: (m): ", Altitude, 0, true);
    OutputStream->print("  GNSS type: ");
    PrintN2kEnumType(GNSStype, OutputStream);
    OutputStream->print("  GNSS method: ");
    PrintN2kEnumType(GNSSmethod, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  satellite count: ", nSatellites, 0, true);
    PrintLabelValWithConversionCheckUnDef("  HDOP: ", HDOP, 0, true);
    PrintLabelValWithConversionCheckUnDef("  PDOP: ", PDOP, 0, true);
    PrintLabelValWithConversionCheckUnDef("  geoidal separation: ", GeoidalSeparation, 0, true);
    PrintLabelValWithConversionCheckUnDef("  reference stations: ", nReferenceStations, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void GNSSSatsInView(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kRangeResidualMode Mode;
  uint8_t NumberOfSVs;
  tSatelliteInfo SatelliteInfo;

  if (ParseN2kPGNSatellitesInView(N2kMsg, SID, Mode, NumberOfSVs)) {
    OutputStream->println("Satellites in view: ");
    OutputStream->print("  mode: ");
    OutputStream->println(Mode);
    OutputStream->print("  number of satellites: ");
    OutputStream->println(NumberOfSVs);
    for (uint8_t i = 0; i < NumberOfSVs && ParseN2kPGNSatellitesInView(N2kMsg, i, SatelliteInfo); i++) {
      OutputStream->print("  Satellite PRN: ");
      OutputStream->println(SatelliteInfo.PRN);
      PrintLabelValWithConversionCheckUnDef("    elevation: ", SatelliteInfo.Elevation, &RadToDeg, true, 1);
      PrintLabelValWithConversionCheckUnDef("    azimuth:   ", SatelliteInfo.Azimuth, &RadToDeg, true, 1);
      PrintLabelValWithConversionCheckUnDef("    SNR:       ", SatelliteInfo.SNR, 0, true, 1);
      PrintLabelValWithConversionCheckUnDef("    residuals: ", SatelliteInfo.RangeResiduals, 0, true, 1);
      OutputStream->print("    status: ");
      OutputStream->println(SatelliteInfo.UsageStatus);
    }
  }
}
//*****************************************************************************
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg) {
  unsigned char BatInstance;
  tN2kBatType BatType;
  tN2kBatEqSupport SupportsEqual;
  tN2kBatNomVolt BatNominalVoltage;
  tN2kBatChem BatChemistry;
  double BatCapacity;
  int8_t BatTemperatureCoefficient;
  double PeukertExponent;
  int8_t ChargeEfficiencyFactor;

  if (ParseN2kBatConf(N2kMsg, BatInstance, BatType, SupportsEqual, BatNominalVoltage, BatChemistry, BatCapacity, BatTemperatureCoefficient, PeukertExponent, ChargeEfficiencyFactor)) {
    PrintLabelValWithConversionCheckUnDef("Battery instance: ", BatInstance, 0, true);
    OutputStream->print("  - type: ");
    PrintN2kEnumType(BatType, OutputStream);
    OutputStream->print("  - support equal.: ");
    PrintN2kEnumType(SupportsEqual, OutputStream);
    OutputStream->print("  - nominal voltage: ");
    PrintN2kEnumType(BatNominalVoltage, OutputStream);
    OutputStream->print("  - chemistry: ");
    PrintN2kEnumType(BatChemistry, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  - capacity (Ah): ", BatCapacity, &CoulombToAh, true);
    PrintLabelValWithConversionCheckUnDef("  - temperature coefficient (%): ", BatTemperatureCoefficient, 0, true);
    PrintLabelValWithConversionCheckUnDef("  - peukert exponent: ", PeukertExponent, 0, true);
    PrintLabelValWithConversionCheckUnDef("  - charge efficiency factor (%): ", ChargeEfficiencyFactor, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void DCStatus(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char DCInstance;
  tN2kDCType DCType;
  unsigned char StateOfCharge;
  unsigned char StateOfHealth;
  double TimeRemaining;
  double RippleVoltage;
  double Capacity;

  if (ParseN2kDCStatus(N2kMsg, SID, DCInstance, DCType, StateOfCharge, StateOfHealth, TimeRemaining, RippleVoltage, Capacity)) {
    OutputStream->print("DC instance: ");
    OutputStream->println(DCInstance);
    OutputStream->print("  - type: ");
    PrintN2kEnumType(DCType, OutputStream);
    OutputStream->print("  - state of charge (%): ");
    OutputStream->println(StateOfCharge);
    OutputStream->print("  - state of health (%): ");
    OutputStream->println(StateOfHealth);
    OutputStream->print("  - time remaining (h): ");
    OutputStream->println(TimeRemaining / 60);
    OutputStream->print("  - ripple voltage: ");
    OutputStream->println(RippleVoltage);
    OutputStream->print("  - capacity: ");
    OutputStream->println(Capacity);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
void Speed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SOW;
  double SOG;
  tN2kSpeedWaterReferenceType SWRT;

  if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT)) {
    OutputStream->print("Boat speed:");
    PrintLabelValWithConversionCheckUnDef(" SOW:", N2kIsNA(SOW) ? SOW : msToKnots(SOW));
    PrintLabelValWithConversionCheckUnDef(", SOG:", N2kIsNA(SOG) ? SOG : msToKnots(SOG));
    OutputStream->print(", ");
    PrintN2kEnumType(SWRT, OutputStream, true);
  }
}
//*****************************************************************************
void WaterDepth(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;

  if (ParseN2kWaterDepth(N2kMsg, SID, DepthBelowTransducer, Offset)) {
    if (N2kIsNA(Offset) || Offset == 0) {
      PrintLabelValWithConversionCheckUnDef("Depth below transducer", DepthBelowTransducer);
      if (N2kIsNA(Offset)) {
        OutputStream->println(", offset not available");
      } else {
        OutputStream->println(", offset=0");
      }
    } else {
      if (Offset > 0) {
        OutputStream->print("Water depth:");
      } else {
        OutputStream->print("Depth below keel:");
      }
      if (!N2kIsNA(DepthBelowTransducer)) {
        OutputStream->println(DepthBelowTransducer + Offset);
      } else {
        OutputStream->println(" not available");
      }
    }
  }
}
//*****************************************************************************
void Attitude(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double Yaw;
  double Pitch;
  double Roll;

  if (ParseN2kAttitude(N2kMsg, SID, Yaw, Pitch, Roll)) {
    OutputStream->println("Attitude:");
    PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    PrintLabelValWithConversionCheckUnDef("  Yaw (deg): ", Yaw, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  Pitch (deg): ", Pitch, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  Roll (deg): ", Roll, &RadToDeg, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}
//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;

  // Find handler
  OutputStream->print("In Main Handler: ");
  OutputStream->println(N2kMsg.PGN);
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++)
    ;

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

//WIFI STUFF
//void notFound(AsyncWebServerRequest *request) {
//  request->send(404, "text/plain", "Not found");
//}

String readFile(fs::FS &fs, const char *path) {
  // Serial.printf("Reading file: %s\r\n", path);
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
  // Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  // Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing and triggered a return");
    return;
  }
  if (file.print(message)) {
    //  Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  delay(2);  // Make sure the CREATE and LASTWRITE times are different Delte later?
  file.close();
}
void writeFile2(const char *path, const char *message) {
  //  Serial.printf("Writing file: %s\n", path);

  File file = LittleFS.open(path, "w");
  if (!file) {
    Serial.println("Failed to open file for writing and triggered a return");
    return;
  }
  if (file.print(message)) {
    //   Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  delay(2);  // Make sure the CREATE and LASTWRITE times are different Delte later?
  file.close();
}

//The processor() is responsible for searching for placeholders in the HTML text and replacing them with actual values saved on LittleFS.
String processor(const String &var) {
  //Serial.println(var);
  //First, the "settings"
  if (var == "TemperatureLimitF") {
    return readFile(LittleFS, "/TemperatureLimitF.txt");
  } else if (var == "ManualVoltage") {
    return readFile(LittleFS, "/ManualVoltage.txt");
  } else if (var == "FullChargeVoltage") {
    return readFile(LittleFS, "/FullChargeVoltage.txt");
  } else if (var == "TargetAmpz") {
    return readFile(LittleFS, "/TargetAmpz.txt");
  } else if (var == "SwitchingFrequency") {
    return readFile(LittleFS, "/SwitchingFrequency.txt");
  } else if (var == "TargetFloatVoltage1") {
    return readFile(LittleFS, "/TargetFloatVoltage1.txt");
  } else if (var == "interval1") {
    return readFile(LittleFS, "/interval1.txt");
  } else if (var == "FieldAdjustmentInterval1") {
    return readFile(LittleFS, "/FieldAdjustmentInterval1.txt");
  } else if (var == "ManualFieldToggle1") {
    return readFile(LittleFS, "/ManualFieldToggle1.txt");
  } else if (var == "SwitchControlOverride1") {
    return readFile(LittleFS, "/SwitchControlOverride1.txt");
  } else if (var == "ForceFloat1") {
    return readFile(LittleFS, "/ForceFloat1.txt");
  } else if (var == "OnOff1") {
    return readFile(LittleFS, "/OnOff1.txt");
  } else if (var == "HiLow1") {
    return readFile(LittleFS, "/HiLow1.txt");
  } else if (var == "LimpHome1") {
    return readFile(LittleFS, "/LimpHome1.txt");
  } else if (var == "VeData1") {
    return readFile(LittleFS, "/VeData1.txt");
  } else if (var == "NMEA0183Data1") {
    return readFile(LittleFS, "/NMEA0183Data1.txt");
  } else if (var == "NMEA2KData1") {
    return readFile(LittleFS, "/NMEA2KData1.txt");
  }

  //Then, the live sensor readings
  else if (var == "ALTERNATORTEMPERATUREF") {
    return String(AlternatorTemperatureF);
  } else if (var == "DUTYCYCLE") {
    return String(DutyCycle);
  } else if (var == "BATTERYV") {
    return String(BatteryV);
  } else if (var == "MEASA") {
    return String(MeasuredAmps);
  } else if (var == "RPMM") {
    return String(RPM);
  } else if (var == "ADSCH3VLTS") {
    return String(Channel3V);
  } else if (var == "IBVV") {
    return String(IBV);
  } else if (var == "BCURR") {
    return String(Bcur);
  } else if (var == "VVOLT") {
    return String(VictronVoltage);
  } else if (var == "GPSH") {
    return String(HeadingNMEA);
  } else if (var == "FIELDVOLTS") {
    return String(vvout);
  } else if (var == "LOOPTIME") {
    return String(LoopTime);
  } else if (var == "MAXIMUMLOOPTIME") {
    return String(MaximumLoopTime);
  } else if (var == "FIELDAMPS") {
    return String(iiout);
  } else if (var == "WIFISTRENGTH") {
    return String(WifiStrength);
  } else if (var == "WIFIHEARTBEAT") {
    return String(WifiHeartBeat);
  } else if (var == "SENDWIFITIME") {
    return String(SendWifiTime);
  } else if (var == "ANALOGREADTIME") {
    return String(AnalogReadTime);
  } else if (var == "VETIME") {
    return String(VeTime);
  }
  return String();
}


// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    //  Serial.print('.');
    delay(500);
  }
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed and triggered a return!");
    return;
  }
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI (lower absolute value is better!): ");
  Serial.println(WiFi.RSSI());
}



// April 2025, this code was updated to avoid the creation of Strings for sending over wifi
void SendWifiData() {
  if (millis() - prev_millis5 > webgaugesinterval) {
    WifiStrength = WiFi.RSSI();
    WifiHeartBeat++;

    if (WifiStrength >= -70) {
      int start66 = micros();  // Start timing the wifi section
      char payload[1024];      // safe size for current and future variables, supposedly.  >1460 bytes exceeds single packet size, avoid in realtime app's

      snprintf(payload, sizeof(payload),
               "{"
               "\"AlternatorTemperatureF\":%d,"
               "\"DutyCycle\":%d,"
               "\"BatteryV\":%d,"
               "\"MeasuredAmps\":%d,"
               "\"RPM\":%d,"
               "\"Channel3V\":%d,"
               "\"IBV\":%d,"
               "\"Bcur\":%d,"
               "\"VictronVoltage\":%d,"
               "\"LoopTime\":%d,"
               "\"WifiStrength\":%d,"
               "\"WifiHeartBeat\":%d,"
               "\"SendWifiTime\":%d,"
               "\"AnalogReadTime\":%d,"
               "\"VeTime\":%d,"
               "\"MaximumLoopTime\":%d,"
               "\"HeadingNMEA\":%d,"
               "\"vvout\":%d,"
               "\"iiout\":%d,"
               "\"TemperatureLimitF\":%d,"
               "\"FullChargeVoltage\":%d,"
               "\"TargetAmpz\":%d,"
               "\"TargetFloatVoltage1\":%d,"
               "\"SwitchingFrequency\":%d,"
               "\"interval1\":%d,"
               "\"FieldAdjustmentInterval1\":%d,"
               "\"ManualVoltage\":%d,"
               "\"SwitchControlOverride1\":%d,"
               "\"OnOff1\":%d,"
               "\"ManualFieldToggle1\":%d,"
               "\"HiLow1\":%d,"
               "\"LimpHome1\":%d,"
               "\"VeData1\":%d,"
               "\"NMEA0183Data1\":%d,"
               "\"NMEA2KData1\":%d"
               "}",
               (int)AlternatorTemperatureF,
               (int)DutyCycle,
               (int)(BatteryV * 100),
               (int)(MeasuredAmps * 10),
               (int)RPM,
               (int)(Channel3V * 100),
               (int)(IBV * 100),
               (int)(Bcur * 10),
               (int)(VictronVoltage * 100),
               (int)LoopTime,
               (int)WifiStrength,
               (int)WifiHeartBeat,
               (int)SendWifiTime,
               (int)AnalogReadTime,
               (int)VeTime,
               (int)MaximumLoopTime,
               (int)HeadingNMEA,
               (int)(vvout * 100),
               (int)(iiout * 10),

               (int)AlternatorTemperatureLimitF,
               (int)(ChargingVoltageTarget * 100),
               (int)TargetAmps,
               (int)(TargetFloatVoltage * 100),
               (int)fffr,
               (int)(interval * 100),
               (int)(FieldAdjustmentInterval * 100),
               (int)(ManualVoltageTarget * 100),
               (int)SwitchControlOverride,
               (int)OnOff,
               (int)ManualFieldToggle,
               (int)HiLow,
               (int)LimpHome,
               (int)VeData,
               (int)NMEA0183Data,
               (int)NMEA2KData);

      events.send(payload, "BulkData");
      SendWifiTime = micros() - start66;  // Calculate WiFi Send Time
    }

    prev_millis5 = millis();
  }
}





// void debounce() {
//   int reading = digitalRead(buttonPin);

//   // If the switch changed, due to noise or pressing:
//   if (reading != lastButtonState) {
//     // reset the debouncing timer
//     lastDebounceTime = millis();
//   }

//   if ((millis() - lastDebounceTime) > debounceDelay) {
//     // whatever the reading is at, it's been there for longer than the debounce
//     // delay, so take it as the actual current state:

//     // if the button state has changed:
//     if (reading != buttonState) {
//       buttonState = reading;

//       // only toggle the LED if the new button state is HIGH
//       if (buttonState == HIGH) {
//         ledState = !ledState;
//       }
//     }
//   }

//   // set the LED:
//   digitalWrite(ledPin, ledState);

//   // save the reading. Next time through the loop, it'll be the lastButtonState:
//   lastButtonState = reading;

// }

// This is needed becasuse JavaScript allows NaN, but JSON does not—it only allows null, numbers, booleans, strings, arrays, and objects
//So when  ESP32 sends a payload with NaN, the browser can’t parse it, and the whole stream fails
String safeFloat(float value) {
  if (isnan(value)) return "null";
  return String(value);
}