// This code works, but doens't have great SIC450 settings

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
//DONT MOVE THE BELOW 6 LINES AROUND, HAVE TO STAY IN ORDER!!
#include <Arduino.h>                  // maybe not needed, was in NMEA2K example I copied
#define ESP32_CAN_RX_PIN GPIO_NUM_16  // If you use ESP32 and do not have TX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_TX_PIN GPIO_NUM_17  // If you use ESP32 and do not have RX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>  // questionably needed
#include <WiFi.h>
#include <AsyncTCP.h>           // for wifi stuff
#include <LittleFS.h>           // for wifi stuff
#include <ESPAsyncWebServer.h>  // for wifi stuff

// Settings

// Replace next 2 lines with your WiFi network credentials
const char *ssid = "MN2G";
const char *password = "5FENYC8PDW";

float TargetAmps = 55;
float TargetFloatVoltage = 13.9;
float TargetBulkVoltage = 14.5;
float ChargingVoltageTarget = 0;          // This is what the code really uses. It gets set to TargetFloatVoltage or TargetBulkVoltage later on
float interval = 0.1;                     // voltage step to adjust field target by, each time the loop runs.  Larger numbers = faster response, less stability
float FieldAdjustmentInterval = 500;      // The regulator field output is updated once every this many milliseconds
float AlternatorTemperatureLimitF = 150;  // the offset appears to be +40 to +50 to get max alternator metal temp, depending on temp sensor installation, so 150 here will produce a metal temp ~200F
int ManualFieldToggle = 0;                // set to 1 to enable manual control of regulator field output, helpful for debugging
float ManualVoltageTarget = 1;            // voltage target corresponding to the toggle above
int SwitchControlOverride = 1;            // set to 1 for web interface switches to override physical switch panel
int ForceFloat = 0;                       // set to 1 to force the float voltage to be the charging target
int OnOff = 1;                            // 0 is charger off, 1 is charger On
int HiLow = 1;                            // 0 will be a low setting, 1 a high setting
int LimpHome = 3;                         // 1 will set to limp home mode, whatever that gets set up to be
float vout = 1.1;                         // default field output voltage
int FaultCheckToggle = 0;                 // Set to 1 to get a serial print out over 20 seconds of error data, delete later
int resolution = 12;                      // for OneWire measurement
float fffr = 1200;                        // this is the switching frequency for SIC450
int VeDataOn = 0;                         // Set to 1 if VE serial data exists

// Used to blink ESPDuino built in Red LED every 2 seconds for reference as a heartbeat of sorts
int ledPin = 2;
bool ledState = LOW;
unsigned long currentMillissss = 0;
unsigned long previousMillissss = 0;
unsigned long interval22 = 2000;

//Variables to store measurements
float Raw;              //Each channel of ADS1115 is temporarily stored here, pre engineering units
float Channel0V;        // voltage divider is 20... same as BatteryV
float BatteryV;         // as read by ADS115
float Channel1V;        // voltage divider is 2, partial step towards getting MeasuredAmps
float MeasuredAmps;     // alternator output current
float Channel2V;        // voltage divider is 2---- wired internally to LM2907
float Channel3V;        // voltage divider is an empty socket for user installation
float ShuntVoltage_mV;  // Battery shunt voltage from INA228
float Bcur;             // battery shunt current from INA228
float IBV;              // Ina 228 battery voltage
float DutyCycle;        // SIC outout %
float vvout;            // SIC output volts
float iiout;            // SIC output current
float RPM;              // from LM2907
float AlternatorTemperatureF = 0;
float VictronVoltage = 0;  // battery reading from VeDirect
float HeadingNMEA = 0;     // Just here to test NMEA functionality

// variables used to show how long each loop takes
unsigned long starttime;
unsigned long endtime;
unsigned long LoopTime;

//"Blink without delay" style timer variables used to control how often differnet parts of code execute
static unsigned long prev_millis4;    // used to delay checking of faults in the SiC450
static unsigned long prev_millis66;   //used to delay the updating of the display
static unsigned long prev_millis22;   // used to delay sampling of sic450
static unsigned long prev_millis3;    // used to delay sampling of ADS1115 to every 2 seconds for example
static unsigned long prev_millis2;    // used to delay sampling of temp sensor to every 2 seconds for example
static unsigned long prev_millis33;   // used to delay sampling of Serial Data (ve direct)
static unsigned long prev_millis743;  // used to read NMEA2K Network Every 15 seconds
static unsigned long prev_millis5;    // used to initiate wifi data exchange

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
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 500;

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

// HTML web page
// Note: for simplicity, everything needed to build the web page is contained in the variable "index_html"
//It will eventually be more practical to have separate HTML, CSS and JavaScript files uploaded to the ESP32 filesystem
//and then reference them in the code
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML> 
         <html>
            <head>
               <title>X Engineering Alt Controller Browser Interface</title>
               <meta name="viewport" content="width=device-width, initial-scale=1">
               <script>
                  function submitMessage() {
                    setTimeout(function(){ document.location.reload(false); }, 500);   
                  }
               
      </script>
      </head>
      <body>
         <h2>Settings</h2>
         <style>
            table, th, td {
            border: 1px solid black;
            border-radius: 10px;
            }
         </style>
         <table>
            <tr>
               <td>
                  <form action="/get" target="hidden-form">
                     Alternator Temperature Limit (F) (current value %TemperatureLimitF%): <input type="text" name="TemperatureLimitF">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Bulk Voltage Target (current value %FullChargeVoltage%): <input type="number " name="FullChargeVoltage">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Target Amps (A) (current value %TargetAmpz%): <input type="number " name="TargetAmpz">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Controller Field Switching Frequency (hz) (current value %SwitchingFrequency%): <input type="number " name="SwitchingFrequency">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Float Voltage Target (V) (current value %TargetFloatVoltage1%): <input type="number " name="TargetFloatVoltage1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Field Adjustment Step Size (V) (current value %interval1%): <input type="number " name="interval1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Field Adjustment Interval (ms)) (current value %interval1%): <input type="number " name="FieldAdjustmentInterval1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
               </td>
               <td>
                  <form action="/get" target="hidden-form">
                     Manual Field Control Toggle (current value %ManualFieldToggle1%): <input type="number " name="ManualFieldToggle1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Manual Voltage Setpoint (V) (current value %ManualVoltage%): <input type="number " name="ManualVoltage">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Physical Switch Panel Override (current value %SwitchControlOverride1%): <input type="number " name="SwitchControlOverride1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     On/Off (current value %OnOff1%): <input type="number " name="OnOff1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Low(0)/High(1) Mode (current value %HiLow1%): <input type="number " name="HiLow1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
                  <hr>
                  <form action="/get" target="hidden-form">
                     Limp Home Mode (current value %LimpHome1%): <input type="number " name="LimpHome1">
                     <input type="submit" value="Submit" onclick="submitMessage()">
                  </form>
               </td>
            </tr>
         </table>
         <iframe style="display:none" name="hidden-form"></iframe>
         <hr/>
         <h2>Sensor and Output Data </h2>
         <style>
            table, th, td {
            border: 1px solid black;
            border-radius: 10px;
            }
         </style>
         <table>
            <tr>
               <td>
                  <div class="content">
                     <div class="card">
                        <p> ADS Battery Voltage</p>
                        <p><span class="reading"><span id="BatteryVID">%BATTERYV%</span> V</span></p>
                     </div>
                     <hr/>
                     <div class="card">
                        <p> Victron Battery Voltage</p>
                        <p><span class="reading"><span id="VictronVoltageID">%VVOLT%</span> V</span></p>
                     </div>
                     <hr/>
                     <div class="card">
                        <p> INA Battery Voltage</p>
                        <p><span class="reading"><span id="IBVID">%IBVV%</span> V</span></p>
                     </div>
                     <hr/>
                     <div class="card">
                        <p> ADS Ch3 Voltage</p>
                        <p><span class="reading"><span id="ADS3ID">%ADSCH3VLTS%</span> V</span></p>
                     </div>
               </td>
               <td>
               <div class="card">
               <p> ADS Alternator Current</p><p><span class="reading"><span id="MeasAmpsID">%MEASA%</span> A</span></p>
               </div>
               <hr/>
               <div class="cards">
               <div class="card">
               <p> Alternator Temperature</p><p><span class="reading"><span id="AltTempID">%ALTERNATORTEMPERATUREF%</span> &deg;C</span></p>
               </div>
               <hr/>
               <div class="card">
               <p> INA Battery Current</p><p><span class="reading"><span id="BCurrID">%BCURR%</span> A</span></p>
               </div>
               <hr/>
               <div class="card">
               <p> Engine Speed</p><p><span class="reading"><span id="RPMID">%RPMM%</span> rev/min</span></p>
               </div>
               <hr/>
               <div class="card">
               <p> GPS Heading</p><p><span class="reading"><span id="GPSHID">%GPSH%</span> &deg</span></p>
               </div>
               </div>
               </div>
               </td>
               <td>
                  <div class="card">
                     <p> Loop Time</p>
                     <p><span class="reading"><span id="LoopTimeID">%LOOPTIME%</span> uS;</span></p>
                  </div>
                  <hr/>
                  <div class="card">
                     <p> Controller Field Duty Cycle</p>
                     <p><span class="reading"><span id="DutyCycleID">%DUTYCYCLE%</span> &percnt;</span></p>
                  </div>
                  <hr/>
                  <div class="card">
                     <p> Controller Field Voltage</p>
                     <p><span class="reading"><span id="FieldVoltsID">%FIELDVOLTS%</span> V</span></p>
                  </div>
                  <hr/>
                  <div class="card">
                     <p> Controller Field Current</p>
                     <p><span class="reading"><span id="FieldAmpsID">%FIELDAMPS%</span> A</span></p>
                  </div>
               </td>
            </tr>
         </table>
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
             source.addEventListener('AlternatorTemperatureF', function(e) {
              console.log("AlternatorTemperatureF", e.data);
              document.getElementById("AltTempID").innerHTML = e.data;
             }, false);
             source.addEventListener('DutyCycle', function(e) {
              console.log("DutyCycle", e.data);
              document.getElementById("DutyCycleID").innerHTML = e.data;
             }, false);
             source.addEventListener('BatteryV', function(e) {
              console.log("BatteryV", e.data);
              document.getElementById("BatteryVID").innerHTML = e.data;
             }, false);
             source.addEventListener('MeasuredAmps', function(e) {
              console.log("MeasuredAmps", e.data);
              document.getElementById("MeasAmpsID").innerHTML = e.data;
             }, false);
             source.addEventListener('LoopTime', function(e) {
              console.log("LoopTime", e.data);
              document.getElementById("LoopTimeID").innerHTML = e.data;
             }, false);          
              source.addEventListener('RPM', function(e) {
              console.log("RPM", e.data);
              document.getElementById("RPMID").innerHTML = e.data;
             }, false);
              source.addEventListener('Channel3V', function(e) {
              console.log("Channel3V", e.data);
              document.getElementById("ADS3ID").innerHTML = e.data;
             }, false);
               source.addEventListener('IBV', function(e) {
              console.log("IBV", e.data);
              document.getElementById("IBVID").innerHTML = e.data;
             }, false);
                source.addEventListener('BCURR', function(e) {
              console.log("Bcur", e.data);
              document.getElementById("BCurrID").innerHTML = e.data;
             }, false);
                 source.addEventListener('GPSH', function(e) {
              console.log("HeadingNMEA", e.data);
              document.getElementById("GPSHID").innerHTML = e.data;
             }, false);
                source.addEventListener('VVOLT', function(e) {
              console.log("VictronVoltage", e.data);
              document.getElementById("VictronVoltageID").innerHTML = e.data;
             }, false);
                source.addEventListener('FIELDVOLTS', function(e) {
              console.log("vvout", e.data);
              document.getElementById("FieldVoltsID").innerHTML = e.data;
             }, false);
                 source.addEventListener('FIELDAMPS', function(e) {
              console.log("iiout", e.data);
              document.getElementById("FieldAmpsID").innerHTML = e.data;
             }, false);
            }
         </script>
   </body>
</html>
)rawliteral";

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




void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(4, OUTPUT);  // This pin is used to provide a high signal to SiC450 Enable pin

initWiFi(); // just leave this here

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
    while (1)
      ;
  }

  // at least 0.27 seconds for an update with these settings for moving average and conversion time
  INA.setMode(11);                       // Bh = Continuous shunt and bus voltage
  INA.setAverage(5);                     //0h = 1, 1h = 4, 2h = 16, 3h = 64, 4h = 128, 5h = 256, 6h = 512, 7h = 1024     Applies to all channels
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

  sic45x.begin();
  sic45x.sendClearFaults();
  sic45x.setFrequencySwitch(fffr);  //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);
  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
  // sic45x.setOperation(ON_OFF_DISABLED | OFFB_IMMEDIATE | MARGIN_COMMAND | MRGNFLT_FOLLOW);
  // sic45x.setVoutOvFaultResponse(VOUT_OV_FAULT_RESPONSE_B);
  // sic45x.setVoutUvFaultResponse(VOUT_UV_FAULT_RESPONSE_B);

  ChargingVoltageTarget = TargetFloatVoltage;

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

  //The range of settings allowed for each register varies stupidly with this chip, must stay within the below limits:
  //The VIN_OV_FAULT_LIMIT,VIN_UV_WARN_LIMIT  range is 1 V to 80 V, resolution is 0.5 V
  //output voltage’s range is 0.3 V to 14 V
  ///same for vout, margin high, margin low,VOUT_OV_FAULT_LIMIT,VOUT_OV_WARN_LIMIT...
  //The POWER_GOOD_ON and POWER_GOOD_OFF range is 0.24 V to 14 V
  //0V to 14V for VOUT_UV_WARN_LIMIT ,VOUT_UV_FAULT_LIMIT
  //The VIN_ON and VIN_OFF range is 1 V to 80 V, resolution is 0.5 V
  sic45x.setVinOvFaultLimit(60);
  //sic45x.setPowerGoodOn(0.24);     // .9
  //sic45x.setPowerGoodOff(0.24);    // .85
  sic45x.setVoutOvFaultLimit(14);  // 115 The default value for this was very low according to datasheet, so I raise it here
  sic45x.setVoutOvWarnLimit(14);   // 110
  sic45x.setVoutUvWarnLimit(0);    // .9
  sic45x.setVoutUvFaultLimit(0);   // .8
  sic45x.setVoutMarginLow(0.3);    //.95
  sic45x.setVoutMarginHigh(0.3);   //105

  //Connection check
  if (!adc.testConnection()) {
    Serial.println("ADS1115 Connection failed");
    return;
  }

  //onewire
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  //WIFI STUFF
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  // Serial.println();
  // Serial.print("IP Address: ");
  //  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?TemperatureLimitF=<inputMessage>
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
      writeFile(LittleFS, "/TargetAmpz.txt", inputMessage.c_str());
      TargetAmps = inputMessage.toFloat();
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
    }

    else {
      inputMessage = "No message sent";
    }

    // Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);

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
    String stringTA = String(TargetAmps, 2);
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
    writeFile2("/ForceFloatl1.txt", stringFF.c_str());
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
  //Update some variable swith values from ESP32 Flash memory
  AlternatorTemperatureLimitF = readFile(LittleFS, "/TemperatureLimitF.txt").toInt();
  ManualVoltageTarget = readFile(LittleFS, "/ManualVoltage.txt").toFloat();
  ChargingVoltageTarget = readFile(LittleFS, "/FullChargeVoltage.txt").toFloat();
  TargetAmps = readFile(LittleFS, "/TargetAmpz.txt").toFloat();
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
}
void loop() {

  starttime = micros();  //Record a start time for demonstration
  ReadAnalogInputs();
  ReadTemperatureData();
  ReadVEData();  //read Data from Victron VeDirect
  AdjustSic450();
  UpdateDisplay();
  FaultCheck();
  BlinkLED();
  SendWifiData();                          // break this out later to a different timed blink without delay thing
  digitalWrite(4, SIC450Enabler);          // Enable the SIC450
  if (millis() - prev_millis743 > 5000) {  // every 5 seconds
    NMEA2000.ParseMessages();
    prev_millis743 = millis();
  }
  endtime = micros();  //Record a start time for demonstration
  LoopTime = starttime - endtime;
}
void BlinkLED() {
  currentMillissss = millis();
  if (currentMillissss - previousMillissss >= interval22) {
    // save the last time you blinked the LED
    previousMillissss = currentMillissss;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
  }
  // set the LED with the ledState of the variable:
  digitalWrite(ledPin, ledState);
}
void AdjustSic450() {
  if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust SIC450 every half second

    //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
    if (ManualFieldToggle == 0) {
      if (MeasuredAmps < TargetAmps && vout < (14 - interval)) {
        vout = vout + interval;
      }
      if (MeasuredAmps > TargetAmps && vout > (0.3 + interval)) {
        vout = vout - interval;
      }
      // HAVE TO MAKE SURE THESE VALUES DON'T GET TOO LOW FOR SIC450 COMMAND VALIDITY
      if (AlternatorTemperatureF > AlternatorTemperatureLimitF && vout > (0.3 + interval)) {
        vout = vout - (2 * interval);  // if no *2, the adjustments offset, so this makes the Temp correction win
      }
      if (BatteryV > ChargingVoltageTarget && vout > (0.3 + interval)) {
        vout = vout - (3 * interval);  // if no *3, the adjustments offset, so this makes the Temp correction win
      }

      //sic45x.setVoutCommand(vout);
      prev_millis22 = millis();
    } else {
      vout = ManualVoltageTarget;
    }
    sic45x.setFrequencySwitch(fffr);  //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
    sic45x.setVoutCommand(vout);
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
  if (millis() - prev_millis3 > 10000) {  // every 2 seconds read ads1115 and INA228
    //The mux setting must be set every time each channel is read, there is NOT a separate function call for each possible mux combination.
    adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  //Set single ended mode between AIN0 and GND
    //manually trigger the conversion
    adc.triggerConversion();    //Start a conversion.  This immediatly returns
    Raw = adc.getConversion();  //This polls the ADS1115 and wait for conversion to finish, THEN returns the value
    endtime = micros();
    //Channel0V = Raw / 32768 * 6.144 * 20.242914979757085;
    //Serial.print(Channel0V);
    Channel0V = Raw / 32768 * 6.144 * 20.24291;
    BatteryV = Channel0V;
    if (BatteryV > 14.5) {
      ChargingVoltageTarget = TargetFloatVoltage;
    }

    adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
    adc.triggerConversion();
    Raw = adc.getConversion();
    Channel1V = Raw / 32768 * 6.144 * 2;
    MeasuredAmps = (2.5 - Channel1V) * 80;
    //Serial.print("      Measured Amps: ");
    //Serial.print(MeasuredAmps);

    adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
    adc.triggerConversion();
    Raw = adc.getConversion();
    Channel2V = Raw / 32768 * 6.144 * 2133.2 * 2;
    RPM = Channel2V;
    // Serial.print("       Ch 2 Volts: ");
    // Serial.print(Channel2V);

    adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);
    adc.triggerConversion();
    Raw = adc.getConversion();
    Channel3V = Raw / 32768 * 6.144 * 833;
    //Serial.print("       Ch 3 HZ: ");
    //Serial.println(Channel3V);
    prev_millis3 = millis();

    vvout = (sic45x.getReadVout());
    iiout = (sic45x.getReadIout());
    DutyCycle = (sic45x.getReadDutyCycle());

    Serial.println();
    //Serial.print("INA228 Battery Voltage: ");
    IBV = INA.getBusVoltage();
    // Serial.println(IBV);
    // Serial.print("INA228 Battery Bcur (Amps): ");
    ShuntVoltage_mV = INA.getShuntVoltage_mV();
    Bcur = ShuntVoltage_mV * 10;
    //Serial.print(Bcur);
    //Serial.println();
  }
}
void ReadTemperatureData() {
  if (millis() - prev_millis2 > 10000) {                  // read temperature
    AlternatorTemperatureF = sensors.getTempFByIndex(0);  // fetch temperature
    sensors.requestTemperatures();                        // immediately after fetching the temperature we request a new sample in the async modus
                                                          // Serial.print("Alternator Temperature:");
                                                          // Serial.println(AlternatorTemperatureF);
    prev_millis2 = millis();
  }
}
void UpdateDisplay() {
  if (millis() - prev_millis66 > 10000) {
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
  if (millis() - prev_millis33 > 10000) {
    if (VeDataOn == 1) {
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
    }
    prev_millis33 = millis();
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

    HeadingNMEA = Heading;
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
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

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
    Serial.println("- failed to open file for writing");
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
    Serial.println("Failed to open file for writing");
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
  } else if (var == "FIELDAMPS") {
    return String(iiout);
  }

  return String();
}


// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    //  Serial.print('.');
    delay(500);
  }
  // Serial.println(WiFi.localIP());
  // Serial.println();
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());
  // Serial.print("RRSI (lower absolute value is better!): ");
  // Serial.println(WiFi.RSSI());
}

void SendWifiData() {
  if (millis() - prev_millis5 > 2000) {  // every 2 seconds

    // Send Events to the Web Client with the Sensor Readings
    events.send("ping", NULL, millis());
    events.send(String(AlternatorTemperatureF).c_str(), "AlternatorTemperatureF", millis());
    events.send(String(DutyCycle).c_str(), "DutyCycle", millis());
    events.send(String(BatteryV).c_str(), "BatteryV", millis());
    events.send(String(MeasuredAmps).c_str(), "MeasuredAmps", millis());
    events.send(String(RPM).c_str(), "RPM", millis());
    events.send(String(Channel3V).c_str(), "Channel3V", millis());
    events.send(String(IBV).c_str(), "IBV", millis());
    events.send(String(Bcur).c_str(), "Bcur", millis());
    events.send(String(VictronVoltage).c_str(), "VictronVoltage", millis());
    events.send(String(LoopTime).c_str(), "LoopTime", millis());
    events.send(String(HeadingNMEA).c_str(), "HeadingNMEA", millis());
    events.send(String(vvout).c_str(), "vvout", millis());
    events.send(String(iiout).c_str(), "iiout", millis());

    prev_millis5 = millis();
  }
}