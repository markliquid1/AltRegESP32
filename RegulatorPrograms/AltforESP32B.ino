// Open Source Alternator Regulator by X Engineering
// https://github.com/markliquid1/AltRegESP32
// wwww.xengineering.net
// Last Updated April 28, 2024
// © 2024 Mark Nickerson <mark@xengineering.net>
// GPL-3.0 License

// ***IMPORTANT REMINDER    BATTERIES WILL NOT CHARGE IF Alt Temp <20F or Battery less than 8 Volts without Override ***

//---------------LIBRARIES------------------------------------------------------------------------------------------------------------------------
#include <TinyGPSPlus.h>           // for NMEA0183 parsing, probably can remove as it has no relevance to alternator
#include "VeDirectFrameHandler.h"  // for victron communication
#include <Arduino.h>
#include <Wire.h>                // for all i2c
#include <ADS1115.h>            // https://lygte-info.dk/project/ADS1115Library%20UK.html
#include <Adafruit_SSD1306.h>   // OLED display (using SPI not i2c, speed reasons)
#include <SPI.h>                // OLED display
#include <OneWire.h>            // temp sensors
#include <DallasTemperature.h>  // temp sensors
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "SPIFFS.h"               // this adds ability to read configuration setpoints from a text file in FLASH
#include <WiFi.h>                 // allow updates over WIFI
#include <esp_wifi.h>             // this one is needed for turning wifi "sleep mode" off permanently for better Wifi reliability
#include <AsyncTCP.h>             //allow updates over WIFI
#include <ESPAsyncWebServer.h> .  //allow updates over WIFI
//#include <ElegantOTA.h> .  //allow program re-flashing over WIFI-- currently not implemented but was working at one point in a differnet file
#include <RTCx.h>              // real time clock
#include "INA3221.h"           // 3 channel analog input card specialized for BMS
#include "SiC45x.h"            // Vishay buck converter (heart of system!)
#include <PID_v1.h>            // PID library from http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
#include "InterpolationLib.h"  // used by PID if RPM table is active

//------------User Adjustable Constants---------------------------------------------------------------------------------------------------------------------

// Network Credentials        update to match your local wifi network
const char* ssid = "MN2G";
const char* password = "5FENYC8PDW";

int Tthreshold = 165;  //Temperature at which the alternator will scale back ( units degF not C )
int lowamps = 20;      // base Amps target on "slow" charge setting
int highamps = 45;     // base Amps target on "fast" charge setting

// RPM based charging target.  To effectively disable, make all Y values 9999 or some high number
const int numValues = 10;                                                      // number of entries in the RPM table
double xValues[10] = { 0, 300, 350, 400, 500, 1000, 2000, 3000, 4000, 5000 };  // X axis values(rpm)
double yValues[10] = { 0, 0, 20, 30, 40, 50, 50, 60, 60, 60 };                 // Y axis values (amps)   Later put these in User Interface/SPIFFS


float VthresholdH = 14.3;  // over volt threshold, regulator will not charge higher than this value, recommend to set to a maximum of "BMS cutoff voltage - 0.1V"
float VthresholdL = 8.0;   // under volt threshold, prevent charging below here

bool VeDataOn = 0;            // If you want to use  VeDirect data for anything set this to 1
bool VeDataforVoltageOn = 0;  //and want to use it as the source of Battery Voltage
bool VeDataforCurrentOn = 0;  //and want to use it as the source of Battery Current

bool NMEA2KOn = 0;                // If you want to use NMEA2K data for anything set this to 1
bool NMEA2KBatteryVoltageOn = 0;  //and want to use it as the source of Battery Voltage
bool NMEA2KBatteryCurrentOn = 0;  //and want to use it as the source of Battery Current
bool NMEA2KCurrentOn = 0;         //and want to use it as the source of Alternator Current (if this even exists)


bool INA3221BatteryCurrentOn = 0;  // If you want to make use of a battery shunt connected to the ESP32/INA3221 chip

bool ADS1115RPMOn = 0;  // if you want to use alternator calculated engine RPM for any reason (such as broken belt detector or RPM based charge table), set to 1
bool NMEA2KRPMOn = 0;   // same deal but for RPM from NMEA2K network  (NMEA2KOn also set to 1)

bool SwitchSource = 0;  // Set to zero give physical hardware switches control.  Set to 1 to give web interface control (ignore hardware switches)


//----------------Variables used by program.  Some of these can be made local variables at some point if memory an issue--------------------------------------------

//Battery Voltage
float VictronVoltage = 0;        // battery reading from VeDirect
float INA3221Volts = 0;          // battery reading from INA3221
float ADS1115Volts = 0;          // battery reading from ADS1115
float NMEA2KBatteryVoltage = 0;  // battery reading from NMEA2K
float BatteryVoltage = 0;        //This will be the chosen one we control off of

//SiC450x measurements
float SiC450xInVoltage;
float SiC450xInCurrent;
float SiC450xOutVoltage;
float SiC450xOutCurrent;
float SiC450xTemp;
float SiC450xDutyCycle;

//Alternator Current and RPM
int AmpTarget;                    // store the selected Amp target- after all checks, this is the final target setpoint
float VictronCurrent = 0;         // battery current as measured by victron
float ADS1115Current = 0;         // Alternator current as measured by ADS1115
float NMEA2KBatteryCurrent = 0;   // Battery current as measured by NMEA2K
float NMEA2KCurrent = 0;          // Alternator current as measured by NMEA2K (probably delete this, doubt it exists)
float INA3221BatteryCurrent = 0;  // Battery current as measured by ESP32/INA3221 chip
float curntValue = 0;             //  This will be the chosen alternator current we control off of
int ADS1115RPM = 0;               //  RPM from built in frequency converter  - this will only be the control if NMEA2KRPM is not present
int NMEA2KRPM = 0;                //  RPM from NMEA2K-  this will become the control by default if it's present
int RPM = 0;                      // primary RPM reading we control off of.  It will continue to = 0 if no source of RPM data is present.
int AmpsLimitbyRPM = 99999;       //variable to store current limit based on RPM table

float DynamicTransferFactor = .2;  //Multiplying the target alternator output (in amps) by this number should = field voltage setpoint.
//This value is continually updated, so this first guess only needs to be in the ballpark
//Another way of stating the same thing:  12V field voltage is assumed to produce 60 amps
// Lower guesses for the factor are more conservative but if the base assumption can't make at least 10 amps (with whatever the initial setpoint is), the regulator will not be able to increase the field

//Battery Current
float BatteryCurrent = 0;  // final one used for control in the case of a battery based goal

//Temperature Related
float AlternatorTemperature = 0;
float ADS1115Thermistor = 0;
int resolution = 12;  // for OneWire measurement

//Error handling
bool ErrorFlag = 0;
char ErrorMessage[50];

//Used to make sure engine is spinning if going to keep supplying field current by function EngienOfforSlipBeltCheck()
unsigned long start_millis;
bool ThisIsTheFirstTime = 1;

//HardwareSwitches
bool HiLowMode = 0;                  // Select high or low amps to target.  High can be used for maximum charge rate, while low can be general use when Solar is generall enough
bool LimpHomeMode = 0;               // DANGER: This mode will override having a valid temperature sensor, voltage reading, current reading, etc, and do the best it can to charge at a low field voltage of your choice (5V default)  Hard coded.
bool ForceFloat = 0;                 // In this mode, the alterantor will target Zero amps (or a value of your choice, hard-coded) at the battery shunt
bool KillCharge = 0;                 // This switch is used to manually turn off the field (stop charging)
bool KillChargeFromExternalBMS = 0;  // This is the same functionality as above, but originating from an outside source such as 3rd party BMS

//PID
/*Tuning Parameters for PID*/
float Kp = .1;          //NOTE: with PonM, a bigger Kp will make your controller more conservative.
float Ki = 1;           // integration time in seconds (i believe larger is more aggressive)
float Kd = 0;           // derivative time in seconds (i believe lower is more aggressive)
int SampleTime = 1100;  // in milliseconds, how often to update output of PID.  VictronVe direct is the main limiting factor here, only 1hz updates so don't want the loop faster
float outMin = 0.3;
float outMax = 14;  // update these to stay in range of SiC450 (volts)
/*Working Variables for PID*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;


//-------------//Misc pre-setup code that don't need frequent changes----------------------------------------------------------------------------------------------------
//SiC450 setup
SiC45x sic45x(0x0D);  // pmbus address set by 10k resistor according to datasheet page 11 "MFR_BASE_ADDRESS Table"
//set up PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, REVERSE);  //P_ON_M specifies that Proportional on Measurement be used
//CAN Tx/Rx pin #'s on ESP32 GPIO
#define ESP32_CAN_TX_PIN GPIO_NUM_16  // If you use ESP32 and do not have TX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_17  // If you use ESP32 and do not have RX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
//INA3221 presetup
// Set I2C address to 0x41 (A0 pin -> VCC)
INA3221 INA(0x40);  // Pin A0 in hardware is grounded to make address =40
//NMEA0183 setup
TinyGPSPlus gps;  //Object for NMEA0183 reading
// In this section, define custom NMEA sentences
//from B&G Wind
TinyGPSCustom ApparentWindMagnitude(gps, "IIVWR", 3);  // $IIVWR sentence, 3rd element
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
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_SSD1306 display(128, 64, 23, 18, 19, -1, 5);
// onewire    Data wire is conntec to the Arduino digital pin 14
#define ONE_WIRE_BUS 14
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

// ADS1115 4 channel analog input card 5V maximum
ADS1115Scanner adc;
//Wifi and User Interface related:
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");
// Timer variables, these appear to be unused.
//unsigned long lastTime = 0; //CAREFUL this is the same variable name as used by PID functions
//unsigned long timerDelay = 1000; // update the web display 1x per second



//-------------The usual setup() and loop() functions----------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);  //The usual serial monitor
  Wire.begin();          // for all I2C.  Warning, this is blocking
  //INA3221
  INA.begin();              //Note: one needs to set Wire.begin() before calling begin().
  INA.setShuntR(0, 0.100);  //int setShuntR(uint8_t channel, float ohm) sets value in Ohm.
  INA.setShuntR(1, 0.100);
  INA.setShuntR(2, 0.100);
  INA.setCriticalAlert(0, 50000);  //sets the critical alert level in microvolts
  INA.setCriticalAlert(1, 100000);
  INA.setCriticalAlert(2, 150000);
  INA.setCriticalCurrect(0, 100000);  //sets the critical alert level in milliAmpere
  INA.setCriticalCurrect(1, 100000);  //sets the critical alert level in milliAmpere
  INA.setCriticalCurrect(2, 100000);  //sets the critical alert level in milliAmpere
  //INA.setWarningAlert(0, 25000); //sets the critical warning level in microvolts
  //INA.setWarningAlert(1, 75000);
  //INA.setWarningAlert(2, 125000);
  INA.setShuntVoltageSumLimit(32198);
  INA.setPowerUpperLimit(5000);
  INA.setPowerLowerLimit(4000);
  INA.enableChannel(0);
  INA.disableChannel(0);
  INA.setAverage(2);


  //Victron
  Serial1.begin(19200, SERIAL_8N1, 16, -1, 1);  // ... note the "1" for inversion of logic.  This is the reading of the combined NMEA0183 data from YachtDevices
  Serial2.begin(19200, SERIAL_8N1, 17, -1, 0);  // This is the reading of Victron VEDirect
  Serial2.flush();
  //onewire
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  //DISPLAY
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  esp_wifi_set_ps(WIFI_PS_NONE);  // make sure wifi doesn't go into low power mode

  //SIC450 setup
  sic45x.begin();
  sic45x.sendClearFaults();  //clear any fault bits that have been set. This command clears all bits in all status registers simultaneously.
  sic45x.printStatusWord();
  //sic45x.setOperation(SIC45X_OPERATION_ON_OFF_DISABLED); // use this line to shut down
  sic45x.setOperation(
    SIC45X_OPERATION_ON_OFF_DISABLED               //Output is enabled
    | SIC45X_OPERATION_OFF_B_IMMEDIATE             //On shutdown output is turned off immediately and power off sequence commands ignored
    | SIC45X_OPERATION_MARGIN_COMMAND              //Output voltage is set by the PMBus VOUT_COMMAND data
    | SIC45X_OPERATION_MRGNFLT_IGNORE              // Faults caused by selecting VOUT_MARGIN_HIGH or VOUT_MARGIN_LOW as nominal output voltage source are ignored (This is a meaningless setting)
    | SIC45X_ON_OFF_CONFIGURATION_PU_COMMAND       //// Regulator does not power up until commanded by the CONTROLEN pin and OPERATION command
    | SIC45X_ON_OFF_CONFIGURATION_CMD_RESPOND      //// Regulator responds the "on" bit in the OPERATION command
    | SIC45X_ON_OFF_CONFIGURATION_EN_REQUIRE       //Regulator requires the EN pin to be asserted to start the unit
    | SIC45X_ON_OFF_CONFIGURATION_ENPOL_HIGH       //EN signal is active high
    | SIC45X_ON_OFF_CONFIGURATION_OFFB1_IMMEDIATE  //Regulator turns off immediately when EN is low
    | SIC45X_WRITE_PROTECT_WTPRT_ALL               //No write protection is used
  );
  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);  // without this, the 5V device can't read or make 12V
  sic45x.setFrequencySwitch(1000);                           //The FREQUENCY_SWITCH range is 300 kHz to 1500 kHz, resolution is 50 kHz, and its NVM register default store value is 0258h
  //equivalent to 600 kHz. Any commands out of the valid range or with incorrect resolution will be ignored and reported.
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_STANDALONE);  // this could be used to synchonise the switching with other devices
  sic45x.setVoutTransitionRate(.125);                       //sets the rate in mV/μs at which the output
  // voltage should change voltage when a PMBus device receives either a
  // VOUT_COMMAND   range is 0.0625 - 2 mV/μs, resolution is 0.0625 mV/μs, and its NVM register default store value
  //is E002h equivalent to 0.125 mV/μs.
  sic45x.setVoutOvFaultLimit(99);   //no need for this feature
  sic45x.setVoutOvWarnLimit(99);    //no need for this feature
  sic45x.setVoutUvWarnLimit(-99);   //no need for this feature
  sic45x.setVoutUvFaultLimit(-99);  //no need for this feature
  //The below are included just for referenc only- there is no need for these safeties
  sic45x.setVoutOvFaultResponse(SIC45X_VOUT_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY);  // The device's output is disabled while the fault is present. Operation resumes and the output is enabled when the fault condition no longer exists.
  sic45x.setVoutUvFaultResponse(SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_DISABLED_WHILE_FAULTY);  // The device's output is disabled while the fault is present. Operation resumes and the output is enabled when the fault condition no longer exists.
  sic45x.setVoutOvFaultResponse(SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_RETRIES);
  sic45x.setVoutOvFaultResponse(SIC45X_VOUT_OV_FAULT_RESPONSE_OVDLY_NO_DELAY);  // and do it without delay in between attempts
  sic45x.setVoutUvFaultResponse(SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_RETRIES);   // infinitely attempt to restart
  sic45x.setVoutUvFaultResponse(SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_NO_DELAY);  //and do it without delay in between attempts

  //PID Setup
  //SetMode();//Specifies whether the PID should be on (Automatic) or off (Manual.) The PID defaults to the off position when created.
  myPID.SetOutputLimits(outMin, outMax);  //The PID controller is designed to vary its output within a given range. By default this range is 0-255: the arduino PWM range. There's no use sending 300, 400, or 500 to the PWM. Depending on the application though, a different range may be desired.
  //SetTunings(Kp, Ki, Kd, POn); // These are only necessary if the values need to be changed during runtime.
  myPID.SetSampleTime(SampleTime);  // How often the output will be updated in ms.     200 (default)is a good starting point for robotics
  //SetControllerDirection; //again only for changing during runtime (very unlikely anyone would ever need this)
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //ADS1115
  adc.setSpeed(ADS1115_SPEED_128SPS);  // fastest possible with full noise rejection
  adc.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144);
  adc.setSamples(5);  // number of samples to average.  Higher numbers could crash Uno but probably ok on ESP32(?)
  adc.start();
}

void loop() {
  adc.update();               // the is for ADS1115
  ReadSensors();              //
  GetSetUp();                 // misc things that need to be done
  AreTheReadingsRealistic();  // Check if all desired inputs are present and reasonable
  EngienOfforSlipBeltCheck();
  SetField();  // Calculate altenrator ouput current target based on a number of factors
  //AdjustCurrent();     // Adjust SiC voltage target based on a number of factors
  UpdateDisplay();  //
}
//--------------------FUNCTIONS--------------------------------------------------------------------------------------------------------
void ReadSensors() {
  ReadSiC450();         // read data from buck converter
  ReadVEData();         //read Data from Victron VeDirect
  ReadNMEA0183();       //read NMEA0183 readings from serial port 1
  ReadDigTempSensor();  //read theOneWire sensor
  ReadVEData();         // read Victron readings from serial port 2
  ReadINA3221();        // reac voltage from INA3221
  ReadADS1115();        // read voltage readings from ADS1115
  ReadNMEA2K();         //read data from NMEA2K network CAN connection
  ReadSwitches();       // poll the various on/off switches
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
  } else if (NMEA2KBatteryVoltageOn) {
    BatteryVoltage = NMEA2KBatteryVoltage;
  } else {
    BatteryVoltage = INA3221Volts;
  }
  if (VeDataforCurrentOn == 1) {
    curntValue = VictronCurrent;
  } else if (NMEA2KCurrentOn == 1) {
    curntValue = NMEA2KCurrent;
  } else {
    curntValue = ADS1115Current;
  }
  if (NMEA2KRPMOn == 1) {
    RPM = NMEA2KRPM;
  } else {
    RPM = ADS1115RPM;
  }
  if (NMEA2KBatteryCurrentOn == 1) {
    BatteryCurrent = NMEA2KBatteryCurrent;
  }
  if (INA3221BatteryCurrentOn == 1) {
    BatteryCurrent = INA3221BatteryCurrent;
  }
}
void AreTheReadingsRealistic() {
  if (AlternatorTemperature > 300 || AlternatorTemperature < 20) {
    ErrorFlag = 1;
    strcpy(ErrorMessage, "Temp sensor out of range");
  }
  if (BatteryVoltage > VthresholdH || BatteryVoltage < VthresholdL) {
    ErrorFlag = 1;
    strcpy(ErrorMessage, "BatteryV out of range");
  }
  if (curntValue < -20 || curntValue > 500) {
    ErrorFlag = 1;
    strcpy(ErrorMessage, "AltCurrent out of range");
  }
  if (BatteryCurrent < -500 || BatteryCurrent > 500) {
    ErrorFlag = 1;
    strcpy(ErrorMessage, "BattCurrent out of range");
  }
}
void EngienOfforSlipBeltCheck() {
  if (ADS1115RPMOn == 1 && NMEA2KRPMOn == 1) {  // % if the sensors are said to be present
    if (abs(ADS1115RPM - NMEA2KRPM) > 500) {    // if they differ by more than 500 RPM
      // Sound alarm 1x every 5 seconds
      // Alert display that belt is slipping
    }
    if (ADS1115RPM < 300) {
      //Alert display that the engine is not spinning according to LM2907 chip
      AmpTarget = 0;  // refuse to charge
    }
    if (NMEA2KRPM < 300) {
      //Alert display that the engine is not spinning according to NMEA2K
      AmpTarget = 0;  // refuse to charge
    }
  }
  if (ADS1115RPMOn == 0 && NMEA2KRPMOn == 1) {
    if (NMEA2KRPM < 300) {
      //Alert display that the engine is not spinning according to NMEA2K
      AmpTarget = 0;  // refuse to charge
    }
  }
  if (ADS1115RPMOn == 1 && NMEA2KRPMOn == 0) {  // if the sensors are said to be present
    if (ADS1115RPM < 300) {
      //Alert display that the engine is not spinning according to LM2907 chip
      AmpTarget = 0;  // refuse to charge
    }
  }
  // In all cases, if there is a field and no measured amps for more than 100ms, then the engine is not spinning, and a 10 second delay is warranted
  if (AmpTarget > 10 && curntValue < 5) {
    if (ThisIsTheFirstTime == 1) {                   //if this is the first time finding a strange lack of charging
      static unsigned long start_millis = millis();  // start timer
      ThisIsTheFirstTime = 0;                        // the timer has been started, it's not the first time, we're counting time now
    }
    if (millis() - start_millis > 100) {  // if the condition does not self-correct within 100 milliseconds (ie current goes up, or setpoint goes down)
      //report to display              //*may need to prevent this from happening too frequently with a "display error message" function?
      AmpTarget = 0;  // refuse to charge
    }
    if (millis() - start_millis > 10000) {  // if it has been more than 10 seconds
      AmpTarget = 15;                       //try charging again
      ThisIsTheFirstTime = 1;               // and get ready to re-check for output
    }
  } else {  // if the condition corrected itself, prepare to restart the timer next time the discrepancy is noticed again.
    ThisIsTheFirstTime = 1;
  }
}
void SetField() {
  if (LimpHomeMode == 1)
    ;
  {
    sic45x.setVoutCommand(5);  //and alert display
    return;
  }

  if (KillChargeFromExternalBMS == 1)
    ;
  {
    sic45x.setVoutCommand(0.3);
    sic45x.setOperation(SIC45X_OPERATION_ON_OFF_DISABLED);  // use this line to shut down
    // and alert display
    return;
  }
  if (KillCharge == 1)
    ;
  {
    sic45x.setVoutCommand(0.3);
    sic45x.setOperation(SIC45X_OPERATION_ON_OFF_DISABLED);  // use this line to shut down
    // and alert display
    return;
  }

  if (HiLowMode == 0) {
    //Then we only want to reduce for temperature, otherwise stay with the amp setting
    return;
  }
  if (ForceFloat == 0) {
    //Then we only want to reduce for temperature, otherwise stay with the voltage target
    return;
  }

  //Check for absorption and float mode somehow here

  AmpsLimitbyRPM = Interpolation::ConstrainedSpline(xValues, yValues, 10, RPM);
  if (AmpsLimitbyRPM < AmpTarget) {
    AmpTarget = AmpsLimitbyRPM;
  }

  Input = AlternatorTemperature;
  //First, calculate approximate field voltage (now called Setpoint) to hit whatever amperage target is called for
  Setpoint = AmpTarget * DynamicTransferFactor;  // This is going to be a voltage value between 0.3 and 14, the target for SiC450
  //Then constrain the setpoint within physical bounds
  if (Setpoint < outMin) {
    Setpoint = outMin;
  }
  if (Setpoint > outMax) {
    Setpoint = outMax;
  }
  //The PID controller returns true when the output is computed, false when nothing has been done.
  myPID.Compute();  //Most of the time it will just return without doing anything. At a frequency specified by SetSampleTime it will calculate a new Output.
  //SiC450 output
  sic45x.setVoutCommand(Setpoint);
  ////Optionally, report the PID gain values
  //Serial.println(GetKp());
  //Serial.println(GetKi());
  //Serial.println(GetKd());
}
void UpdateDisplay() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {

    if (ErrorFlag == 1) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Error: ");
      display.println(ErrorMessage);
    } else {
      //  PrintData(); // this function prints everything available to serial monitor
      display.clearDisplay();
      //display.display();
      display.setCursor(0, 0);
      display.println("Vlts: ");
      display.setCursor(90, 0);
      display.println(BatteryVoltage, 2);
      display.setCursor(0, 11);
      display.println("Amps:");
      display.setCursor(90, 11);
      display.println(curntValue, 1);
      display.setCursor(0, 22);
      display.println("Temp: ");
      display.setCursor(90, 22);
      display.println(AlternatorTemperature, 1);
      display.setCursor(0, 33);
      display.println("PWM%:");
      display.setCursor(90, 33);
      display.println(Output, 1);  //String(val, decimalPlaces)
      display.display();
    }
    prev_millis = millis();
  }
}

//Called from function "ReadSensors()"
void ReadSiC450() {

  // Report operational parameters every 2s
  SiC450xInVoltage = (sic45x.getReadVin());
  SiC450xInCurrent = (sic45x.getReadIin());
  SiC450xOutVoltage = (sic45x.getReadVout());
  SiC450xOutCurrent = (sic45x.getReadIout());
  SiC450xTemp = (sic45x.getReadTemperature());
  SiC450xDutyCycle = (sic45x.getReadDutyCycle());
}
void ReadVEData() {
  if (VeDataOn == 1) {
    while (Serial2.available()) {
      myve.rxData(Serial2.read());
      for (int i = 0; i < myve.veEnd; i++) {
        if (strcmp(myve.veName[i], "V") == 0) {
          VictronVoltage = (atof(myve.veValue[i]) / 1000);
        }
        if (strcmp(myve.veName[i], "I") == 0) {
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
  if (ApparentWindMagnitude.isUpdated() || ApparentWindSide.isUpdated() || ApparentWindAngle.isUpdated() || Heading.isUpdated() || RudderAngle.isUpdated() || SpeedOverGround.isUpdated() || CourseOverGround.isUpdated()) {  //These can all get converted into Floats and then displayed in the OncePerSecond Loop someday

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
    AlternatorTemperature = sensors.getTempFByIndex(0);  // fetch temperature
    sensors.requestTemperatures();                       // immediately after fetching the temperature we request a new sample in the async modus
    prev_millis2 = millis();
    //Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  }
}
void ReadINA3221() {
  INA3221Volts = 0;           // battery voltage reading from INA3221
  INA3221BatteryCurrent = 0;  // shunt current reading (battery current)
   INA.getBusVoltage(0), 3; // read voltage on channel 0
   INA.getShuntVoltage_mV(0), 3;  //read shunt on channel 0
   INA.getCurrent_mA(0), 3; // read current on channel 0
}
void ReadADS1115() {
  if (ADS1115RPMOn == 1) {
    if (adc.ready()) {
      //ADS1115Current=(adc.readAverage(0)* 2* 0.0001875*100)-250; // (200A range clamp sensor) 2 is the voltage divider
      ADS1115Current = (adc.readAverage(0) * 2 * 0.0001875 * 50) - 125;  // (100A range clamp sensor)  2 is the voltage divider
      //ADS1115Current = -1 * ((adc.readAverage(0) * 0.0001875 * 5 * 100) - 165);  // (Amazon Sensor) 5 is the voltage divider
      ADS1115Volts = (adc.readAverage(1) * 0.0001875 * 25);                   //25 is the voltage divider, adjust later
      ADS1115RPM = adc.readAverage(2) * 0.0001875 * 3000 / 3.375 * 60 / 2.5;  // 3000hz/3.375 volts is the sensitivity.  60 goes to "per minute".  2.5 is the alternator pulley ratio.
      ADS1115Thermistor = adc.readAverage(3) * 0.0001875;                     //Future expansion capability for a thermistor
      adc.start();
    }
  }
}
void ReadNMEA2K() {
  if (NMEA2KOn == 1) {
    if (NMEA2KRPMOn == 1) {
      RPM = NMEA2KRPM;
    }
    if (NMEA2KBatteryVoltageOn == 1) {
      //Code
    }
    if (NMEA2KCurrentOn == 1) {
      //Code
    }
    if (NMEA2KBatteryCurrentOn == 1) {
      //Code
    }
  }
}
void ReadSwitches() {
  if (SwitchSource == 0) {        // if this is set to 1, you can only make changes thru the Web Interface, besides Limp Home Mode, which is always allowed
    HiLowMode = digitalRead(A1);  // Check which charge mode is selected, either fast or slow
    ForceFloat = digitalRead(A3);
    KillCharge = digitalRead(A4);
    KillChargeFromExternalBMS = digitalRead(A5);
  }
  LimpHomeMode = digitalRead(A2);
}
void PrintData() {  //This prints all victron data received to the serial monitor, put this in "Loop" for debugging if needed
  for (int i = 0; i < myve.veEnd; i++) {
    Serial.print(myve.veName[i]);
    Serial.print(i);
    Serial.print("= ");
    Serial.println(myve.veValue[i]);
  }
}

