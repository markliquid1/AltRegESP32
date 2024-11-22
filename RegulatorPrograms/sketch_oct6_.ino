#include <OneWire.h>            // temp sensors
#include <DallasTemperature.h>  // temp sensors
#include <SPI.h>                // display
#include <Wire.h>               // RTC and maybe others
#define PCF8526address 0x51     // address of RTC
#include <Adafruit_GFX.h>       // display
#include <Adafruit_SSD1306.h>   // display
#include <ADS1115_lite.h>       // measuring 4 analog inputs
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);
#include "VeDirectFrameHandler.h"  // for victron communication
#include "INA228.h"
INA228 INA(0x40);
#include "SiC45x.h"  // heart of system, DC/DC converter chip
SiC45x sic45x(0x1D);
#include <Arduino.h>  // maybe not needed, was in NMEA2K example I copied
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>  // questionably needed

// Settings
float TargetAmps = 55;
float TargetFloatVoltage = 13.5;
float TargetBulkVoltage = 14.5;
float ChargingVoltageTarget = 0;
float vout = 0.9;                        // default field output voltage
float interval = 0.1;                    // voltage step to adjust field by each time the loop runs
float fffr = 1200;                       // this is the switching frequency for SIC450
bool benchtest = 0;                      // set to 1 if the engine isn't going to be running during debugging, to prevent high field output
float AlternatorTemperatureLimit = 150;  // the offset appears to be +40 to +50 to get max alternator temp
int resolution = 12;                     // for OneWire measurement
bool VeDataOn = 1;                       // Set to 1 if VE serial data exists
bool FaultCheckToggle = 0;               // Set to 1 to get a serial print out over 20 seconds of error data

//Variables to store measurements
float Raw;
float Channel0V;  // voltage divider is 20
float BatteryV;
float Channel1V;        // voltage divider is 2
float MeasuredAmps;     // alternator output current
float ShuntVoltage_mV;  // Battery shunt voltage from INA228
float Bcur;             // battery shunt current from INA228
float IBV;              // Ina 228 battery voltage
float Channel2V;        // voltage divider is 2---- wired internally to LM2907
float Channel3V;        // voltage divider is an empty socket for user installation
float DutyCycle;        //SIC parameter
float vvout;            // SIC parameter
float iiout;            // SIC parameter, not currently working
float RPM;              // from LM2907
float AlternatorTemperature = 0;
float VictronVoltage = 0;  // battery reading from VeDirect
//RTC variables and math
byte second, minute, hour;
byte bcdToDec(byte value) {
  return ((value / 16) * 10 + value % 16);
}
byte decToBcd(byte value) {
  return (value / 10 * 16 + value % 10);
}


// variables used to show how long the ADC conversion takes place
unsigned long starttime;
unsigned long endtime;
//"Blink without delay" style timer variables used to control how often differnet parts of code execute
static unsigned long prev_millis4;    // used to delay checking of faults in the SiC450
static unsigned long prev_millis66;   //used to delay the updating of the display
static unsigned long prev_millis22;   // used to delay sampling of sic450
static unsigned long prev_millis3;    // used to delay sampling of ADS1115 to every 2 seconds for example
static unsigned long prev_millis2;    // used to delay sampling of temp sensor to every 2 seconds for example
static unsigned long prev_millis33;   // used to delay sampling of Serial Data (ve direct)
static unsigned long prev_millis842;  // used to check realtime clock
static unsigned long prev_millis843;   // another RTC
// pre-setup stuff
// onewire    Data wire is connetec to the Arduino digital pin 13
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);´
Adafruit_SSD1306 display(128, 64, 23, 18, 19, -1, 5);


#define ESP32_CAN_RX_PIN GPIO_NUM_16  // If you use ESP32 and do not have TX on default IO X, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_TX_PIN GPIO_NUM_17  // If you use ESP32 and do not have RX on default IO X, uncomment this and and modify definition to match your CAN TX pin.

//VictronEnergy
VeDirectFrameHandler myve;

//temporary interrupt stuff- this is used to pause and ask for data from keyboard serial
//bool interrupted = 0;
//void IRAM_ATTR ISR() {
//  interrupted = 1;
//}


typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void TripFuelConsumption(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void BinaryStatus(const tN2kMsg &N2kMsg);
void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void LocalOffset(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void Humidity(const tN2kMsg &N2kMsg);
void Pressure(const tN2kMsg &N2kMsg);
void UserDatumSettings(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  { 126992L, &SystemTime },
  { 127245L, &Rudder },
  { 127250L, &Heading },
  { 127257L, &Attitude },
  { 127488L, &EngineRapid },
  { 127489L, &EngineDynamicParameters },
  { 127493L, &TransmissionParameters },
  { 127497L, &TripFuelConsumption },
  { 127501L, &BinaryStatus },
  { 127505L, &FluidLevel },
  { 127506L, &DCStatus },
  { 127513L, &BatteryConfigurationStatus },
  { 128259L, &Speed },
  { 128267L, &WaterDepth },
  { 129026L, &COGSOG },
  { 129029L, &GNSS },
  { 129033L, &LocalOffset },
  { 129045L, &UserDatumSettings },
  { 129540L, &GNSSSatsInView },
  { 130310L, &OutsideEnvironmental },
  { 130312L, &Temperature },
  { 130313L, &Humidity },
  { 130314L, &Pressure },
  { 130316L, &TemperatureExt },
  { 0, 0 }
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);


void setup() {
  Serial.begin(115200);
  delay(500);

  //Wire.begin();  // is this needed?  Probably was done elsewhere in some library?
  //RTC
  //This will set up register 27 which mainly disables RTC oscillator output to save a little power
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x27);
  Wire.write(B10000000);
  Wire.endTransmission();
  //This will set up register 28 which mainly changes from clock mode to stopwatch mode
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x28);
  Wire.write(B00010000);
  Wire.endTransmission();

  //NMEA2K
  OutputStream = &Serial;
  //   while (!Serial)
  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(true);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->print("Running...");

  //Victron VeDirect
  Serial1.begin(19200, SERIAL_8N1, 25, -1, 0);  // ... note the "0" at end for normal logic.  This is the reading of the combined NMEA0183 data from YachtDevices
  Serial2.begin(19200, SERIAL_8N1, 26, -1, 1);  // This is the reading of Victron VEDirect
  Serial2.flush();
  //temporary interrupt stuff- this is used to pause and ask for data from keyboard serial
  //attachInterrupt(35, ISR, RISING);  // GPIO 35 is limp home pin, but borrowing it for this
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

  pinMode(4, OUTPUT);     // This pin is used to provide a high signal to SiC450 Enable pin
  digitalWrite(4, HIGH);  // Enable the SIC450

  sic45x.begin();
  sic45x.sendClearFaults();
  sic45x.setFrequencySwitch(fffr);  //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);
  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
  //sic45x.setOperation(ON_OFF_DISABLED | OFFB_IMMEDIATE | MARGIN_COMMAND | MRGNFLT_FOLLOW);
  //sic45x.setVoutOvFaultResponse(VOUT_OV_FAULT_RESPONSE_B);
  //sic45x.setVoutUvFaultResponse(VOUT_UV_FAULT_RESPONSE_B);

  ChargingVoltageTarget = TargetBulkVoltage;


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
}
void loop() {
  //temporary interrupt stuff- this is used to pause and ask for data from keyboard serial
  //if (interrupted == 1) {
  // Serial.print("Please enter voltage target: ");
  // while (Serial.available() == 0) {
  //  }
  // vout = Serial.parseFloat();
  //  Serial.print("Please enter freq target: ");
  //  while (Serial.available() == 0) {
  //  }
  //  fffr = Serial.parseFloat();
  //  interrupted = 0;
  //}

  ReadAnalogInputs();
  ReadTemperatureData();
  ReadVEData();  //read Data from Victron VeDirect
  AdjustSic450();
  ClockStuff();
  UpdateDisplay();
  FaultCheck();
  NMEA2000.ParseMessages();
}
void ClockStuff() {  // read stopwatch every 5 seconds, print stopwatch every 60 seconds
  if (millis() - prev_millis843 > 5000) {
    readPCF8526();
    if (hour > 12) {
      ChargingVoltageTarget = TargetBulkVoltage;
    }
    prev_millis843 = millis();
  }

  if (millis() - prev_millis842 > 60000) {
    Serial.print(hour);
    Serial.print(":");
    if (minute < 10) {
      Serial.print("0");
    }
    Serial.print(minute);
    Serial.print(":");
    if (second < 10) {
      Serial.print("0");
    }
    Serial.println(second);
    prev_millis842 = millis();
  }
}

void AdjustSic450() {
  if (millis() - prev_millis22 > 500) {  // adjust SIC450 every half second

    //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
    if (benchtest == 0) {
      if (MeasuredAmps < TargetAmps && vout < 12.7) {
        vout = vout + interval;
      }
      if (MeasuredAmps > TargetAmps && vout > 1.75) {
        vout = vout - interval;
      }
      // HAVE TO MAKE SURE THESE VALUES DON'T GET TOO LOW FOR SIC450 COMMAND VALIDITY
      if (AlternatorTemperature > AlternatorTemperatureLimit && vout > 1.75) {
        vout = vout - (2 * interval);  // if no *2, the adjustments offset, so this makes the Temp correction win
      }
      if (BatteryV > ChargingVoltageTarget && vout > 1.75) {
        vout = vout - (3 * interval);  // if no *3, the adjustments offset, so this makes the Temp correction win
      }

      //sic45x.setVoutCommand(vout);
      prev_millis22 = millis();
    }

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
    starttime = micros();       //Record a start time for demonstration
    adc.triggerConversion();    //Start a conversion.  This immediatly returns
    Raw = adc.getConversion();  //This polls the ADS1115 and wait for conversion to finish, THEN returns the value
    endtime = micros();
    //Channel0V = Raw / 32768 * 6.144 * 20.242914979757085;
    //Serial.print(Channel0V);
    Channel0V = Raw / 32768 * 6.144 * 20.24291;
    BatteryV = Channel0V;
    //Serial.print("Ch 0 Volts: ");
    //Serial.print(Channel0V);
    //Serial.print(endtime - starttime);
    //Serial.println("us");
    if (BatteryV > 14.5) {
      ChargingVoltageTarget = TargetFloatVoltage;
      setPCF8526();
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
    Serial.print("INA228 Battery Voltage: ");
    IBV = INA.getBusVoltage();
    Serial.println(IBV);
    Serial.print("INA228 Battery Bcur (Amps): ");
    ShuntVoltage_mV = INA.getShuntVoltage_mV();
    Bcur = ShuntVoltage_mV * 10;
    Serial.print(Bcur);
    Serial.println();
  }
}
void ReadTemperatureData() {
  if (millis() - prev_millis2 > 10000) {                 // read temperature
    AlternatorTemperature = sensors.getTempFByIndex(0);  // fetch temperature
    sensors.requestTemperatures();                       // immediately after fetching the temperature we request a new sample in the async modus
    Serial.print("Alternator Temperature:");
    Serial.println(AlternatorTemperature);
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
    display.println(AlternatorTemperature, 1);

    display.setCursor(79, 22);
    display.println("t: ");
    display.setCursor(90, 22);
    display.println("XXX");

    display.setCursor(0, 33);
    display.println("PWM%:");
    display.setCursor(35, 33);
    display.println(DutyCycle, 1);  //String(val, decimalPlaces)

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
void EngineRapid(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double EngineSpeed;
  double EngineBoostPressure;
  int8_t EngineTiltTrim;

  if (ParseN2kEngineParamRapid(N2kMsg, EngineInstance, EngineSpeed, EngineBoostPressure, EngineTiltTrim)) {
    PrintLabelValWithConversionCheckUnDef("Engine rapid params: ", EngineInstance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  RPM: ", EngineSpeed, 0, true);
    PrintLabelValWithConversionCheckUnDef("  boost pressure (Pa): ", EngineBoostPressure, 0, true);
    PrintLabelValWithConversionCheckUnDef("  tilt trim: ", EngineTiltTrim, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void EngineDynamicParameters(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double EngineOilPress;
  double EngineOilTemp;
  double EngineCoolantTemp;
  double AltenatorVoltage;
  double FuelRate;
  double EngineHours;
  double EngineCoolantPress;
  double EngineFuelPress;
  int8_t EngineLoad;
  int8_t EngineTorque;
  tN2kEngineDiscreteStatus1 Status1;
  tN2kEngineDiscreteStatus2 Status2;

  if (ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress, EngineOilTemp, EngineCoolantTemp,
                                 AltenatorVoltage, FuelRate, EngineHours,
                                 EngineCoolantPress, EngineFuelPress,
                                 EngineLoad, EngineTorque, Status1, Status2)) {
    PrintLabelValWithConversionCheckUnDef("Engine dynamic params: ", EngineInstance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ", EngineOilPress, 0, true);
    PrintLabelValWithConversionCheckUnDef("  oil temp (C): ", EngineOilTemp, &KelvinToC, true);
    PrintLabelValWithConversionCheckUnDef("  coolant temp (C): ", EngineCoolantTemp, &KelvinToC, true);
    PrintLabelValWithConversionCheckUnDef("  altenator voltage (V): ", AltenatorVoltage, 0, true);
    PrintLabelValWithConversionCheckUnDef("  fuel rate (l/h): ", FuelRate, 0, true);
    PrintLabelValWithConversionCheckUnDef("  engine hours (h): ", EngineHours, &SecondsToh, true);
    PrintLabelValWithConversionCheckUnDef("  coolant pressure (Pa): ", EngineCoolantPress, 0, true);
    PrintLabelValWithConversionCheckUnDef("  fuel pressure (Pa): ", EngineFuelPress, 0, true);
    PrintLabelValWithConversionCheckUnDef("  engine load (%): ", EngineLoad, 0, true);
    PrintLabelValWithConversionCheckUnDef("  engine torque (%): ", EngineTorque, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void TransmissionParameters(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  tN2kTransmissionGear TransmissionGear;
  double OilPressure;
  double OilTemperature;
  unsigned char DiscreteStatus1;

  if (ParseN2kTransmissionParameters(N2kMsg, EngineInstance, TransmissionGear, OilPressure, OilTemperature, DiscreteStatus1)) {
    PrintLabelValWithConversionCheckUnDef("Transmission params: ", EngineInstance, 0, true);
    OutputStream->print("  gear: ");
    PrintN2kEnumType(TransmissionGear, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ", OilPressure, 0, true);
    PrintLabelValWithConversionCheckUnDef("  oil temperature (C): ", OilTemperature, &KelvinToC, true);
    PrintLabelValWithConversionCheckUnDef("  discrete status: ", DiscreteStatus1, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void TripFuelConsumption(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double TripFuelUsed;
  double FuelRateAverage;
  double FuelRateEconomy;
  double InstantaneousFuelEconomy;

  if (ParseN2kEngineTripParameters(N2kMsg, EngineInstance, TripFuelUsed, FuelRateAverage, FuelRateEconomy, InstantaneousFuelEconomy)) {
    PrintLabelValWithConversionCheckUnDef("Trip fuel consumption: ", EngineInstance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  fuel used (l): ", TripFuelUsed, 0, true);
    PrintLabelValWithConversionCheckUnDef("  average fuel rate (l/h): ", FuelRateAverage, 0, true);
    PrintLabelValWithConversionCheckUnDef("  economy fuel rate (l/h): ", FuelRateEconomy, 0, true);
    PrintLabelValWithConversionCheckUnDef("  instantaneous fuel economy (l/h): ", InstantaneousFuelEconomy, 0, true);
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
void UserDatumSettings(const tN2kMsg &N2kMsg) {
  if (N2kMsg.PGN != 129045L) return;
  int Index = 0;
  double val;

  OutputStream->println("User Datum Settings: ");
  val = N2kMsg.Get4ByteDouble(1e-2, Index);
  PrintLabelValWithConversionCheckUnDef("  delta x (m): ", val, 0, true);
  val = N2kMsg.Get4ByteDouble(1e-2, Index);
  PrintLabelValWithConversionCheckUnDef("  delta y (m): ", val, 0, true);
  val = N2kMsg.Get4ByteDouble(1e-2, Index);
  PrintLabelValWithConversionCheckUnDef("  delta z (m): ", val, 0, true);
  val = N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in x (deg): ", val, &RadToDeg, true, 5);
  val = N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in y (deg): ", val, &RadToDeg, true, 5);
  val = N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in z (deg): ", val, &RadToDeg, true, 5);
  val = N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  scale: ", val, 0, true, 3);
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
void LocalOffset(const tN2kMsg &N2kMsg) {
  uint16_t SystemDate;
  double SystemTime;
  int16_t Offset;

  if (ParseN2kLocalOffset(N2kMsg, SystemDate, SystemTime, Offset)) {
    OutputStream->println("Date,time and local offset: ");
    PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", SystemDate, 0, true);
    PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SystemTime, 0, true);
    PrintLabelValWithConversionCheckUnDef("  local offset (min): ", Offset, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void OutsideEnvironmental(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double WaterTemperature;
  double OutsideAmbientAirTemperature;
  double AtmosphericPressure;

  if (ParseN2kOutsideEnvironmentalParameters(N2kMsg, SID, WaterTemperature, OutsideAmbientAirTemperature, AtmosphericPressure)) {
    PrintLabelValWithConversionCheckUnDef("Water temp: ", WaterTemperature, &KelvinToC);
    PrintLabelValWithConversionCheckUnDef(", outside ambient temp: ", OutsideAmbientAirTemperature, &KelvinToC);
    PrintLabelValWithConversionCheckUnDef(", pressure: ", AtmosphericPressure, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void Temperature(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char TempInstance;
  tN2kTempSource TempSource;
  double ActualTemperature;
  double SetTemperature;

  if (ParseN2kTemperature(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature)) {
    OutputStream->print("Temperature source: ");
    PrintN2kEnumType(TempSource, OutputStream, false);
    PrintLabelValWithConversionCheckUnDef(", actual temperature: ", ActualTemperature, &KelvinToC);
    PrintLabelValWithConversionCheckUnDef(", set temperature: ", SetTemperature, &KelvinToC, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void Humidity(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char Instance;
  tN2kHumiditySource HumiditySource;
  double ActualHumidity, SetHumidity;

  if (ParseN2kHumidity(N2kMsg, SID, Instance, HumiditySource, ActualHumidity, SetHumidity)) {
    OutputStream->print("Humidity source: ");
    PrintN2kEnumType(HumiditySource, OutputStream, false);
    PrintLabelValWithConversionCheckUnDef(", humidity: ", ActualHumidity, 0, false);
    PrintLabelValWithConversionCheckUnDef(", set humidity: ", SetHumidity, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void Pressure(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char Instance;
  tN2kPressureSource PressureSource;
  double ActualPressure;

  if (ParseN2kPressure(N2kMsg, SID, Instance, PressureSource, ActualPressure)) {
    OutputStream->print("Pressure source: ");
    PrintN2kEnumType(PressureSource, OutputStream, false);
    PrintLabelValWithConversionCheckUnDef(", pressure: ", ActualPressure, &PascalTomBar, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void TemperatureExt(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char TempInstance;
  tN2kTempSource TempSource;
  double ActualTemperature;
  double SetTemperature;

  if (ParseN2kTemperatureExt(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature)) {
    OutputStream->print("Temperature source: ");
    PrintN2kEnumType(TempSource, OutputStream, false);
    PrintLabelValWithConversionCheckUnDef(", actual temperature: ", ActualTemperature, &KelvinToC);
    PrintLabelValWithConversionCheckUnDef(", set temperature: ", SetTemperature, &KelvinToC, true);
  } else {
    OutputStream->print("Failed to parse PGN: ");
    OutputStream->println(N2kMsg.PGN);
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
void printLLNumber(Stream *OutputStream, unsigned long long n, uint8_t base = 10) {
  unsigned char buf[16 * sizeof(long)];  // Assumes 8-bit chars.
  unsigned long long i = 0;

  if (n == 0) {
    OutputStream->print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    OutputStream->print((char)(buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
}

//*****************************************************************************
void BinaryStatusFull(const tN2kMsg &N2kMsg) {
  unsigned char BankInstance;
  tN2kBinaryStatus BankStatus;

  if (ParseN2kBinaryStatus(N2kMsg, BankInstance, BankStatus)) {
    OutputStream->print("Binary status for bank ");
    OutputStream->print(BankInstance);
    OutputStream->println(":");
    OutputStream->print("  ");  //printLLNumber(OutputStream,BankStatus,16);
    for (uint8_t i = 1; i <= 28; i++) {
      if (i > 1) OutputStream->print(",");
      PrintN2kEnumType(N2kGetStatusOnBinaryStatus(BankStatus, i), OutputStream, false);
    }
    OutputStream->println();
  }
}

//*****************************************************************************
void BinaryStatus(const tN2kMsg &N2kMsg) {
  unsigned char BankInstance;
  tN2kOnOff Status1, Status2, Status3, Status4;

  if (ParseN2kBinaryStatus(N2kMsg, BankInstance, Status1, Status2, Status3, Status4)) {
    if (BankInstance > 2) {  // note that this is only for testing different methods. MessageSender.ini sends 4 status for instace 2
      BinaryStatusFull(N2kMsg);
    } else {
      OutputStream->print("Binary status for bank ");
      OutputStream->print(BankInstance);
      OutputStream->println(":");
      OutputStream->print("  Status1=");
      PrintN2kEnumType(Status1, OutputStream, false);
      OutputStream->print(", Status2=");
      PrintN2kEnumType(Status2, OutputStream, false);
      OutputStream->print(", Status3=");
      PrintN2kEnumType(Status3, OutputStream, false);
      OutputStream->print(", Status4=");
      PrintN2kEnumType(Status4, OutputStream, false);
      OutputStream->println();
    }
  }
}

//*****************************************************************************
void FluidLevel(const tN2kMsg &N2kMsg) {
  unsigned char Instance;
  tN2kFluidType FluidType;
  double Level = 0;
  double Capacity = 0;

  if (ParseN2kFluidLevel(N2kMsg, Instance, FluidType, Level, Capacity)) {
    switch (FluidType) {
      case N2kft_Fuel:
        OutputStream->print("Fuel level :");
        break;
      case N2kft_Water:
        OutputStream->print("Water level :");
        break;
      case N2kft_GrayWater:
        OutputStream->print("Gray water level :");
        break;
      case N2kft_LiveWell:
        OutputStream->print("Live well level :");
        break;
      case N2kft_Oil:
        OutputStream->print("Oil level :");
        break;
      case N2kft_BlackWater:
        OutputStream->print("Black water level :");
        break;
      case N2kft_FuelGasoline:
        OutputStream->print("Gasoline level :");
        break;
      case N2kft_Error:
        OutputStream->print("Error level :");
        break;
      case N2kft_Unavailable:
        OutputStream->print("Unknown level :");
        break;
    }
    OutputStream->print(Level);
    OutputStream->print("%");
    OutputStream->print(" (");
    OutputStream->print(Capacity * Level / 100);
    OutputStream->print("l)");
    OutputStream->print(" capacity :");
    OutputStream->println(Capacity);
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



//RTC
void setPCF8526()
// this resets the stopwatch
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x01);
  Wire.write(decToBcd(0));  // second to reset to
  Wire.write(decToBcd(0));  // minute to reset to
  Wire.write(decToBcd(0));  // hour to reset to
  Wire.endTransmission();
}

void readPCF8526()
// this reads the stopwatch from the PCF8526
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(PCF8526address, 7);
  second = bcdToDec(Wire.read() & B01111111);  // remove VL error bit
  minute = bcdToDec(Wire.read() & B01111111);  // remove unwanted bits from MSB
  hour = bcdToDec(Wire.read() & B00111111);
}