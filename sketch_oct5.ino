
#include <OneWire.h>               // temp sensors
#include <DallasTemperature.h>     // temp sensors
#include <SPI.h>                   // display
#include <Wire.h>                  // something
#include <Adafruit_GFX.h>          // display
#include <Adafruit_SSD1306.h>      // display
#include <ADS1115_lite.h>          // measuring 4 analog inputs
#include "VeDirectFrameHandler.h"  // for victron communication
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);
#include "INA228.h"
INA228 INA(0x40);
#include "SiC45x.h"  // heart of system, DC/DC converter chip
SiC45x sic45x(0x1D);

//  Settings
float TargetAmps = 50;
float vout = 0.15;                       // default field output voltage
float interval = 0.1;                    // voltage step to adjust field by each time the loop runs
float fffr = 1200;                       // this is the switching frequency for SIC450
bool benchtest = 0;                      // set to 1 if the engine isn't going to be running during debugging, to prevent high field output
float AlternatorTemperatureLimit = 130;  // the offset appears to be +40 to +50 to get max alternator temp
int resolution = 12;                     // for OneWire measurement
bool VeDataOn = 1;                       // Set to 1 if VE serial data exists

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

// variables used to show how long the ADC conversion takes place
unsigned long starttime;
unsigned long endtime;

// onewire    Data wire is conntec to the Arduino digital pin 14
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);´
Adafruit_SSD1306 display(128, 64, 23, 18, 19, -1, 5);

//"Blink without delay" style timer variables used to control how often differnet parts of code execute
static unsigned long prev_millis4;   // used to delay checking of faults in the SiC450
static unsigned long prev_millis66;  //used to delay the updating of the display
static unsigned long prev_millis22;  // used to delay sampling of sic450
static unsigned long prev_millis3;   // used to delay sampling of ADS1115 to every 2 seconds for example
static unsigned long prev_millis2;   // used to delay sampling of temp sensor to every 2 seconds for example
static unsigned long prev_millis33;  // used to delay sampling of Serial Data (ve direct)


#define ESP32_CAN_TX_PIN GPIO_NUM_25  //NMEA0183 data
#define ESP32_CAN_RX_PIN GPIO_NUM_26  // Victron VE Direct
//VictronEnergy
VeDirectFrameHandler myve;

//temporary interrupt stuff- this is used to pause and ask for data from keyboard serial
//bool interrupted = 0;
//void IRAM_ATTR ISR() {
//  interrupted = 1;
//}

void setup() {

  Serial.begin(115200);
  Wire.begin();  // is this needed?  Probably was done elsewhere in some library?
  Serial.println();

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

  if (millis() - prev_millis2 > 5000) {                  // read temperature
    AlternatorTemperature = sensors.getTempFByIndex(0);  // fetch temperature
    sensors.requestTemperatures();                       // immediately after fetching the temperature we request a new sample in the async modus
    Serial.print("Alternator Temperature:");
    Serial.println(AlternatorTemperature);
    prev_millis2 = millis();
  }

  if (millis() - prev_millis3 > 2000) {  // every 2 seconds read ads1115 and INA228
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

    Serial.print("VictronVoltage:  ");
    Serial.println(VictronVoltage, 3);
  }

  if (millis() - prev_millis4 > 20000) {  // every 20 seconds check faults
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

  if (millis() - prev_millis22 > 500) {  // adjust SIC450 every half second

    sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
    sic45x.setVinOvFaultLimit(60);
    //sic45x.setPowerGoodOn(0.24);   // .9
    //sic45x.setPowerGoodOff(0.24);  // .85
    sic45x.setVoutOvFaultLimit(14);  // 115 The default value for this was very low according to datasheet, so I raise it here
    sic45x.setVoutOvWarnLimit(14);   // 110
    sic45x.setVoutUvWarnLimit(0);    // .9
    sic45x.setVoutUvFaultLimit(0);   // .8
    sic45x.setVoutMarginLow(0.3);    //.95
    sic45x.setVoutMarginHigh(0.3);   //105

    sic45x.setFrequencySwitch(fffr);  //range is 300 kHz to 1500 kHz, resolution is 50 kHz,

    //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
    if (benchtest == 0) {
      if (MeasuredAmps < TargetAmps && vout < 12.7) {
        vout = vout + interval;
      }
      if (MeasuredAmps > TargetAmps && vout > 1.75) {
        vout = vout - interval;
      }
      if (AlternatorTemperature > AlternatorTemperatureLimit && vout > 1.75) {
        vout = vout - (2 * interval);  // if no *2, the adjustments offset, so this makes the Temp correction win
      }

      //sic45x.setVoutCommand(vout);
      prev_millis22 = millis();
    }

    sic45x.setVoutCommand(vout);
  }

  ReadVEData();     //read Data from Victron VeDirect
  UpdateDisplay();  //
}

void UpdateDisplay() {
  if (millis() - prev_millis66 > 2000) {
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
  if (millis() - prev_millis33 > 2000) {
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