/*
ADS1115 lite library - Adapted from adafruit ADS1015/ADS1115 library

	This library is stripped down version with bug fixes from the adafruit ADS1015/ADS1115 library in order to save as much space as possible
		1. No explicit "Read ADC" functions exist to save space.  Simply set your mux manually, trigger a conversion manually, and read the value manually.  See the example program

		 
	https://github.com/terryjmyers/ADS1115-Lite.Git
    v1.0 - First release
*/

#include <ADS1115_lite.h>
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);  //Initializes wire library, sets private configuration variables to ADS1115 default(2.048V, 128SPS, Differential mode between  AIN0 and AIN1.  The Address parameter is not required if you want default

//Setup some variables used to show how long the ADC conversion takes place
unsigned long starttime;
unsigned long endtime;
float Raw;
float Channel0V;  // voltage divider is 20
float Channel1V;  // voltage divider is 2
float Channel2V;  // voltage divider is 2---- wired internally to LM2907
float Channel3V;  // voltage divider is an empty socket for user installation

void setup() {


  Serial.begin(115200);
  Serial.println();

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

  //Connection check
  if (!adc.testConnection()) {
    Serial.println("ADS1115 Connection failed");
    return;
  }
}

void loop() {

  //The mux setting must be set every time each channel is read, there is NOT a separate function call for each possible mux combination.
  adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  //Set single ended mode between AIN0 and GND
  //manually trigger the conversion
  starttime = micros();       //Record a start time for demonstration
  adc.triggerConversion();    //Start a conversion.  This immediatly returns
  Raw = adc.getConversion();  //This polls the ADS1115 and wait for conversion to finish, THEN returns the value
  endtime = micros();
  //Channel0V = Raw / 32768 * 6.144 * 20.242914979757085;
  Serial.print("Ch 0 RAW:");
  Serial.print(Raw);
  //Serial.print(Channel0V);
  Channel0V = Raw / 32768 * 6.144 * 20.24291;
  Serial.print("      Ch 0 Volts: ");
  Serial.println(Channel0V);
  //Serial.print(endtime - starttime);
  //Serial.println("us");

  adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
  adc.triggerConversion();
  Raw = adc.getConversion();
  Channel1V = Raw / 32768 * 6.144 * 2;
  Serial.print("Ch 1 Volts:");
  Serial.print(Channel1V);
  Serial.print("         Ch 1 RAW:");
  Serial.println(Raw);

  adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
  adc.triggerConversion();
  Raw = adc.getConversion();
  Channel2V = Raw / 32768 * 6.144 * 2;
  Serial.print("Ch 2 Volts:");
  Serial.println(Channel2V);

  adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);
  adc.triggerConversion();
  Raw = adc.getConversion();
  Channel3V = Raw / 32768 * 6.144;
  Serial.print("Ch 3 Volts:");
  Serial.println(Channel3V);

  Serial.println();
  Serial.println();
  Serial.println();


  delay(4000);
}
