// Working code for setting and then reading RTC
#include "Wire.h"
#define PCF8526address 0x51

void PCF8526oscOFF()
// turns off oscillator
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x0D);
  Wire.write(0);
  Wire.endTransmission();
}

byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
String days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

byte bcdToDec(byte value)
{
  return ((value / 16) * 10 + value % 16);
}

byte decToBcd(byte value){
  return (value / 10 * 16 + value % 10);
}

void PCF8526StopWatchMode()
// turns off oscillator
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x0D);
  Wire.write(0);
  Wire.endTransmission();
}

void setPCF8526()
// this sets the time and date to the PCF8526
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x01);
  Wire.write(decToBcd(second));  
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));     
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(dayOfWeek));  
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}

void readPCF8526()
// this gets the time and date from the PCF8526
{
  Wire.beginTransmission(PCF8526address);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(PCF8526address, 7);
  second     = bcdToDec(Wire.read() & B01111111); // remove VL error bit
  minute     = bcdToDec(Wire.read() & B01111111); // remove unwanted bits from MSB
  hour       = bcdToDec(Wire.read() & B00111111); 
  dayOfMonth = bcdToDec(Wire.read() & B00111111);
  dayOfWeek  = bcdToDec(Wire.read() & B00000111);  
  month      = bcdToDec(Wire.read() & B00011111);  // remove century bit, 1999 is over
  year       = bcdToDec(Wire.read());
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  // change the following to set your initial time
  second = 0;
  minute = 12;
  hour = 17;
  dayOfWeek = 6;
  dayOfMonth = 19;
  month = 10;
  year = 24;
 // PCF8525StopWatchMode();
  // comment out the next line and upload again to set and keep the time from resetting every reset
  setPCF8526();
}

void loop()
{
  readPCF8526();
  Serial.print(days[dayOfWeek]); 
  Serial.print(" ");  
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/20");
  Serial.print(year, DEC);
  Serial.print(" - ");
  Serial.print(hour, DEC);
  Serial.print(":");
  if (minute < 10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");  
  if (second < 10)
  {
    Serial.print("0");
  }  
  Serial.println(second, DEC);  
  delay(1000);
}