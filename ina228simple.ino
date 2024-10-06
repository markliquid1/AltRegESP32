//
//    FILE: INA228_dump_config.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: test - dump configuration
//     URL: https://github.com/RobTillaart/INA228

#include "INA228.h"

INA228 INA(0x40);
float IBV;          // Battery voltage from INA228
float ShuntVoltage_mV;  // Battery shunt voltage from INA228
float Bcur;          // Battery Bcur reading from INA228

void setup() {

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println();
  Serial.print("INA228_LIB_VERSION: ");
  Serial.println(INA228_LIB_VERSION);
  Serial.println();

  Wire.begin();
  if (!INA.begin()) {
    Serial.println("Could not connect. Fix and Reboot");
    while (1)
      ;
  }

  // at least 16ms for an update with these settings for moving average and conversion time
  INA.setMode(11);                       // Bh = Continuous shunt and bus voltage
  INA.setAverage(2);                     //0h = 1, 1h = 4, 2h = 16, 3h = 64, 4h = 128, 5h = 256, 6h = 512, 7h = 1024     Applies to all channels
  INA.setBusVoltageConversionTime(5);    // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs
  INA.setShuntVoltageConversionTime(5);  // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs



  //  REGISTER 0
  Serial.print("Accumulation: ");
  Serial.println(INA.getAccumulation());
  Serial.print("ConversionDelay: ");
  Serial.println(INA.getConversionDelay());
  Serial.print("TemperatureCompensation: ");
  Serial.println(INA.getTemperatureCompensation());
  Serial.print("ADCRange: ");
  Serial.println(INA.getADCRange());
  Serial.println();

  //  REGISTER 1
  Serial.print("Mode: ");
  Serial.println(INA.getMode());
  Serial.print("BusVoltageConversionTime: ");
  Serial.println(INA.getBusVoltageConversionTime());
  Serial.print("ShuntVoltageConversionTime: ");
  Serial.println(INA.getShuntVoltageConversionTime());
  // Serial.print("TemperatureConversionTime: ");
  // Serial.println(INA.getTemperatureConversionTime());
  Serial.print("getAverage: ");  //this should be called "averaging mode" or "number of averages"
  Serial.println(INA.getAverage());
  Serial.println();
  Serial.print("\n done");
}


void loop() {
  Serial.print("INA228 Battery Voltage: ");
  IBV = INA.getBusVoltage();
  Serial.println(IBV);
  Serial.print("INA228 Battery Bcur (Amps): ");
  ShuntVoltage_mV = INA.getShuntVoltage_mV();
  Bcur= ShuntVoltage_mV * 10;
  Serial.print(Bcur);
  Serial.println();


  delay(2000);
}


//  -- END OF FILE --
