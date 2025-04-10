
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
    digitalWrite(4, SIC450Enabler);  // Enable the SIC450

    if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust SIC450 every half second

      //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
      if (ManualFieldToggle == 0) {
        if (MeasuredAmps < TargetAmps && vout < (14 - interval)) {
          vout = vout + interval;
        }
        if (MeasuredAmps > TargetAmps && vout > (MinimumFieldVoltage + interval)) {
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

      int end33 = micros();              // End timing
      AnalogReadTime2 = end33 - start33;  // Store elapsed time
      if (AnalogReadTime2>AnalogReadTime){
        AnalogReadTime=AnalogReadTime2;
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







// void ReadAnalogInputs() {
//   if (millis() - prev_millis3 > 1000) {  // every 1 seconds read ads1115 and INA228
//     if (ADS1115Disconnected == 0) {
//       //The mux setting must be set every time each channel is read, there is NOT a separate function call for each possible mux combination.
//       adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  //Set single ended mode between AIN0 and GND
//       //manually trigger the conversion
//       adc.triggerConversion();    //Start a conversion.  This immediatly returns
//       Raw = adc.getConversion();  //This polls the ADS1115 and wait for conversion to finish, THEN returns the value
//       endtime = micros();
//       //Channel0V = Raw / 32768 * 6.144 * 20.242914979757085;
//       //Serial.print(Channel0V);
//       Channel0V = Raw / 32768 * 6.144 * 20.24291;
//       BatteryV = Channel0V;
//       if (BatteryV > 14.5) {
//         ChargingVoltageTarget = TargetFloatVoltage;
//       }

//       adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
//       adc.triggerConversion();
//       Raw = adc.getConversion();
//       Channel1V = Raw / 32768 * 6.144 * 2;
//       MeasuredAmps = (2.5 - Channel1V) * 80;
//       //Serial.print("      Measured Amps: ");
//       //Serial.print(MeasuredAmps);

//       adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
//       adc.triggerConversion();
//       Raw = adc.getConversion();
//       Channel2V = Raw / 32768 * 6.144 * 2133.2 * 2;
//       RPM = Channel2V;
//       // Serial.print("       Ch 2 Volts: ");
//       // Serial.print(Channel2V);

//       adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);
//       adc.triggerConversion();
//       Raw = adc.getConversion();
//       Channel3V = Raw / 32768 * 6.144 * 833;
//       //Serial.print("       Ch 3 HZ: ");
//       //Serial.println(Channel3V);
//       prev_millis3 = millis();
//     }
//     // vvout = (sic45x.getReadVout());
//     // iiout = (sic45x.getReadIout());
//     // DutyCycle = (sic45x.getReadDutyCycle());

//     if (INADisconnected == 0) {
//       //Serial.println();
//       //Serial.print("INA228 Battery Voltage: ");
//       IBV = INA.getBusVoltage();
//       // Serial.println(IBV);
//       // Serial.print("INA228 Battery Bcur (Amps): ");
//       ShuntVoltage_mV = INA.getShuntVoltage_mV();
//       Bcur = ShuntVoltage_mV * 10;
//       //Serial.print(Bcur);
//       //Serial.println();
//     }
//   }
// }

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
  if (millis() - prev_millis33 > 1000) {  // read VE data every 1 second
    if (VeDataOn == 1) {
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

void SendWifiData() {
  if (millis() - prev_millis5 > webgaugesinterval) {

    int start66 = micros();  // Start timing the wifi section
    WifiStrength = WiFi.RSSI();
    WifiHeartBeat = WifiHeartBeat + 1;
    if (WifiStrength >= -70) {
      // Send Events to the Web Client with the Sensor Readings
      //Serial.printf("DEBUG: AnalogReadTime=%d, VeTime=%d, SendWifiTime=%d\n", AnalogReadTime, VeTime, SendWifiTime);

      String payload = "{";
payload += "\"AlternatorTemperatureF\":" + safeFloat(AlternatorTemperatureF) + ",";
payload += "\"DutyCycle\":" + safeFloat(DutyCycle) + ",";
payload += "\"BatteryV\":" + safeFloat(BatteryV) + ",";
payload += "\"MeasuredAmps\":" + safeFloat(MeasuredAmps) + ",";
payload += "\"RPM\":" + safeFloat(RPM) + ",";
payload += "\"Channel3V\":" + safeFloat(Channel3V) + ",";
payload += "\"IBV\":" + safeFloat(IBV) + ",";
payload += "\"Bcur\":" + safeFloat(Bcur) + ",";
payload += "\"VictronVoltage\":" + safeFloat(VictronVoltage) + ",";
payload += "\"LoopTime\":" + safeFloat(LoopTime) + ",";
payload += "\"WifiStrength\":" + safeFloat(WifiStrength) + ",";
payload += "\"WifiHeartBeat\":" + safeFloat(WifiHeartBeat) + ",";
payload += "\"SendWifiTime\":" + safeFloat(SendWifiTime) + ",";
payload += "\"AnalogReadTime\":" + safeFloat(AnalogReadTime) + ",";
payload += "\"VeTime\":" + safeFloat(VeTime) + ",";
payload += "\"MaximumLoopTime\":" + safeFloat(MaximumLoopTime) + ",";
payload += "\"HeadingNMEA\":" + safeFloat(HeadingNMEA) + ",";
payload += "\"vvout\":" + safeFloat(vvout) + ",";
payload += "\"iiout\":" + safeFloat(iiout);
payload += "}";


events.send(payload.c_str(), "BulkData", millis());
} else {
      Serial.printf("Skipping all SSE updates due to weak Wi-Fi (%d dBm)\n", WiFi.RSSI());
    }
    int end66 = micros();                // End timing wifi stuff
    SendWifiTime = end66 - start66;  // Store the elapsed time
    prev_millis5 = millis();
    //Serial.println("event sent");
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
