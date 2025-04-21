
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
String processor(const String &var) {
  //The processor() is responsible for searching for placeholders in the HTML text and replacing them with actual values saved on LittleFS.
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
int SafeInt(float f, int scale = 1) {
  // where this is matters!!   Put utility functions like SafeInt() above setup() and loop() , according to ChatGPT.  And I proved it matters.
  return isnan(f) || isinf(f) ? -1 : (int)(f * scale);
}

void SendWifiData() {
  if (millis() - prev_millis5 > webgaugesinterval) {
    WifiStrength = WiFi.RSSI();
    WifiHeartBeat++;
    if (WifiStrength >= -70) {
      int start66 = micros();               // Start timing the wifi section
      FreeHeap = ESP.getFreeHeap() / 1024;  // making it smaller to transmit in kb
      // Build CSV string with all data as integers
      // Format: multiply floats by 10, 100 or 1000 to preserve decimal precision as needed
      char payload[512];  // Smaller buffer size since CSV is more compact
      snprintf(payload, sizeof(payload),
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               // Readings
               SafeInt(AlternatorTemperatureF),
               SafeInt(DutyCycle),
               SafeInt(BatteryV, 100),
               SafeInt(MeasuredAmps, 10),
               SafeInt(RPM),
               SafeInt(Channel3V, 100),
               SafeInt(IBV, 100),
               SafeInt(Bcur, 10),
               SafeInt(VictronVoltage, 100),
               SafeInt(LoopTime),
               SafeInt(WifiStrength),
               SafeInt(WifiHeartBeat),
               SafeInt(SendWifiTime),
               SafeInt(AnalogReadTime),
               SafeInt(VeTime),
               SafeInt(MaximumLoopTime),
               SafeInt(HeadingNMEA),
               SafeInt(vvout, 100),
               SafeInt(iiout, 10),
               SafeInt(FreeHeap),

               // Settings
               SafeInt(AlternatorTemperatureLimitF),
               SafeInt(ChargingVoltageTarget, 100),
               SafeInt(TargetAmps),
               SafeInt(TargetFloatVoltage, 100),
               SafeInt(fffr),
               SafeInt(interval, 100),
               SafeInt(FieldAdjustmentInterval),
               SafeInt(ManualVoltageTarget, 100),
               SafeInt(SwitchControlOverride),
               SafeInt(OnOff),
               SafeInt(ManualFieldToggle),
               SafeInt(HiLow),
               SafeInt(LimpHome),
               SafeInt(VeData),
               SafeInt(NMEA0183Data),
               SafeInt(NMEA2KData));
      events.send(payload, "CSVData");    // Changed event name to reflect new format
      SendWifiTime = micros() - start66;  // Calculate WiFi Send Time
    }

    prev_millis5 = millis();
  }
}

void checkAndRestart() {
  //Restart the ESP32 every hour just for maintenance because we can eventaually want to use littleFS to store Battery Monitor Stuff first
  unsigned long currentMillis = millis();

  // Check if millis() has rolled over (happens after ~49.7 days)
  if (currentMillis < lastRestartTime) {
    lastRestartTime = 0;  // Reset on overflow
  }

  // Check if it's time to restart
  if (currentMillis - lastRestartTime >= RESTART_INTERVAL) {
    Serial.println("Performing scheduled restart for system maintenance");

    // Optional: send a message to the web client before restarting
    events.send("Device restarting for maintenance. Will reconnect shortly.", "status");

    // Allow time for the message to be sent
    delay(500);

    // Restart the ESP32
    ESP.restart();

    // This line won't be reached, but for clarity:
    lastRestartTime = currentMillis;
  }
}


// Function to set up WiFi - tries to connect to saved network, falls back to AP mode
void setupWiFi() {
  // Try to get saved WiFi credentials
  String saved_ssid = readFile(LittleFS, WIFI_SSID_FILE);
  String saved_password = readFile(LittleFS, WIFI_PASS_FILE);

  // If no saved credentials, use defaults
  if (saved_ssid.length() == 0) {
    saved_ssid = default_ssid;
    saved_password = default_password;
  }

  Serial.println("Attempting to connect to WiFi network: " + saved_ssid);

  // Try to connect to the WiFi network
  if (connectToWiFi(saved_ssid.c_str(), saved_password.c_str(), WIFI_TIMEOUT)) {
    // Successfully connected to WiFi
    currentWiFiMode = AWIFI_MODE_CLIENT;
    Serial.println("Connected to WiFi network!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Set up the web server for normal operation
    setupServer();
  } else {
    // Failed to connect to WiFi, create access point instead
    Serial.println("Failed to connect to WiFi. Starting AP mode for configuration...");
    setupAccessPoint();
    currentWiFiMode = AWIFI_MODE_AP;
  }
}
// Function to connect to a WiFi network with timeout
bool connectToWiFi(const char *ssid, const char *password, unsigned long timeout) {
  if (strlen(ssid) == 0) {
    Serial.println("No SSID provided");
    return false;
  }

  Serial.printf("Attempting to connect to WiFi network: %s\n", ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  unsigned long startTime = millis();
  
  // Wait for connection or timeout
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Initialize mDNS after successful connection
    if (MDNS.begin("alternator")) {
      Serial.println("mDNS responder started");
      // Add service to mDNS
      MDNS.addService("http", "tcp", 80);
    } else {
      Serial.println("Error setting up mDNS responder!");
    }
    
    return true;
  } else {
    Serial.println("Failed to connect to WiFi within timeout period");
    return false;
  }
}
// Function to set up the device as an access point
void setupAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);

  Serial.println("Access Point Started");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start DNS server for captive portal
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  // Set up the configuration web server
  setupWiFiConfigServer();
}
// Function to set up the configuration web server in AP mode
void setupWiFiConfigServer() {
  // Handle DNS requests for captive portal
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("http://" + WiFi.softAPIP().toString());
  });

  // Serve the WiFi configuration page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", WIFI_CONFIG_HTML);
  });

  // Handle form submission for saving WiFi credentials
  server.on("/saveWiFi", HTTP_POST, [](AsyncWebServerRequest *request) {
    String new_ssid;
    String new_password;

    if (request->hasParam("ssid", true)) {
      new_ssid = request->getParam("ssid", true)->value();
    }

    if (request->hasParam("password", true)) {
      new_password = request->getParam("password", true)->value();
    }

    // Save the new credentials to LittleFS
    if (new_ssid.length() > 0) {
      writeFile(LittleFS, WIFI_SSID_FILE, new_ssid.c_str());
      writeFile(LittleFS, WIFI_PASS_FILE, new_password.c_str());

      // Redirect with success message
      request->redirect("/?status=saved");

      // Set a timer to restart the ESP after a short delay
      // This gives time for the response to be sent to the client
      Serial.println("New WiFi credentials saved. Restarting in 5 seconds...");
      delay(5000);
      ESP.restart();
    } else {
      // Redirect with error message
      request->redirect("/?status=error");
    }
  });

  server.begin();
}
// Standard server setup for normal operation mode
void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html", false, processor);
  });

  // Send a GET request to <ESP_IP>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(TLimit)) {
      inputMessage = request->getParam(TLimit)->value();
      writeFile(LittleFS, "/TemperatureLimitF.txt", inputMessage.c_str());
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
    } else {
      inputMessage = "No message sent";
    }

    request->send(200, "text/text", inputMessage);
  });

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
}
// Process DNS requests for captive portal
void dnsHandleRequest() {
  if (currentWiFiMode == AWIFI_MODE_AP) {
    dnsServer.processNextRequest();
  }
}



// void reportStackUsageEverySecond() {
//   static unsigned long lastCheck = 0;
//   static UBaseType_t minLoopStack = UINT32_MAX;
//   static UBaseType_t minMyTaskStack = UINT32_MAX;

//   if (millis() - lastCheck >= 1000) {
//     UBaseType_t loopStack = uxTaskGetStackHighWaterMark(NULL);  // current task (loop)
//     UBaseType_t taskStack = tempTaskHandle ? uxTaskGetStackHighWaterMark(tempTaskHandle) : 0;

//     if (loopStack < minLoopStack) minLoopStack = loopStack;
//     if (taskStack < minMyTaskStack) minMyTaskStack = taskStack;

//     float loopUsedPct = 100.0f * (LOOP_STACK_ALLOC_WORDS - minLoopStack) / LOOP_STACK_ALLOC_WORDS;
//     float taskUsedPct = myTaskHandle
//                           ? 100.0f * (MYTASK_STACK_ALLOC_WORDS - minMyTaskStack) / MYTASK_STACK_ALLOC_WORDS
//                           : 0.0f;

//     Serial.printf("Stack usage: loop %.1f%% | myTask %.1f%%\n", loopUsedPct, taskUsedPct);

//     // Reset for next interval
//     minLoopStack = UINT32_MAX;
//     minMyTaskStack = UINT32_MAX;
//     lastCheck = millis();
//   }
// }
