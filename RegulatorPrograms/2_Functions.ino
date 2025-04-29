

void AdjustField() {
  if (Ignition == 1 && OnOff == 1) {

    if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust Field every half second
      digitalWrite(4, FieldEnabler);                           // Enable the Field

      //The below code is only used when engine is running and we want field to adjust to meet the alternator amps target
      if (ManualFieldToggle == 0) {
        if (MeasuredAmps < TargetAmps && vout < (14 - interval)) {  // if amps are low, add field (if field limit not exceeded
          vout = vout + interval;
        }
        if (MeasuredAmps > TargetAmps && vout > (MinimumFieldVoltage + interval)) {  // if amps are high and field isn't too low, drop field
          vout = vout - interval;
        }
        // HAVE TO MAKE SURE THESE VALUES DON'T GET TOO LOW FOR Field COMMAND VALIDITY.   THIS LOGIC IS ALSO NOT GREAT IF INTERVAL GETS BIG FOR ANY REASON
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
      // Update the field voltage here
    }
  } else {
    vout = MinimumFieldVoltage;  // start over from a low field voltage when it comes time to turn back on
  }
}

void ReadAnalogInputs() {


  if (millis() - lastINARead >= 900) {  // could go down to 600 here, but this logic belongs in Loop anyway
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
      BatteryCurrent_scaled = Bcur * 100;

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
            MeasuredAmps = (2.5 - Channel1V) * 80;  // alternator current
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

  // Lazy check and update of maximum values, clean this up later if desired
  if (!isnan(IBV) && IBV > IBVMax) IBVMax = IBV;
  if (MeasuredAmps > MeasuredAmpsMax) MeasuredAmpsMax = MeasuredAmps;
  if (RPM > RPMMax) RPMMax = RPM;
  if (!isnan(MaxAlternatorTemperatureF) && AlternatorTemperatureF > MaxAlternatorTemperatureF) MaxAlternatorTemperatureF = AlternatorTemperatureF;
}
void TempTask(void *parameter) {

  // a placeholder for the temperature measurment- uncomment this and comment out temp measurement for debugging
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));  // sleep for 1 second
  }

  // for (;;) {
  //   // Step 1: Trigger a conversion
  //   sensors.requestTemperaturesByAddress(tempDeviceAddress);

  //   // Step 2: Wait for conversion to complete while other things run
  //   vTaskDelay(pdMS_TO_TICKS(9000));  // This is the spacing between reads

  //   // Step 3: Read the completed result
  //   uint8_t scratchPad[9];
  //   if (sensors.readScratchPad(tempDeviceAddress, scratchPad)) {
  //     int16_t raw = (scratchPad[1] << 8) | scratchPad[0];
  //     float tempC = raw / 16.0;
  //     AlternatorTemperatureF = tempC * 1.8 + 32.0;
  //   } else {
  //     AlternatorTemperatureF = NAN;
  //     Serial.println("Temp read failed");
  //   }
  //   Serial.printf("Temp: %.2f °F at %lu ms\n", AlternatorTemperatureF, millis());
  //   // Immediately loop again — next conversion starts right now
  // }
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

  // //Then, the live sensor readings
  // else if (var == "ALTERNATORTEMPERATUREF") {
  //   return String(AlternatorTemperatureF);
  // } else if (var == "DUTYCYCLE") {
  //   return String(DutyCycle);
  // } else if (var == "BATTERYV") {
  //   return String(BatteryV);
  // } else if (var == "MEASA") {
  //   return String(MeasuredAmps);
  // } else if (var == "RPMM") {
  //   return String(RPM);
  // } else if (var == "ADSCH3VLTS") {
  //   return String(Channel3V);
  // } else if (var == "IBVV") {
  //   return String(IBV);
  // } else if (var == "BCURR") {
  //   return String(Bcur);
  // } else if (var == "VVOLT") {
  //   return String(VictronVoltage);
  // } else if (var == "GPSH") {
  //   return String(HeadingNMEA);
  // } else if (var == "FIELDVOLTS") {
  //   return String(vvout);
  // } else if (var == "LOOPTIME") {
  //   return String(LoopTime);
  // } else if (var == "MAXIMUMLOOPTIME") {
  //   return String(MaximumLoopTime);
  // } else if (var == "FIELDAMPS") {
  //   return String(iiout);
  // } else if (var == "WIFISTRENGTH") {
  //   return String(WifiStrength);
  // } else if (var == "WIFIHEARTBEAT") {
  //   return String(WifiHeartBeat);
  // } else if (var == "SENDWIFITIME") {
  //   return String(SendWifiTime);
  // } else if (var == "ANALOGREADTIME") {
  //   return String(AnalogReadTime);
  // } else if (var == "VETIME") {
  //   return String(VeTime);
  //}
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
      int start66 = micros();     // Start timing the wifi section
      printHeapStats();           //   Should be ~25–65 µs with no serial prints
      printBasicTaskStackInfo();  //Should be ~70–170 µs µs for 10 tasks (conservative estimate with no serial prints)
      updateCpuLoad();            //~200–250 for 10 tasks
      testTaskStats();            // 👈 Add this line to test
      // Build CSV string with all data as integers
      // Format: multiply floats by 10, 100 or 1000 to preserve decimal precision as needed
      // CSV field order: see index.html -> fields[] mapping
      char payload[1024];  // >1400 the wifi transmission won't fit in 1 packet
      snprintf(payload, sizeof(payload),
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               // Readings
               SafeInt(AlternatorTemperatureF),
               SafeInt(DutyCycle),
               SafeInt(BatteryV, 100),
               SafeInt(MeasuredAmps, 100),
               SafeInt(RPM),
               SafeInt(Channel3V, 100),
               SafeInt(IBV, 100),
               SafeInt(Bcur, 100),
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
               SafeInt(IBVMax, 100),
               SafeInt(MeasuredAmpsMax, 100),
               SafeInt(RPMMax),
               SafeInt(SoC_percent),
               SafeInt(EngineRunTime),
               SafeInt(EngineCycles),
               SafeInt(AlternatorOnTime),
               SafeInt(AlternatorFuelUsed),
               SafeInt(ChargedEnergy),
               SafeInt(DischargedEnergy),
               SafeInt(AlternatorChargedEnergy),
               SafeInt(MaxAlternatorTemperatureF),

               // Settings    (for the echoes)
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
     // Serial.print("Payload: ");          //For debug
     // Serial.println(payload);            // for debug
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
  const char *ap_ssid = "ALTERNATOR_CONFIG";  // Hardcoded AP SSID
  const char *ap_password = "alternator123";  // Hardcoded AP password

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
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("http://" + WiFi.softAPIP().toString());
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", WIFI_CONFIG_HTML);
  });

  server.on("/wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
    String ssid = request->getParam("ssid", true)->value();
    String password = request->getParam("password", true)->value();
    ssid.trim();
    password.trim();
    writeFile(LittleFS, "/ssid.txt", ssid.c_str());
    writeFile(LittleFS, "/pass.txt", password.c_str());

    String response = "<!DOCTYPE html><html><head><title>WiFi Setup</title>";
    response += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    response += "<style>:root{--primary:#333;--accent:#f60;--bg-light:#f5f5f5;--text-dark:#333;--card-light:#fff;--radius:4px}body{font-family:Arial,sans-serif;background-color:var(--bg-light);color:var(--text-dark);padding:20px;line-height:1.6;font-size:14px}h2{color:var(--text-dark);border-bottom:2px solid var(--accent);padding-bottom:.25rem;margin-top:1rem;margin-bottom:.75rem;font-size:18px}.card{background:var(--card-light);padding:16px;border-left:2px solid var(--accent);border-radius:var(--radius);box-shadow:0 1px 2px rgba(0,0,0,0.1);margin-bottom:16px}a{color:var(--accent);text-decoration:none;font-weight:bold}b{color:var(--text-dark);font-weight:bold}</style></head><body><div class='card'>";
    response += "<h2>WiFi Configuration Saved</h2>";
    response += "<p>The regulator will now attempt to connect to WiFi</p>";
    response += "<p><b>Next:</b> Close this page, connect your device to the ship WiFi, and browse to <b>http://alternator.local</b> (if your browser supports it) or the device's IP address (192.168.4.1)</p>";
    response += "</div></body></html>";

    request->send(200, "text/html", response);
  });

  server.begin();
}
void setupServer() {
  //This code handles web requests when a user changes settings on the web interface. In plain English:
  //First, it checks if the request includes a valid password. If not, it rejects the request with a "Forbidden" message.
  //If the password is correct, it looks at which setting the user is trying to change by checking which form field was submitted.
  //When it finds the matching parameter (like temperature limit, manual voltage, etc.), it:
  //Gets the new value the user entered
  //Saves that value to a file in the ESP32's storage system
  //Updates the corresponding variable in the program's memory
  //Finally, it sends back a confirmation message to the web browser with the value that was set.
  //This is essentially how the system handles all the setting changes from the web interface - validating the request,
  //identifying which setting to change, saving it permanently, updating it in memory, and confirming the action to the user.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html", false, [](const String &var) -> String {
      if (var == "PASSWORD") {
        return String("");  //
      }
      return processor(var);  // Keep all your normal variable replacements
    });
  });
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("password") || strcmp(request->getParam("password")->value().c_str(), requiredPassword) != 0) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }
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
      VeData = request->getParam(VD)->value().toInt();
    } else if (request->hasParam(N0)) {
      inputMessage = request->getParam(N0)->value();
      writeFile(LittleFS, "/NMEA0183Data1.txt", inputMessage.c_str());
      NMEA0183Data = request->getParam(N0)->value().toInt();
    } else if (request->hasParam(N2)) {
      inputMessage = request->getParam(N2)->value();
      writeFile(LittleFS, "/NMEA2KData1.txt", inputMessage.c_str());
      NMEA2KData = request->getParam(N2)->value().toInt();
    } else {
      inputMessage = "No message sent";
    }

    request->send(200, "text/plain", inputMessage);
  });

  server.on("/setPassword", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("password", true) || !request->hasParam("newpassword", true)) {
      request->send(400, "text/plain", "Missing fields");
      return;
    }

    String password = request->getParam("password", true)->value();
    String newPassword = request->getParam("newpassword", true)->value();

    password.trim();
    newPassword.trim();

    if (newPassword.length() == 0) {
      request->send(400, "text/plain", "Empty new password");
      return;
    }

    // Validate the existing admin password first
    if (!validatePassword(password.c_str())) {
      request->send(403, "text/plain", "FAIL");  // Wrong password
      return;
    }

    // Save the plaintext password for recovery
    File plainFile = LittleFS.open("/password.txt", "w");
    if (plainFile) {
      plainFile.println(newPassword);
      plainFile.close();
    }

    // Create and save the hash
    char hash[65] = { 0 };
    sha256(newPassword.c_str(), hash);

    File file = LittleFS.open("/password.hash", "w");
    if (!file) {
      request->send(500, "text/plain", "Failed to open password file");
      return;
    }

    file.println(hash);
    file.close();

    // Update RAM copy
    strncpy(requiredPassword, newPassword.c_str(), sizeof(requiredPassword) - 1);
    strncpy(storedPasswordHash, hash, sizeof(storedPasswordHash) - 1);

    request->send(200, "text/plain", "OK");
  });

  server.on("/checkPassword", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("password", true)) {
      request->send(400, "text/plain", "Missing password");
      return;
    }
    String password = request->getParam("password", true)->value();
    password.trim();

    if (validatePassword(password.c_str())) {
      request->send(200, "text/plain", "OK");
    } else {
      request->send(403, "text/plain", "FAIL");
    }
  });


  server.onNotFound([](AsyncWebServerRequest *request) {
    String path = request->url();
    Serial.print("Request for: ");
    Serial.println(path);

    if (LittleFS.exists(path)) {
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

  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
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

void printBasicTaskStackInfo() {
  numTasks = uxTaskGetNumberOfTasks();
  if (numTasks > MAX_TASKS) numTasks = MAX_TASKS;  // Prevent overflow

  tasksCaptured = uxTaskGetSystemState(taskArray, numTasks, NULL);

  Serial.println(F("\n===== TASK STACK REMAINING (BYTES) ====="));
  Serial.println(F("Task Name        | Core | Stack Remaining | Alert"));

  char coreIdBuffer[8];  // Buffer for core ID display

  for (int i = 0; i < tasksCaptured; i++) {
    const char *taskName = taskArray[i].pcTaskName;
    stackBytes = taskArray[i].usStackHighWaterMark * sizeof(StackType_t);
    core = taskArray[i].xCoreID;

    // Format core ID
    if (core < 0 || core > 16) {
      snprintf(coreIdBuffer, sizeof(coreIdBuffer), "N/A");
    } else {
      snprintf(coreIdBuffer, sizeof(coreIdBuffer), "%d", core);
    }

    const char *alert = "";
    if (
      strcmp(taskName, "IDLE0") == 0 || strcmp(taskName, "IDLE1") == 0 || strcmp(taskName, "ipc0") == 0 || strcmp(taskName, "ipc1") == 0) {
      if (stackBytes < 256) {
        alert = "LOW STACK";
      }
    } else {
      if (stackBytes < 256) {
        alert = "LOW STACK";
      } else if (stackBytes < 512) {
        alert = "WARN";
      }
    }


    Serial.printf("%-16s |  %-3s  |     %5d B     | %s\n",
                  taskName,
                  coreIdBuffer,
                  stackBytes,
                  alert);
  }

  Serial.println(F("==========================================\n"));
}

void printHeapStats() {
  rawFreeHeap = esp_get_free_heap_size();                                 // in bytes
  FreeHeap = rawFreeHeap / 1024;                                          // in KB
  MinFreeHeap = esp_get_minimum_free_heap_size() / 1024;                  // in KB
  FreeInternalRam = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;  // in KB

  if (rawFreeHeap == 0) {
    Heapfrag = 100;
  } else {
    Heapfrag = 100 - ((heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) * 100) / rawFreeHeap);
  }

  Serial.println(F("========== HEAP STATS =========="));
  Serial.printf("Free Heap:               %5u KB\n", FreeHeap);
  Serial.printf("Minimum Ever Free Heap:  %5u KB\n", MinFreeHeap);
  Serial.printf("Free Internal RAM:       %5u KB\n", FreeInternalRam);
  Serial.printf("Heap Fragmentation:      %5u %%\n", Heapfrag);
  Serial.println(F("================================"));
}

void updateCpuLoad() {
  const int MAX_TASKS = 20;
  TaskStatus_t taskSnapshot[MAX_TASKS];
  UBaseType_t taskCount = uxTaskGetSystemState(taskSnapshot, MAX_TASKS, NULL);
  unsigned long idle0Time = 0;
  unsigned long idle1Time = 0;
  unsigned long now = millis();
  for (int i = 0; i < taskCount; i++) {
    if (strcmp(taskSnapshot[i].pcTaskName, "IDLE0") == 0) {
      idle0Time = taskSnapshot[i].ulRunTimeCounter;
    } else if (strcmp(taskSnapshot[i].pcTaskName, "IDLE1") == 0) {
      idle1Time = taskSnapshot[i].ulRunTimeCounter;
    }
  }
  if (lastCheckTime == 0) {
    lastIdle0Time = idle0Time;
    lastIdle1Time = idle1Time;
    lastCheckTime = now;
    return;
  }
  unsigned long deltaIdle0 = idle0Time - lastIdle0Time;
  unsigned long deltaIdle1 = idle1Time - lastIdle1Time;
  unsigned long timeDiff = now - lastCheckTime;
  if (timeDiff == 0) return;
  // Fixed calculation - using your existing variables
  cpuLoadCore0 = 100 - ((deltaIdle0 * 100) / (timeDiff * 100));
  cpuLoadCore1 = 100 - ((deltaIdle1 * 100) / (timeDiff * 100));
  // Ensure values stay in valid range
  if (cpuLoadCore0 < 0) cpuLoadCore0 = 0;
  if (cpuLoadCore0 > 100) cpuLoadCore0 = 100;
  if (cpuLoadCore1 < 0) cpuLoadCore1 = 0;
  if (cpuLoadCore1 > 100) cpuLoadCore1 = 100;
  lastIdle0Time = idle0Time;
  lastIdle1Time = idle1Time;
  lastCheckTime = now;
  // Print CPU load directly
  Serial.printf("CPU Load: Core 0 = %3d%%, Core 1 = %3d%%\n", cpuLoadCore0, cpuLoadCore1);
}

void testTaskStats() {
  char statsBuffer[1024];  // Enough for 15–20 tasks

  vTaskGetRunTimeStats(statsBuffer);

  Serial.println(F("========== TASK CPU USAGE =========="));
  Serial.println(statsBuffer);
  Serial.println(F("====================================\n"));
}

void UpdateBatterySOC(unsigned long elapsedMillis) {
  // Convert elapsed milliseconds to seconds for calculations
  unsigned long elapsedSeconds = elapsedMillis / 1000;
  if (elapsedSeconds < 1) elapsedSeconds = 1;

  // Update scaled values
  Voltage_scaled = BatteryV * 100;
  AlternatorCurrent_scaled = MeasuredAmps * 100;
  BatteryPower_scaled = (Voltage_scaled * BatteryCurrent_scaled) / 100;  // W × 100
  EnergyDelta_scaled = (BatteryPower_scaled * elapsedSeconds) / 3600;
  AlternatorPower_scaled = (int)(BatteryV * MeasuredAmps * 100);  // W × 100
  AltEnergyDelta_scaled = (AlternatorPower_scaled * elapsedSeconds) / 3600;

  // Calculate fuel used based on alternator energy output (Wh × 100)
  joulesOut = (AltEnergyDelta_scaled * 3600) / 100;  // Joules
  fuelEnergyUsed_J = joulesOut * 2;                  // Assume 50% alternator efficiency
  AlternatorFuelUsed += (fuelEnergyUsed_J / 36000);  // Inline the mL calc

  alternatorIsOn = (AlternatorCurrent_scaled > CurrentThreshold_scaled);

  if (alternatorIsOn) {
    alternatorOnAccumulator += elapsedMillis;
    if (alternatorOnAccumulator >= 60000) {
      AlternatorOnTime += alternatorOnAccumulator / 60000;
      alternatorOnAccumulator %= 60000;
    }
  }

  alternatorWasOn = alternatorIsOn;

  // Correctly scaled threshold for BatteryCurrent_scaled
  if (abs(BatteryCurrent_scaled) < 50) return;  // 0.5 * 100 = 50

  // Use integer math for deltaAh
  int deltaAh_scaled = (BatteryCurrent_scaled * elapsedSeconds) / 3600;  // scaled by 100 to match CoulombCount_Ah_scaled

  if (BatteryCurrent_scaled >= 0) {
    // Apply charge efficiency (ChargeEfficiency_scaled is already percentage)
    int batteryDeltaAh_scaled = (deltaAh_scaled * ChargeEfficiency_scaled) / 100;
    CoulombCount_Ah_scaled += batteryDeltaAh_scaled;
  } else {
    // Apply Peukert compensation (PeukertExponent_scaled is × 100)
    int batteryDeltaAh_scaled = (deltaAh_scaled * 100) / PeukertExponent_scaled;
    CoulombCount_Ah_scaled += batteryDeltaAh_scaled;
  }

  CoulombCount_Ah_scaled = constrain(CoulombCount_Ah_scaled, 0, BatteryCapacity_Ah * 100);
  SoC_percent = CoulombCount_Ah_scaled / BatteryCapacity_Ah / 100;  // divide by 100 because CoulombCount is scaled

  // --- Full Charge Detection (Integer Only, No Temps) ---
  if ((abs(BatteryCurrent_scaled) <= (TailCurrent_scaled * BatteryCapacity_Ah)) && (Voltage_scaled >= ChargedVoltage_scaled)) {
    FullChargeTimer += elapsedSeconds;
    if (FullChargeTimer >= ChargedDetectionTime) {
      SoC_percent = 100;
      CoulombCount_Ah_scaled = BatteryCapacity_Ah * 100;
      FullChargeDetected = true;
    }
  } else {
    FullChargeTimer = 0;
    FullChargeDetected = false;
  }
}

void UpdateEngineRuntime(unsigned long elapsedMillis) {
  // Check if engine is running (RPM > 100)
  bool engineIsRunning = (RPM > 100 && RPM < 6000);

  if (engineIsRunning) {
    // Add time to engine running counter
    engineRunAccumulator += elapsedMillis;

    // Update total engine run time every minute
    if (engineRunAccumulator >= 60000) {  // 1 minute in milliseconds
      int minutesRun = engineRunAccumulator / 60000;
      EngineRunTime += minutesRun;

      // Update engine cycles (RPM * minutes)
      EngineCycles += RPM * minutesRun;

      // Keep the remainder milliseconds
      engineRunAccumulator %= 60000;
    }
  }

  // Update engine state
  engineWasRunning = engineIsRunning;
}

void SaveAllData() {
  // Save all persistent energy data
  // Create directory if it doesn't exist (LittleFS doesn't need this, but included for completeness)
  // Write files, creating them if they don't exist
  writeFile(LittleFS, "/AltEnergy.txt", String(AlternatorChargedEnergy).c_str());
  writeFile(LittleFS, "/FuelUsed.txt", String(AlternatorFuelUsed).c_str());
  writeFile(LittleFS, "/IBVMax.txt", String(IBVMax, 3).c_str());
  writeFile(LittleFS, "/MeasuredAmpsMax.txt", String(MeasuredAmpsMax, 3).c_str());
  writeFile(LittleFS, "/RPMMax.txt", String(RPMMax).c_str());
  writeFile(LittleFS, "/SoC_percent.txt", String(SoC_percent).c_str());
  writeFile(LittleFS, "/EngineRunTime.txt", String(EngineRunTime).c_str());
  writeFile(LittleFS, "/EngineCycles.txt", String(EngineCycles).c_str());
  writeFile(LittleFS, "/AlternatorOnTime.txt", String(AlternatorOnTime).c_str());
  writeFile(LittleFS, "/AlternatorFuelUsed.txt", String(AlternatorFuelUsed).c_str());
  writeFile(LittleFS, "/ChargedEnergy.txt", String(ChargedEnergy).c_str());
  writeFile(LittleFS, "/DischargedEnergy.txt", String(DischargedEnergy).c_str());
  writeFile(LittleFS, "/AlternatorChargedEnergy.txt", String(AlternatorChargedEnergy).c_str());
  writeFile(LittleFS, "/MaxAlternatorTemperatureF.txt", String(MaxAlternatorTemperatureF).c_str());
}

void ResetRuntimeCounters() {
  // Reset runtime tracking variables
  EngineRunTime = 0;
  EngineCycles = 0;
  AlternatorOnTime = 0;
  engineRunAccumulator = 0;
  alternatorOnAccumulator = 0;

  // Save the reset values
  SaveAllData();
}

void ResetEnergyCounters() {
  // Reset all energy tracking variables
  ChargedEnergy = 0;
  DischargedEnergy = 0;
  AlternatorChargedEnergy = 0;
  AlternatorFuelUsed = 0;

  // Save the reset values
  SaveAllData();
}

void sha256(const char *input, char *outputBuffer) {  // for security
  byte shaResult[32];
  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, info, 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char *)input, strlen(input));
  mbedtls_md_finish(&ctx, shaResult);
  mbedtls_md_free(&ctx);

  for (int i = 0; i < 32; ++i) {
    sprintf(outputBuffer + (i * 2), "%02x", shaResult[i]);
  }
}

void loadPasswordHash() {
  // First try to load plaintext password (for auth)
  if (LittleFS.exists("/password.txt")) {
    File plainFile = LittleFS.open("/password.txt", "r");
    if (plainFile) {
      String pwdStr = plainFile.readStringUntil('\n');
      pwdStr.trim();
      strncpy(requiredPassword, pwdStr.c_str(), sizeof(requiredPassword) - 1);
      plainFile.close();
      Serial.println("Plaintext password loaded from LittleFS");
    }
  }

  // Now load the hash (for future use)
  if (LittleFS.exists("/password.hash")) {
    File file = LittleFS.open("/password.hash", "r");
    if (file) {
      size_t len = file.readBytesUntil('\n', storedPasswordHash, sizeof(storedPasswordHash) - 1);
      storedPasswordHash[len] = '\0';  // null-terminate
      file.close();
      Serial.println("Password hash loaded from LittleFS");
      return;
    }
  }

  // If we get here, no password files exist - set defaults
  strncpy(requiredPassword, "admin", sizeof(requiredPassword) - 1);
  sha256("admin", storedPasswordHash);
  Serial.println("No password file, using default admin password");
}

void savePasswordHash() {
  File file = LittleFS.open("/password.hash", "w");
  if (file) {
    file.println(storedPasswordHash);
    file.close();
    Serial.println("Password hash saved to LittleFS");
  } else {
    Serial.println("Failed to open password.hash for writing");
  }
}

void savePasswordPlaintext(const char *password) {
  File file = LittleFS.open("/password.txt", "w");
  if (file) {
    file.println(password);
    file.close();
    Serial.println("Password saved to LittleFS");
  } else {
    Serial.println("Failed to open password.txt for writing");
  }
}

bool validatePassword(const char *password) {
  if (!password) return false;

  char hash[65] = { 0 };
  sha256(password, hash);

  return (strcmp(hash, storedPasswordHash) == 0);
}

void InitSystemSettings() {  // load all settings from LittleFS.  If no files exist, create them and populate with the hardcoded values

  if (!LittleFS.exists("/BatteryCapacity.txt")) {
    writeFile(LittleFS, "/BatteryCapacity.txt", String(BatteryCapacity_Ah).c_str());
  } else {
    BatteryCapacity_Ah = readFile(LittleFS, "/BatteryCapacity.txt").toInt();
  }

  if (!LittleFS.exists("/PeukertExponent.txt")) {
    writeFile(LittleFS, "/PeukertExponent.txt", String(PeukertExponent_scaled).c_str());
  } else {
    PeukertExponent_scaled = readFile(LittleFS, "/PeukertExponent.txt").toInt();
  }

  if (!LittleFS.exists("/ChargeEfficiency.txt")) {
    writeFile(LittleFS, "/ChargeEfficiency.txt", String(ChargeEfficiency_scaled).c_str());
  } else {
    ChargeEfficiency_scaled = readFile(LittleFS, "/ChargeEfficiency.txt").toInt();
  }

  if (!LittleFS.exists("/ChargedVoltage.txt")) {
    writeFile(LittleFS, "/ChargedVoltage.txt", String(ChargedVoltage_scaled).c_str());
  } else {
    ChargedVoltage_scaled = readFile(LittleFS, "/ChargedVoltage.txt").toInt();
  }

  if (!LittleFS.exists("/TailCurrent.txt")) {
    writeFile(LittleFS, "/TailCurrent.txt", String(TailCurrent_scaled).c_str());
  } else {
    TailCurrent_scaled = readFile(LittleFS, "/TailCurrent.txt").toInt();
  }

  if (!LittleFS.exists("/FuelEfficiency.txt")) {
    writeFile(LittleFS, "/FuelEfficiency.txt", String(FuelEfficiency_scaled).c_str());
  } else {
    FuelEfficiency_scaled = readFile(LittleFS, "/FuelEfficiency.txt").toInt();
  }
  //////////////////////////////////
  if (!LittleFS.exists("/TemperatureLimitF.txt")) {
    writeFile(LittleFS, "/TemperatureLimitF.txt", String(AlternatorTemperatureLimitF).c_str());
  } else {
    AlternatorTemperatureLimitF = readFile(LittleFS, "/TemperatureLimitF.txt").toInt();
  }
  if (!LittleFS.exists("/ManualVoltage.txt")) {
    writeFile(LittleFS, "/ManualVoltage.txt", String(ManualVoltageTarget).c_str());
  } else {
    ManualVoltageTarget = readFile(LittleFS, "/ManualVoltage.txt").toFloat();  // these float values might become problematic for payload size?
  }
  if (!LittleFS.exists("/FullChargeVoltage.txt")) {
    writeFile(LittleFS, "/FullChargeVoltage.txt", String(ChargingVoltageTarget).c_str());
  } else {
    ChargingVoltageTarget = readFile(LittleFS, "/FullChargeVoltage.txt").toFloat();
  }
  if (!LittleFS.exists("/TargetAmpz.txt")) {
    writeFile(LittleFS, "/TargetAmpz.txt", String(TargetAmps).c_str());
  } else {
    TargetAmps = readFile(LittleFS, "/TargetAmpz.txt").toInt();
  }
  if (!LittleFS.exists("/SwitchingFrequency.txt")) {
    writeFile(LittleFS, "/SwitchingFrequency.txt", String(fffr).c_str());
  } else {
    fffr = readFile(LittleFS, "/SwitchingFrequency.txt").toInt();
  }
  if (!LittleFS.exists("/TargetFloatVoltage1.txt")) {
    writeFile(LittleFS, "/TargetFloatVoltage1.txt", String(TargetFloatVoltage).c_str());
  } else {
    TargetFloatVoltage = readFile(LittleFS, "/TargetFloatVoltage1.txt").toFloat();
  }
  if (!LittleFS.exists("/interval1.txt")) {
    writeFile(LittleFS, "/interval1.txt", String(interval).c_str());
  } else {
    interval = readFile(LittleFS, "/interval1.txt").toFloat();
  }
  if (!LittleFS.exists("/FieldAdjustmentInterval1.txt")) {
    writeFile(LittleFS, "/FieldAdjustmentInterval1.txt", String(FieldAdjustmentInterval).c_str());
  } else {
    FieldAdjustmentInterval = readFile(LittleFS, "/FieldAdjustmentInterval1.txt").toFloat();
  }
  if (!LittleFS.exists("/ManualFieldToggle1.txt")) {
    writeFile(LittleFS, "/ManualFieldToggle1.txt", String(ManualFieldToggle).c_str());
  } else {
    ManualFieldToggle = readFile(LittleFS, "/ManualFieldToggle1.txt").toInt();
  }
  if (!LittleFS.exists("/SwitchControlOverride1.txt")) {
    writeFile(LittleFS, "/SwitchControlOverride1.txt", String(SwitchControlOverride).c_str());
  } else {
    SwitchControlOverride = readFile(LittleFS, "/SwitchControlOverride1.txt").toInt();
  }
  if (!LittleFS.exists("/ForceFloat1.txt")) {
    writeFile(LittleFS, "/ForceFloat1.txt", String(ForceFloat).c_str());
  } else {
    ForceFloat = readFile(LittleFS, "/ForceFloat1.txt").toInt();
  }
  if (!LittleFS.exists("/OnOff1.txt")) {
    writeFile(LittleFS, "/OnOff1.txt", String(OnOff).c_str());
  } else {
    OnOff = readFile(LittleFS, "/OnOff1.txt").toInt();
  }
  if (!LittleFS.exists("/HiLow1.txt")) {
    writeFile(LittleFS, "/HiLow1.txt", String(HiLow).c_str());
  } else {
    HiLow = readFile(LittleFS, "/HiLow1.txt").toInt();
  }
  if (!LittleFS.exists("/LimpHome1.txt")) {
    writeFile(LittleFS, "/LimpHome1.txt", String(LimpHome).c_str());
  } else {
    LimpHome = readFile(LittleFS, "/LimpHome1.txt").toInt();
  }
  if (!LittleFS.exists("/VeData1.txt")) {
    writeFile(LittleFS, "/VeData1.txt", String(VeData).c_str());
  } else {
    VeData = readFile(LittleFS, "/VeData1.txt").toInt();
  }
  if (!LittleFS.exists("/NMEA0183Data1.txt")) {
    writeFile(LittleFS, "/NMEA0183Data1.txt", String(NMEA0183Data).c_str());
  } else {
    NMEA0183Data = readFile(LittleFS, "/NMEA0183Data1.txt").toInt();
  }
  if (!LittleFS.exists("/NMEA2KData1.txt")) {
    writeFile(LittleFS, "/NMEA2KData1.txt", String(NMEA2KData).c_str());
  } else {
    NMEA2KData = readFile(LittleFS, "/NMEA2KData1.txt").toInt();
  }
}

void InitPersistentVariables() {

  // Initializer for all Persistent variables  involving littleFS

  if (!LittleFS.exists("/IBVMax.txt")) {                            // if the Flash storage file does not exist
    writeFile(LittleFS, "/IBVMax.txt", String(IBVMax, 3).c_str());  // create the file, and save the current RAM variable value
  } else {                                                          // otherwise
    IBVMax = readFile(LittleFS, "/IBVMax.txt").toFloat();           // update the variable value in RAM from the Flash storage
  }
  if (!LittleFS.exists("/MeasuredAmpsMax.txt")) {
    writeFile(LittleFS, "/MeasuredAmpsMax.txt", String(MeasuredAmpsMax, 3).c_str());
  } else {
    MeasuredAmpsMax = readFile(LittleFS, "/MeasuredAmpsMax.txt").toFloat();
  }
  if (!LittleFS.exists("/RPMMax.txt")) {
    writeFile(LittleFS, "/RPMMax.txt", String(RPMMax).c_str());
  } else {
    RPMMax = readFile(LittleFS, "/RPMMax.txt").toInt();
  }
  if (!LittleFS.exists("/SoC_percent.txt")) {  // Does this really belong here?
    writeFile(LittleFS, "/SoC_percent.txt", String(SoC_percent).c_str());
  } else {
    SoC_percent = readFile(LittleFS, "/SoC_percent.txt").toInt();
  }
  if (!LittleFS.exists("/EngineRunTime.txt")) {
    writeFile(LittleFS, "/EngineRunTime.txt", String(EngineRunTime).c_str());
  } else {
    EngineRunTime = readFile(LittleFS, "/EngineRunTime.txt").toInt();
  }
  if (!LittleFS.exists("/EngineCycles.txt")) {
    writeFile(LittleFS, "/EngineCycles.txt", String(EngineCycles).c_str());
  } else {
    EngineCycles = readFile(LittleFS, "/EngineCycles.txt").toInt();
  }
  if (!LittleFS.exists("/AlternatorOnTime.txt")) {
    writeFile(LittleFS, "/AlternatorOnTime.txt", String(AlternatorOnTime).c_str());
  } else {
    AlternatorOnTime = readFile(LittleFS, "/AlternatorOnTime.txt").toInt();
  }
  if (!LittleFS.exists("/AlternatorFuelUsed.txt")) {
    writeFile(LittleFS, "/AlternatorFuelUsed.txt", String(AlternatorFuelUsed).c_str());
  } else {
    AlternatorFuelUsed = readFile(LittleFS, "/AlternatorFuelUsed.txt").toInt();
  }
  if (!LittleFS.exists("/ChargedEnergy.txt")) {
    writeFile(LittleFS, "/ChargedEnergy.txt", String(ChargedEnergy).c_str());
  } else {
    ChargedEnergy = readFile(LittleFS, "/ChargedEnergy.txt").toInt();
  }
  if (!LittleFS.exists("/DischargedEnergy.txt")) {
    writeFile(LittleFS, "/DischargedEnergy.txt", String(DischargedEnergy).c_str());
  } else {
    DischargedEnergy = readFile(LittleFS, "/DischargedEnergy.txt").toInt();
  }
  if (!LittleFS.exists("/AlternatorChargedEnergy.txt")) {
    writeFile(LittleFS, "/AlternatorChargedEnergy.txt", String(AlternatorChargedEnergy).c_str());
  } else {
    AlternatorChargedEnergy = readFile(LittleFS, "/AlternatorChargedEnergy.txt").toInt();
  }
  if (!LittleFS.exists("/MaxAlternatorTemperatureF.txt")) {
    writeFile(LittleFS, "/MaxAlternatorTemperatureF.txt", String(MaxAlternatorTemperatureF).c_str());
  } else {
    MaxAlternatorTemperatureF = readFile(LittleFS, "/MaxAlternatorTemperatureF.txt").toInt();
  }
}
