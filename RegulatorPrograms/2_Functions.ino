// X Engineering Alternator Regulator
//     Copyright (C) 2025  Mark Nickerson

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//  See <https://www.gnu.org/licenses/> for GNU General Public License

// Contact me at mark@xengineering.net

void setDutyPercent(int percent) {  // Function to set PWM duty cycle by percentage
  percent = constrain(percent, 0, 100);
  uint32_t duty = (65535UL * percent) / 100;
  ledcWrite(pwmPin, duty);  // In v3.x, first parameter is the pin number
}

void AdjustField() {
  if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust field every FieldAdjustmentInterval milliseconds
    float currentBatteryVoltage = getBatteryVoltage();
    // Check if charging should be enabled (ignition on, system enabled, BMS allows)
    chargingEnabled = (Ignition == 1 && OnOff == 1);
    // Check BMS override if BMS logic is enabled--- this is a manual human setting in the user interface
    if (BMSlogic == 1) {
      // If BMS signal is active (based on BMSLogicLevelOff setting)
      bmsSignalActive = !digitalRead(36);  // this is the signal from the BMS itself (need "!"" because of optocouplers)
      if (BMSLogicLevelOff == 0) {
        // BMS gives LOW signal when charging NOT desired
        chargingEnabled = chargingEnabled && bmsSignalActive;
      } else {
        // BMS gives HIGH signal when charging NOT desired
        chargingEnabled = chargingEnabled && !bmsSignalActive;
      }
    }
    if (chargingEnabled) {           // if the BMS doesn't want charging, this is skipped, but otherwise....
      digitalWrite(4, 1);            // Enable the Field FieldEnable
      if (ManualFieldToggle == 0) {  // Automatic mode       // Should move this outside the BMS logic at some point..
        // Step 1: Determine base target amps from Hi/Low setting
        if (HiLow == 1) {
          uTargetAmps = TargetAmps;  // Normal target
        } else {
          uTargetAmps = TargetAmpLA;  // Low target
        }
        // Step 2: Apply RPM-based modification if enabled
        if (AmpControlByRPM == 1 && RPM > 100 && RPM < 6000) {
          int rpmBasedAmps = interpolateAmpsFromRPM(RPM);

          // Apply RPM curve but respect Hi/Low setting
          if (HiLow == 0) {
            // In Low mode, use lesser of RPM curve or low setting
            uTargetAmps = min(rpmBasedAmps, (int)(TargetAmpLA));
          } else {
            // In Normal mode, use RPM curve directly
            uTargetAmps = rpmBasedAmps;
          }
        }

        // Step 2.5, figure out the actual amps reading of whichever value we are controlling on
        targetCurrent = getTargetAmps();
        //Step 2.6 figure out the actual temp reading o whichever value we are controlling on
        if (TempSource == 0) {
          TempToUse = AlternatorTemperatureF;
        }
        if (TempSource == 1) {
          TempToUse = temperatureThermistor;
        }
        // Step 3: Apply control logic to adjust duty cycle
        // Increase duty cycle if below target and not at maximum
        if (targetCurrent < uTargetAmps && dutyCycle < (MaxDuty - dutyStep)) {
          dutyCycle += dutyStep;
        }
        // Decrease duty cycle if above target and not at minimum
        if (targetCurrent > uTargetAmps && dutyCycle > (MinDuty + dutyStep)) {
          dutyCycle -= dutyStep;
        }
        // Temperature protection (more aggressive reduction)
        if (!IgnoreTemperature && TempToUse > AlternatorTemperatureLimitF && dutyCycle > (MinDuty + 2 * dutyStep)) {
          dutyCycle -= 2 * dutyStep;
          queueConsoleMessage("Temp limit reached, backing off...");
        }
        // Voltage protection (most aggressive reduction)
        // float currentBatteryVoltage = getBatteryVoltage();  // was this not redundant?  delete later
        if (currentBatteryVoltage > ChargingVoltageTarget && dutyCycle > (MinDuty + 3 * dutyStep)) {
          dutyCycle -= 3 * dutyStep;
          queueConsoleMessage("Voltage limit reached, backing off...");
        }
        // Battery current protection (safety limit)
        if (Bcur > MaximumAllowedBatteryAmps && dutyCycle > (MinDuty + dutyStep)) {
          dutyCycle -= dutyStep;
          queueConsoleMessage("Battery current limit reached, backing off...");
        }
        // Ensure duty cycle stays within bounds
        dutyCycle = constrain(dutyCycle, MinDuty, MaxDuty);  //Critical that no charging can happen at MinDuty!!
      }

      else {  // Manual override mode
        dutyCycle = ManualDutyTarget;
        uTargetAmps = -999;  // just useful for debugging, delete later
        // Ensure duty cycle stays within bounds
        dutyCycle = constrain(dutyCycle, MinDuty, MaxDuty);
      }
    } else {
      // Charging disabled: shut down field and reset for next enable
      digitalWrite(4, 0);  // Disable the Field (FieldEnable)
      dutyCycle = MinDuty;
      uTargetAmps = -888;  // just useful for debugging, delete later
    }
    // Apply the calculated duty cycle
    setDutyPercent((int)dutyCycle);
    DutyCycle = dutyCycle;                            //shoddy work, oh well
    vvout = dutyCycle / 100 * currentBatteryVoltage;  //
    iiout = vvout / FieldResistance;
    // Update timer (only once)
    prev_millis22 = millis();


/// delete this whole thing later
    static unsigned long lastRunTime2g = 0;
    const unsigned long interval2g = 10000;  // 10 seconds

    if (millis() - lastRunTime2g >= interval2g) {
      lastRunTime2g = millis();

      if (dutyCycle <= (MinDuty + 1.0)) {
        Serial.println();
        String msg = " utargetAmps=" + String((float)uTargetAmps, 1)
                     + " targetCurrent=" + String(targetCurrent, 1)
                     + " currentBatteryVoltage=" + String(currentBatteryVoltage, 2) + "V"
                     + " TempToUse=" + String((float)TempToUse, 1) + "F"
                     + " dutyCycle=" + String(dutyCycle, 1);
        queueConsoleMessage(msg);
      }
    }
  }
}

void ReadAnalogInputs() {
  if (millis() - lastINARead >= 900) {  // could go down to 600 here, but this logic belongs in Loop anyway
    if (INADisconnected == 0) {
      int start33 = micros();  // Start timing analog input reading
      lastINARead = millis();
      IBV = INA.getBusVoltage();
      ShuntVoltage_mV = INA.getShuntVoltage_mV();
      //Bcur = (ShuntVoltage_mV * 1000.0) / ShuntResistance_uOhm;
      //Example:
      //For a 100 µΩ shunt and 5 mV reading:
      // Bcur = (5.0 * 1000.0) / 100.0 = 50.0 Amps
      Bcur = ShuntVoltage_mV * 1000.0f / ShuntResistanceMicroOhm;  //shunt is 0.1 mΩ or 100uOohms, Bcur is in Amps
      if (InvertBattAmps == 1) {
        Bcur = Bcur * -1;  // swap sign if necessary
      }
      Bcur = Bcur + BatteryCOffset;
      BatteryCurrent_scaled = Bcur * 100;
      int end33 = micros();               // End timing
      AnalogReadTime2 = end33 - start33;  // Store elapsed time
      if (AnalogReadTime2 > AnalogReadTime) {
        AnalogReadTime = AnalogReadTime2;
      }
    }
  }

  //ADS1115 reading is based on trigger→wait→read   so as to not waste time.  That is way the below is so complicated
  if (ADS1115Disconnected != 0) {
    queueConsoleMessage("theADS1115 was not connected and triggered a return");

    return;
  }
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
            Channel0V = Raw / 32767.0 * 6.144 / 0.0697674419;  // voltage divider is 1,000,000 and 75,000 ohms
            BatteryV = Channel0V;
            if (BatteryV > 14.5) {
              ChargingVoltageTarget = TargetFloatVoltage;  // this needs to be placed somewhere else someday
            }
            break;
          case 1:
            Channel1V = Raw / 32767.0 * 6.144 * 2;  // voltage divider is 2:1, so this gets us to volts
            // Amps=100×(Vin−2.5)
            MeasuredAmps = (Channel1V - 2.5) * 100;  // alternator current
            if (InvertAltAmps == 1) {
              MeasuredAmps = MeasuredAmps * -1;  // swap sign if necessary
            }
            MeasuredAmps = MeasuredAmps - AlternatorCOffset;
            break;
          case 2:
            Channel2V = Raw / 32767.0 * 2 * 6.144 * RPMScalingFactor;  // voltage divider is 2:1,  Guess and check to develop RPMScaingFactor
            RPM = Channel2V;
            if (RPM < 100) {
              RPM = 0;
            }
            break;
          case 3:
            Channel3V = Raw / 32767.0 * 6.144 * 833 * 2;  // Does nothing right now, thermistor someday?     The /2 is because of voltage divider
            temperatureThermistor = thermistorTempC(Channel3V);
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
  if (!isnan(MaxTemperatureThermistor) && temperatureThermistor > MaxTemperatureThermistor) {
    MaxTemperatureThermistor = temperatureThermistor;
  }
}
void TempTask(void *parameter) {

  // a placeholder for the temperature measurment- uncomment this and comment out temp measurement for debugging
  // for (;;) {
  //   vTaskDelay(pdMS_TO_TICKS(1000));  // sleep for 1 second
  // }

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
      queueConsoleMessage("WARNING: Temp sensor read failed");
    }
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

    if (millis() - prev_millis33 > 2000) {  // read VE data every 2 second

      int start1 = micros();  // Start timing VeData
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
        yield();  // not sure what this does
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

int SafeInt(float f, int scale = 1) {
  // where this is matters!!   Put utility functions like SafeInt() above setup() and loop() , according to ChatGPT.  And I proved it matters.
  return isnan(f) || isinf(f) ? -1 : (int)(f * scale);
}
void SendWifiData() {
  if (millis() - prev_millis5 > webgaugesinterval) {
    WifiStrength = WiFi.RSSI();
    WifiHeartBeat++;
    // Process console message queue - send one message per interval
    processConsoleQueue();
    if (WifiStrength >= -70) {
      int start66 = micros();     // Start timing the wifi section
      printHeapStats();           //   Should be ~25–65 µs with no serial prints
      printBasicTaskStackInfo();  //Should be ~70–170 µs µs for 10 tasks (conservative estimate with no serial prints)
      updateCpuLoad();            //~200–250 for 10 tasks
      testTaskStats();            // 👈 Add this line to test
                                  // Build CSV string with all data as integers
                                  // Format: multiply floats by 10, 100 or 1000 to preserve decimal precision as needed
                                  // CSV field order: see index.html -> fields[] mapping
      char payload[1400];         // >1400 the wifi transmission won't fit in 1 packet
      snprintf(payload, sizeof(payload),
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               // Readings
               SafeInt(AlternatorTemperatureF),     // 0
               SafeInt(DutyCycle),                  // 1
               SafeInt(BatteryV, 100),              // 2
               SafeInt(MeasuredAmps, 100),          // 3
               SafeInt(RPM),                        // 4
               SafeInt(Channel3V, 100),             // 5
               SafeInt(IBV, 100),                   // 6
               SafeInt(Bcur, 100),                  // 7
               SafeInt(VictronVoltage, 100),        // 8
               SafeInt(LoopTime),                   // 9
               SafeInt(WifiStrength),               // 10
               SafeInt(WifiHeartBeat),              // 11
               SafeInt(SendWifiTime),               // 12
               SafeInt(AnalogReadTime),             // 13
               SafeInt(VeTime),                     // 14
               SafeInt(MaximumLoopTime),            // 15
               SafeInt(HeadingNMEA),                // 16
               SafeInt(vvout, 100),                 // 17
               SafeInt(iiout, 10),                  // 18
               SafeInt(FreeHeap),                   // 19
               SafeInt(IBVMax, 100),                // 20
               SafeInt(MeasuredAmpsMax, 100),       // 21
               SafeInt(RPMMax),                     // 22
               SafeInt(SoC_percent),                // 23
               SafeInt(EngineRunTime),              // 24
               SafeInt(EngineCycles),               // 25
               SafeInt(AlternatorOnTime),           // 26
               SafeInt(AlternatorFuelUsed),         // 27
               SafeInt(ChargedEnergy),              // 28
               SafeInt(DischargedEnergy),           // 29
               SafeInt(AlternatorChargedEnergy),    // 30
               SafeInt(MaxAlternatorTemperatureF),  // 31

               // Settings Echo
               SafeInt(AlternatorTemperatureLimitF),  // 32
               SafeInt(ChargingVoltageTarget, 100),   // 33
               SafeInt(TargetAmps),                   // 34
               SafeInt(TargetFloatVoltage, 100),      // 35
               SafeInt(fffr),                         // 36
               SafeInt(interval, 100),                // 37
               SafeInt(FieldAdjustmentInterval),      // 38
               SafeInt(ManualDutyTarget),             // 39
               SafeInt(SwitchControlOverride),        // 40
               SafeInt(OnOff),                        // 41
               SafeInt(ManualFieldToggle),            // 42
               SafeInt(HiLow),                        // 43
               SafeInt(LimpHome),                     // 44
               SafeInt(VeData),                       // 45
               SafeInt(NMEA0183Data),                 // 46
               SafeInt(NMEA2KData),                   // 47
               SafeInt(TargetAmpLA),                  // 48

               // More Settings
               SafeInt(CurrentThreshold_scaled),    // 49
               SafeInt(PeukertExponent_scaled),     // 50
               SafeInt(ChargeEfficiency_scaled),    // 51
               SafeInt(ChargedVoltage_scaled),      // 52
               SafeInt(TailCurrent_scaled),         // 53
               SafeInt(ChargedDetectionTime),       // 54
               SafeInt(IgnoreTemperature),          // 55
               SafeInt(BMSlogic),                   // 56
               SafeInt(BMSLogicLevelOff),           // 57
               SafeInt(AlarmActivate),              // 58
               SafeInt(TempAlarm),                  // 59
               SafeInt(VoltageAlarmHigh),           // 60
               SafeInt(VoltageAlarmLow),            // 61
               SafeInt(CurrentAlarmHigh),           // 62
               SafeInt(FourWay),                    // 63
               SafeInt(RPMScalingFactor),           // 64
               SafeInt(ResetTemp),                  // 65
               SafeInt(ResetVoltage),               // 66
               SafeInt(ResetCurrent),               // 67
               SafeInt(ResetEngineRunTime),         // 68
               SafeInt(ResetAlternatorOnTime),      // 69
               SafeInt(ResetEnergy),                // 70
               SafeInt(MaximumAllowedBatteryAmps),  // 71
               SafeInt(ManualSOCPoint),             // 72
               SafeInt(BatteryVoltageSource),       //73
               SafeInt(AmpControlByRPM),            //74
               SafeInt(RPM1),                       //75
               SafeInt(RPM2),                       //76
               SafeInt(RPM3),                       //77
               SafeInt(RPM4),                       //78
               SafeInt(Amps1),                      //79
               SafeInt(Amps2),                      //80
               SafeInt(Amps3),                      //81
               SafeInt(Amps4),                      //82
               SafeInt(RPM5),                       //83
               SafeInt(RPM6),                       //84
               SafeInt(RPM7),                       //85
               SafeInt(Amps5),                      //86
               SafeInt(Amps6),                      //87
               SafeInt(Amps7),                      //88
               SafeInt(ShuntResistanceMicroOhm),    //89
               SafeInt(InvertAltAmps),              // 90
               SafeInt(InvertBattAmps),             //91
               SafeInt(MaxDuty),                    //92
               SafeInt(MinDuty),                    //93
               SafeInt(FieldResistance),            //94
               SafeInt(maxPoints),                  //95
               SafeInt(AlternatorCOffset, 100),     //96
               SafeInt(BatteryCOffset, 100),        //97
               SafeInt(BatteryCapacity_Ah),
               SafeInt(AmpSrc),
               SafeInt(R_fixed, 100),  //100
               SafeInt(Beta, 100),
               SafeInt(T0_C, 100),         //102
               SafeInt(TempSource),        //103
               SafeInt(IgnitionOverride),  //104
               // More Readings
               SafeInt(temperatureThermistor),     // 105
               SafeInt(MaxTemperatureThermistor),  // 106
               SafeInt(VictronCurrent, 100)        // 107


      );


      events.send(payload, "CSVData");    // Changed event name to reflect new format
      SendWifiTime = micros() - start66;  // Calculate WiFi Send Time
    }
    prev_millis5 = millis();
  }
}

void checkAndRestart() {
  //Restart the ESP32 every hour just for maintenance
  unsigned long currentMillis = millis();

  // Check if millis() has rolled over (happens after ~49.7 days)
  if (currentMillis < lastRestartTime) {
    lastRestartTime = 0;  // Reset on overflow
  }

  // Check if it's time to restart
  if (currentMillis - lastRestartTime >= RESTART_INTERVAL) {
    queueConsoleMessage("Performing scheduled restart for system maintenance");

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
    request->send(LittleFS, "/index.html", "text/html");
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
  Serial.println("Serving ONLY index.html");

  server.begin();
}
void setupServer() {

  //Factory Reset Logic
  server.on("/factoryReset", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("password", true)) {
      request->send(400, "text/plain", "Missing password");
      return;
    }

    String password = request->getParam("password", true)->value();
    if (!validatePassword(password.c_str())) {
      request->send(403, "text/plain", "FAIL");
      return;
    }

    queueConsoleMessage("FACTORY RESET: Restoring all defaults...");

    // Delete ALL settings files - let InitSystemSettings() recreate with defaults
    const char *settingsFiles[] = {
      "/TemperatureLimitF.txt", "/ManualDuty.txt", "/FullChargeVoltage.txt",
      "/TargetAmps.txt", "/SwitchingFrequency.txt", "/TargetFloatVoltage.txt", "/R_fixed.txt", "/Beta.txt", "/T0_C.txt", "/TempSource.txt", "/interval.txt",
      "/FieldAdjustmentinterval.txt", "/ManualFieldToggle.txt",
      "/SwitchControlOverride.txt", "/OnOff.txt", "/HiLow.txt", "/LimpHome.txt", "/AmpSrc.txt", "/IgnitionOverride.txt",
      "/VeData.txt", "/NMEA0183Data.txt", "/NMEA2KData.txt", "/TargetAmpL.txt",
      "/CurrentThreshold.txt", "/PeukertExponent.txt", "/ChargeEfficiency.txt",
      "/ChargedVoltage.txt", "/TailCurrent.txt", "/ChargedDetectionTime.txt",
      "/IgnoreTemperature.txt", "/BMSLogic.txt", "/BMSLogicLevelOff.txt",
      "/AlarmActivate.txt", "/TempAlarm.txt", "/VoltageAlarmHigh.txt",
      "/VoltageAlarmLow.txt", "/CurrentAlarmHigh.txt", "/RPMScalingFactor.txt",
      "/MaximumAllowedBatteryAmps.txt", "/ManualSOCPoint.txt", "/BatteryVoltageSource.txt",
      "/AmpControlByRPM.txt", "/RPM1.txt", "/RPM2.txt", "/RPM3.txt", "/RPM4.txt",
      "/RPM5.txt", "/RPM6.txt", "/RPM7.txt", "/Amps1.txt", "/Amps2.txt",
      "/Amps3.txt", "/Amps4.txt", "/Amps5.txt", "/Amps6.txt", "/Amps7.txt",
      "/ShuntResistanceMicroOhm.txt", "/InvertAltAmps.txt", "/InvertBattAmps.txt",
      "/MaxDuty.txt", "/MinDuty.txt", "/FieldResistance.txt", "/maxPoints.txt",
      "/AlternatorCOffset.txt", "/BatteryCOffset.txt"
    };

    // Delete all settings files
    for (int i = 0; i < sizeof(settingsFiles) / sizeof(settingsFiles[0]); i++) {
      if (LittleFS.exists(settingsFiles[i])) {
        LittleFS.remove(settingsFiles[i]);
      }
    }

    // Reinitialize with defaults
    InitSystemSettings();

    queueConsoleMessage("FACTORY RESET: Complete! All settings restored to defaults.");
    request->send(200, "text/plain", "OK");
  });


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
    request->send(LittleFS, "/index.html", "text/html");
  });


  server.on(
    "/get", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("password") || strcmp(request->getParam("password")->value().c_str(), requiredPassword) != 0) {
        request->send(403, "text/plain", "Forbidden");
        return;
      }

      String inputMessage;

      if (request->hasParam("TemperatureLimitF")) {
        inputMessage = request->getParam("TemperatureLimitF")->value();
        writeFile(LittleFS, "/TemperatureLimitF.txt", inputMessage.c_str());
        AlternatorTemperatureLimitF = inputMessage.toInt();
      } else if (request->hasParam("ManualDuty")) {
        inputMessage = request->getParam("ManualDuty")->value();
        writeFile(LittleFS, "/ManualDuty.txt", inputMessage.c_str());
        ManualDutyTarget = inputMessage.toInt();
      } else if (request->hasParam("FullChargeVoltage")) {
        inputMessage = request->getParam("FullChargeVoltage")->value();
        writeFile(LittleFS, "/FullChargeVoltage.txt", inputMessage.c_str());
        ChargingVoltageTarget = inputMessage.toFloat();
      } else if (request->hasParam("TargetAmps")) {
        inputMessage = request->getParam("TargetAmps")->value();
        writeFile(LittleFS, "/TargetAmps.txt", inputMessage.c_str());
        TargetAmps = inputMessage.toInt();
      } else if (request->hasParam("SwitchingFrequency")) {
        inputMessage = request->getParam("SwitchingFrequency")->value();
        writeFile(LittleFS, "/SwitchingFrequency.txt", inputMessage.c_str());
        fffr = inputMessage.toInt();
      } else if (request->hasParam("TargetFloatVoltage")) {
        inputMessage = request->getParam("TargetFloatVoltage")->value();
        writeFile(LittleFS, "/TargetFloatVoltage.txt", inputMessage.c_str());
        TargetFloatVoltage = inputMessage.toFloat();
      } else if (request->hasParam("interval")) {
        inputMessage = request->getParam("interval")->value();
        writeFile(LittleFS, "/interval.txt", inputMessage.c_str());
        interval = inputMessage.toFloat();
      } else if (request->hasParam("FieldAdjustmentinterval")) {
        inputMessage = request->getParam("FieldAdjustmentinterval")->value();
        writeFile(LittleFS, "/FieldAdjustmentinterval.txt", inputMessage.c_str());
        FieldAdjustmentInterval = inputMessage.toFloat();
      } else if (request->hasParam("ManualFieldToggle")) {
        inputMessage = request->getParam("ManualFieldToggle")->value();
        writeFile(LittleFS, "/ManualFieldToggle.txt", inputMessage.c_str());
        ManualFieldToggle = inputMessage.toInt();
      } else if (request->hasParam("SwitchControlOverride")) {
        inputMessage = request->getParam("SwitchControlOverride")->value();
        writeFile(LittleFS, "/SwitchControlOverride.txt", inputMessage.c_str());
        SwitchControlOverride = inputMessage.toInt();
      } else if (request->hasParam("ForceFloat1")) {
        inputMessage = request->getParam("ForceFloat1")->value();
        writeFile(LittleFS, "/ForceFloat1.txt", inputMessage.c_str());
        ForceFloat = inputMessage.toInt();
      } else if (request->hasParam("OnOff")) {
        inputMessage = request->getParam("OnOff")->value();
        writeFile(LittleFS, "/OnOff.txt", inputMessage.c_str());
        OnOff = inputMessage.toInt();
      } else if (request->hasParam("HiLow")) {
        inputMessage = request->getParam("HiLow")->value();
        writeFile(LittleFS, "/HiLow.txt", inputMessage.c_str());
        HiLow = inputMessage.toInt();
      } else if (request->hasParam("InvertAltAmps")) {
        inputMessage = request->getParam("InvertAltAmps")->value();
        writeFile(LittleFS, "/InvertAltAmps.txt", inputMessage.c_str());
        InvertAltAmps = inputMessage.toInt();
      } else if (request->hasParam("InvertBattAmps")) {
        inputMessage = request->getParam("InvertBattAmps")->value();
        writeFile(LittleFS, "/InvertBattAmps.txt", inputMessage.c_str());
        InvertBattAmps = inputMessage.toInt();
      } else if (request->hasParam("MaxDuty")) {
        inputMessage = request->getParam("MaxDuty")->value();
        writeFile(LittleFS, "/MaxDuty.txt", inputMessage.c_str());
        MaxDuty = inputMessage.toInt();
      } else if (request->hasParam("MinDuty")) {
        inputMessage = request->getParam("MinDuty")->value();
        writeFile(LittleFS, "/MinDuty.txt", inputMessage.c_str());
        MinDuty = inputMessage.toInt();
      } else if (request->hasParam("LimpHome")) {
        inputMessage = request->getParam("LimpHome")->value();
        writeFile(LittleFS, "/LimpHome.txt", inputMessage.c_str());
        LimpHome = inputMessage.toInt();
      } else if (request->hasParam("VeData")) {
        inputMessage = request->getParam("VeData")->value();
        writeFile(LittleFS, "/VeData.txt", inputMessage.c_str());
        VeData = inputMessage.toInt();
      } else if (request->hasParam("NMEA0183Data")) {
        inputMessage = request->getParam("NMEA0183Data")->value();
        writeFile(LittleFS, "/NMEA0183Data.txt", inputMessage.c_str());
        NMEA0183Data = inputMessage.toInt();
      } else if (request->hasParam("NMEA2KData")) {
        inputMessage = request->getParam("NMEA2KData")->value();
        writeFile(LittleFS, "/NMEA2KData.txt", inputMessage.c_str());
        NMEA2KData = inputMessage.toInt();
      } else if (request->hasParam("TargetAmpL")) {
        inputMessage = request->getParam("TargetAmpL")->value();
        writeFile(LittleFS, "/TargetAmpL.txt", inputMessage.c_str());
        TargetAmpLA = inputMessage.toInt();
      } else if (request->hasParam("CurrentThreshold")) {
        inputMessage = request->getParam("CurrentThreshold")->value();
        writeFile(LittleFS, "/CurrentThreshold.txt", inputMessage.c_str());
        CurrentThreshold_scaled = inputMessage.toInt();
      } else if (request->hasParam("PeukertExponent")) {
        inputMessage = request->getParam("PeukertExponent")->value();
        writeFile(LittleFS, "/PeukertExponent.txt", inputMessage.c_str());
        PeukertExponent_scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargeEfficiency")) {
        inputMessage = request->getParam("ChargeEfficiency")->value();
        writeFile(LittleFS, "/ChargeEfficiency.txt", inputMessage.c_str());
        ChargeEfficiency_scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargedVoltage")) {
        inputMessage = request->getParam("ChargedVoltage")->value();
        writeFile(LittleFS, "/ChargedVoltage.txt", inputMessage.c_str());
        ChargedVoltage_scaled = inputMessage.toInt();
      } else if (request->hasParam("TailCurrent")) {
        inputMessage = request->getParam("TailCurrent")->value();
        writeFile(LittleFS, "/TailCurrent.txt", inputMessage.c_str());
        TailCurrent_scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargedDetectionTime")) {
        inputMessage = request->getParam("ChargedDetectionTime")->value();
        writeFile(LittleFS, "/ChargedDetectionTime.txt", inputMessage.c_str());
        ChargedDetectionTime = inputMessage.toInt();
      } else if (request->hasParam("IgnoreTemperature")) {
        inputMessage = request->getParam("IgnoreTemperature")->value();
        writeFile(LittleFS, "/IgnoreTemperature.txt", inputMessage.c_str());
        IgnoreTemperature = inputMessage.toInt();
      } else if (request->hasParam("BMSLogic")) {
        inputMessage = request->getParam("BMSLogic")->value();
        writeFile(LittleFS, "/BMSLogic.txt", inputMessage.c_str());
        BMSlogic = inputMessage.toInt();
      } else if (request->hasParam("BMSLogicLevelOff")) {
        inputMessage = request->getParam("BMSLogicLevelOff")->value();
        writeFile(LittleFS, "/BMSLogicLevelOff.txt", inputMessage.c_str());
        BMSLogicLevelOff = inputMessage.toInt();
      } else if (request->hasParam("AlarmActivate")) {
        inputMessage = request->getParam("AlarmActivate")->value();
        writeFile(LittleFS, "/AlarmActivate.txt", inputMessage.c_str());
        AlarmActivate = inputMessage.toInt();
      } else if (request->hasParam("TempAlarm")) {
        inputMessage = request->getParam("TempAlarm")->value();
        writeFile(LittleFS, "/TempAlarm.txt", inputMessage.c_str());
        TempAlarm = inputMessage.toInt();
      } else if (request->hasParam("VoltageAlarmHigh")) {
        inputMessage = request->getParam("VoltageAlarmHigh")->value();
        writeFile(LittleFS, "/VoltageAlarmHigh.txt", inputMessage.c_str());
        VoltageAlarmHigh = inputMessage.toInt();
      } else if (request->hasParam("VoltageAlarmLow")) {
        inputMessage = request->getParam("VoltageAlarmLow")->value();
        writeFile(LittleFS, "/VoltageAlarmLow.txt", inputMessage.c_str());
        VoltageAlarmLow = inputMessage.toInt();
      } else if (request->hasParam("CurrentAlarmHigh")) {
        inputMessage = request->getParam("CurrentAlarmHigh")->value();
        writeFile(LittleFS, "/CurrentAlarmHigh.txt", inputMessage.c_str());
        CurrentAlarmHigh = inputMessage.toInt();
      } else if (request->hasParam("FourWay")) {
        inputMessage = request->getParam("FourWay")->value();
        writeFile(LittleFS, "/FourWay.txt", inputMessage.c_str());
        FourWay = inputMessage.toInt();
      } else if (request->hasParam("RPMScalingFactor")) {
        inputMessage = request->getParam("RPMScalingFactor")->value();
        writeFile(LittleFS, "/RPMScalingFactor.txt", inputMessage.c_str());
        RPMScalingFactor = inputMessage.toInt();
      } else if (request->hasParam("FieldResistance")) {
        inputMessage = request->getParam("FieldResistance")->value();
        writeFile(LittleFS, "/FieldResistance.txt", inputMessage.c_str());
        FieldResistance = inputMessage.toInt();
      } else if (request->hasParam("AlternatorCOffset")) {
        inputMessage = request->getParam("AlternatorCOffset")->value();
        writeFile(LittleFS, "/AlternatorCOffset.txt", inputMessage.c_str());
        AlternatorCOffset = inputMessage.toFloat();
      } else if (request->hasParam("BatteryCOffset")) {
        inputMessage = request->getParam("BatteryCOffset")->value();
        writeFile(LittleFS, "/BatteryCOffset.txt", inputMessage.c_str());
        BatteryCOffset = inputMessage.toFloat();
      } else if (request->hasParam("AmpSrc")) {
        inputMessage = request->getParam("AmpSrc")->value();
        writeFile(LittleFS, "/AmpSrc.txt", inputMessage.c_str());
        AmpSrc = inputMessage.toInt();
        queueConsoleMessage("AmpSrc changed to: " + String(AmpSrc));  // Add this debug line
      } else if (request->hasParam("BatteryVoltageSource")) {
        inputMessage = request->getParam("BatteryVoltageSource")->value();
        writeFile(LittleFS, "/BatteryVoltageSource.txt", inputMessage.c_str());
        BatteryVoltageSource = inputMessage.toInt();
        queueConsoleMessage("Battery voltage source changed to: " + String(BatteryVoltageSource));  // Debug line
      } else if (request->hasParam("R_fixed")) {
        inputMessage = request->getParam("R_fixed")->value();
        writeFile(LittleFS, "/R_fixed.txt", inputMessage.c_str());
        R_fixed = inputMessage.toFloat();
      } else if (request->hasParam("Beta")) {
        inputMessage = request->getParam("Beta")->value();
        writeFile(LittleFS, "/Beta.txt", inputMessage.c_str());
        Beta = inputMessage.toFloat();
      } else if (request->hasParam("T0_C")) {
        inputMessage = request->getParam("T0_C")->value();
        writeFile(LittleFS, "/T0_C.txt", inputMessage.c_str());
        T0_C = inputMessage.toFloat();
      } else if (request->hasParam("TempSource")) {
        inputMessage = request->getParam("TempSource")->value();
        writeFile(LittleFS, "/TempSource.txt", inputMessage.c_str());
        TempSource = inputMessage.toInt();
      } else if (request->hasParam("IgnitionOverride")) {
        inputMessage = request->getParam("IgnitionOverride")->value();
        writeFile(LittleFS, "/IgnitionOverride.txt", inputMessage.c_str());
        IgnitionOverride = inputMessage.toInt();
      } else if (request->hasParam("ResetTemp")) {
        inputMessage = request->getParam("ResetTemp")->value();       // pointless
        writeFile(LittleFS, "/ResetTemp.txt", inputMessage.c_str());  // pointless
        ResetTemp = inputMessage.toInt();                             // pointless
        MaxAlternatorTemperatureF = 0;                                // reset the variable on ESP32 mem
        writeFile(LittleFS, "/MaxAlternatorTemperatureF.txt", "0");   // update littleFS
      } else if (request->hasParam("ResetVoltage")) {
        inputMessage = request->getParam("ResetVoltage")->value();       // pointless
        writeFile(LittleFS, "/ResetVoltage.txt", inputMessage.c_str());  // pointless
        ResetVoltage = inputMessage.toInt();                             // pointless
        IBVMax = 0;                                                      // reset the variable on ESP32 mem
        writeFile(LittleFS, "/IBVMax.txt", "0");                         // update littleFS
      } else if (request->hasParam("ResetCurrent")) {
        inputMessage = request->getParam("ResetCurrent")->value();       // pointless
        writeFile(LittleFS, "/ResetCurrent.txt", inputMessage.c_str());  // pointless
        ResetCurrent = inputMessage.toInt();                             // pointless
        MeasuredAmpsMax = 0;                                             // reset the variable on ESP32 mem
        writeFile(LittleFS, "/MeasuredAmpsMax.txt", "0");                // update littleFS
      } else if (request->hasParam("ResetEngineRunTime")) {
        inputMessage = request->getParam("ResetEngineRunTime")->value();       // pointless
        writeFile(LittleFS, "/ResetEngineRunTime.txt", inputMessage.c_str());  // pointless
        ResetEngineRunTime = inputMessage.toInt();                             // pointless
        EngineRunTime = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/EngineRunTime.txt", "0");                        // update littleFS
      } else if (request->hasParam("ResetAlternatorOnTime")) {
        inputMessage = request->getParam("ResetAlternatorOnTime")->value();       // pointless
        writeFile(LittleFS, "/ResetAlternatorOnTime.txt", inputMessage.c_str());  // pointless
        ResetAlternatorOnTime = inputMessage.toInt();                             // pointless
        AlternatorOnTime = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorOnTime.txt", "0");                        // update littleFS
      } else if (request->hasParam("ResetEnergy")) {
        inputMessage = request->getParam("ResetEnergy")->value();       // pointless
        writeFile(LittleFS, "/ResetEnergy.txt", inputMessage.c_str());  // pointless
        ResetEnergy = inputMessage.toInt();                             // pointless
        ChargedEnergy = 0;                                              // reset the variable on ESP32 mem
        writeFile(LittleFS, "/ChargedEnergy.txt", "0");                 // update littleFS
      } else if (request->hasParam("ResetDischargedEnergy")) {
        inputMessage = request->getParam("ResetDischargedEnergy")->value();       // pointless
        writeFile(LittleFS, "/ResetDischargedEnergy.txt", inputMessage.c_str());  // pointless
        ResetDischargedEnergy = inputMessage.toInt();                             // pointless
        DischargedEnergy = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/DischargedEnergy.txt", "0");                        // update littleFS
      } else if (request->hasParam("ResetFuelUsed")) {
        inputMessage = request->getParam("ResetFuelUsed")->value();       // pointless
        writeFile(LittleFS, "/ResetFuelUsed.txt", inputMessage.c_str());  // pointless
        ResetFuelUsed = inputMessage.toInt();                             // pointless
        AlternatorFuelUsed = 0;                                           // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorFuelUsed.txt", "0");              // update littleFS
      } else if (request->hasParam("ResetAlternatorChargedEnergy")) {
        inputMessage = request->getParam("ResetAlternatorChargedEnergy")->value();       // pointless
        writeFile(LittleFS, "/ResetAlternatorChargedEnergy.txt", inputMessage.c_str());  // pointless
        ResetAlternatorChargedEnergy = inputMessage.toInt();                             // pointless
        AlternatorChargedEnergy = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorChargedEnergy.txt", "0");                        // update littleFS
      } else if (request->hasParam("ResetEngineCycles")) {
        inputMessage = request->getParam("ResetEngineCycles")->value();       // pointless
        writeFile(LittleFS, "/ResetEngineCycles.txt", inputMessage.c_str());  // pointless
        ResetEngineCycles = inputMessage.toInt();                             // pointless
        EngineCycles = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/EngineCycles.txt", "0");                        // update littleFS
      } else if (request->hasParam("ResetRPMMax")) {
        inputMessage = request->getParam("ResetRPMMax")->value();       // pointless
        writeFile(LittleFS, "/ResetRPMMax.txt", inputMessage.c_str());  // pointless
        ResetRPMMax = inputMessage.toInt();                             // pointless
        RPMMax = 0;                                                     // reset the variable on ESP32 mem
        writeFile(LittleFS, "/RPMMax.txt", "0");                        // update littleFS
      } else if (request->hasParam("MaximumAllowedBatteryAmps")) {
        inputMessage = request->getParam("MaximumAllowedBatteryAmps")->value();
        writeFile(LittleFS, "/MaximumAllowedBatteryAmps.txt", inputMessage.c_str());
        MaximumAllowedBatteryAmps = inputMessage.toInt();
      } else if (request->hasParam("ManualSOCPoint")) {                        //pointless
        inputMessage = request->getParam("ManualSOCPoint")->value();           //pointless
        writeFile(LittleFS, "/ManualSOCPoint.txt", inputMessage.c_str());      //pointless
        ManualSOCPoint = inputMessage.toInt();                                 //pointless
        SoC_percent = ManualSOCPoint * 100;                                    // Convert user input to internal scaling//pointless
        writeFile(LittleFS, "/SoC_percent.txt", String(SoC_percent).c_str());  //pointless
      } else if (request->hasParam("ResetThermTemp")) {
        inputMessage = request->getParam("ResetThermTemp")->value();
        writeFile(LittleFS, "/ResetThermTemp.txt", inputMessage.c_str());
        ResetThermTemp = inputMessage.toInt();
        MaxTemperatureThermistor = 0;
        writeFile(LittleFS, "/MaxTemperatureThermistor.txt", "0");
      } else if (request->hasParam("BatteryVoltageSource")) {
        inputMessage = request->getParam("BatteryVoltageSource")->value();
        writeFile(LittleFS, "/BatteryVoltageSource.txt", inputMessage.c_str());
        BatteryVoltageSource = inputMessage.toInt();
      } else if (request->hasParam("AmpControlByRPM")) {
        inputMessage = request->getParam("AmpControlByRPM")->value();
        writeFile(LittleFS, "/AmpControlByRPM.txt", inputMessage.c_str());
        AmpControlByRPM = inputMessage.toInt();
      } else if (request->hasParam("ShuntResistanceMicroOhm")) {
        inputMessage = request->getParam("ShuntResistanceMicroOhm")->value();
        writeFile(LittleFS, "/ShuntResistanceMicroOhm.txt", inputMessage.c_str());
        ShuntResistanceMicroOhm = inputMessage.toInt();
      } else if (request->hasParam("maxPoints")) {
        inputMessage = request->getParam("maxPoints")->value();
        writeFile(LittleFS, "/maxPoints.txt", inputMessage.c_str());
        maxPoints = inputMessage.toInt();
      }
      // Handle RPM/AMPS table - use separate if statements, not else if
      if (request->hasParam("RPM1")) {
        inputMessage = request->getParam("RPM1")->value();
        writeFile(LittleFS, "/RPM1.txt", inputMessage.c_str());
        RPM1 = inputMessage.toInt();
      }
      if (request->hasParam("RPM2")) {
        inputMessage = request->getParam("RPM2")->value();
        writeFile(LittleFS, "/RPM2.txt", inputMessage.c_str());
        RPM2 = inputMessage.toInt();
      }
      if (request->hasParam("RPM3")) {
        inputMessage = request->getParam("RPM3")->value();
        writeFile(LittleFS, "/RPM3.txt", inputMessage.c_str());
        RPM3 = inputMessage.toInt();
      }
      if (request->hasParam("RPM4")) {
        inputMessage = request->getParam("RPM4")->value();
        writeFile(LittleFS, "/RPM4.txt", inputMessage.c_str());
        RPM4 = inputMessage.toInt();
      }
      if (request->hasParam("RPM5")) {
        inputMessage = request->getParam("RPM5")->value();
        writeFile(LittleFS, "/RPM5.txt", inputMessage.c_str());
        RPM5 = inputMessage.toInt();
      }
      if (request->hasParam("RPM6")) {
        inputMessage = request->getParam("RPM6")->value();
        writeFile(LittleFS, "/RPM6.txt", inputMessage.c_str());
        RPM6 = inputMessage.toInt();
      }
      if (request->hasParam("RPM7")) {
        inputMessage = request->getParam("RPM7")->value();
        writeFile(LittleFS, "/RPM7.txt", inputMessage.c_str());
        RPM7 = inputMessage.toInt();
      }
      if (request->hasParam("Amps1")) {
        inputMessage = request->getParam("Amps1")->value();
        writeFile(LittleFS, "/Amps1.txt", inputMessage.c_str());
        Amps1 = inputMessage.toInt();
      }
      if (request->hasParam("Amps2")) {
        inputMessage = request->getParam("Amps2")->value();
        writeFile(LittleFS, "/Amps2.txt", inputMessage.c_str());
        Amps2 = inputMessage.toInt();
      }
      if (request->hasParam("Amps3")) {
        inputMessage = request->getParam("Amps3")->value();
        writeFile(LittleFS, "/Amps3.txt", inputMessage.c_str());
        Amps3 = inputMessage.toInt();
      }
      if (request->hasParam("Amps4")) {
        inputMessage = request->getParam("Amps4")->value();
        writeFile(LittleFS, "/Amps4.txt", inputMessage.c_str());
        Amps4 = inputMessage.toInt();
      }
      if (request->hasParam("Amps5")) {
        inputMessage = request->getParam("Amps5")->value();
        writeFile(LittleFS, "/Amps5.txt", inputMessage.c_str());
        Amps5 = inputMessage.toInt();
      }
      if (request->hasParam("Amps6")) {
        inputMessage = request->getParam("Amps6")->value();
        writeFile(LittleFS, "/Amps6.txt", inputMessage.c_str());
        Amps6 = inputMessage.toInt();
      }
      if (request->hasParam("Amps7")) {
        inputMessage = request->getParam("Amps7")->value();
        writeFile(LittleFS, "/Amps7.txt", inputMessage.c_str());
        Amps7 = inputMessage.toInt();
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

  // Serial.println(F("\n===== TASK STACK REMAINING (BYTES) ====="));
  // Serial.println(F("Task Name        | Core | Stack Remaining | Alert"));

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


    //   Serial.printf("%-16s |  %-3s  |     %5d B     | %s\n",
    //                 taskName,
    //                 coreIdBuffer,
    //                 stackBytes,
    //                 alert);
  }

  // Serial.println(F("==========================================\n"));
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

  // Serial.println(F("========== HEAP STATS =========="));
  // Serial.printf("Free Heap:               %5u KB\n", FreeHeap);
  // Serial.printf("Minimum Ever Free Heap:  %5u KB\n", MinFreeHeap);
  // Serial.printf("Free Internal RAM:       %5u KB\n", FreeInternalRam);
  // Serial.printf("Heap Fragmentation:      %5u %%\n", Heapfrag);
  // Serial.println(F("================================"));
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
  // Serial.printf("CPU Load: Core 0 = %3d%%, Core 1 = %3d%%\n", cpuLoadCore0, cpuLoadCore1);
}

void testTaskStats() {
  char statsBuffer[1024];  // Enough for 15–20 tasks

  vTaskGetRunTimeStats(statsBuffer);

  // Serial.println(F("========== TASK CPU USAGE =========="));
  // Serial.println(statsBuffer);
  // Serial.println(F("====================================\n"));
}

void UpdateBatterySOC(unsigned long elapsedMillis) {
  // Convert elapsed milliseconds to seconds for calculations
  float elapsedSeconds = elapsedMillis / 1000.0f;

  // Update scaled values
  float currentBatteryVoltage = getBatteryVoltage();
  Voltage_scaled = currentBatteryVoltage * 100;
  AlternatorCurrent_scaled = MeasuredAmps * 100;
  BatteryPower_scaled = (Voltage_scaled * BatteryCurrent_scaled) / 100;  // W × 100

  // FIXED: Energy calculation using proper floating point math, then convert to integer
  float batteryPower_W = BatteryPower_scaled / 100.0f;
  float energyDelta_Wh = (batteryPower_W * elapsedSeconds) / 3600.0f;

  float alternatorPower_W = (currentBatteryVoltage * MeasuredAmps);
  float altEnergyDelta_Wh = (alternatorPower_W * elapsedSeconds) / 3600.0f;

  // Calculate fuel used (convert Wh to mL for integer storage)
  if (altEnergyDelta_Wh > 0) {
    // 1. Convert watt-hours to joules (1 Wh = 3600 J)
    float energyJoules = altEnergyDelta_Wh * 3600.0f;
    // 2. Assume engine thermal efficiency is 30%
    const float engineEfficiency = 0.30f;
    // 3. Assume alternator mechanical-to-electrical efficiency is 50%
    const float alternatorEfficiency = 0.50f;
    // 4. Total system efficiency = engine × alternator = 0.30 × 0.50 = 0.15
    float fuelEnergyUsed_J = energyJoules / (engineEfficiency * alternatorEfficiency);
    // 5. Diesel energy content ≈ 36,000 J per mL
    const float dieselEnergy_J_per_mL = 36000.0f;
    // 6. Convert fuel energy to actual mL of diesel burned
    float fuelUsed_mL = fuelEnergyUsed_J / dieselEnergy_J_per_mL;
    // 7. Use accumulator to prevent losing small values
    static float fuelAccumulator = 0.0f;
    fuelAccumulator += fuelUsed_mL;
    if (fuelAccumulator >= 1.0f) {
      AlternatorFuelUsed += (int)fuelAccumulator;
      fuelAccumulator -= (int)fuelAccumulator;
    }
    // Debug output every 10 seconds to see what's happening
    // static unsigned long lastFuelDebug = 0;
    // if (millis() - lastFuelDebug > 10000) {
    //  queueConsoleMessage("Fuel: " + String(fuelUsed_mL, 4) + "mL this cycle, " + String(fuelAccumulator, 4) + "mL accumulated, " + String(AlternatorFuelUsed) + "mL total");
    //  lastFuelDebug = millis();
    // }
  }

  // Energy accumulation - use proper rounding to preserve precision
  static float chargedEnergyAccumulator = 0.0f;
  static float dischargedEnergyAccumulator = 0.0f;
  static float alternatorEnergyAccumulator = 0.0f;

  if (BatteryCurrent_scaled > 0) {
    // Charging - energy going into battery
    chargedEnergyAccumulator += energyDelta_Wh;
    if (chargedEnergyAccumulator >= 1.0f) {
      ChargedEnergy += (int)chargedEnergyAccumulator;
      chargedEnergyAccumulator -= (int)chargedEnergyAccumulator;
    }
  } else if (BatteryCurrent_scaled < 0) {
    // Discharging - energy coming out of battery
    dischargedEnergyAccumulator += abs(energyDelta_Wh);
    if (dischargedEnergyAccumulator >= 1.0f) {
      DischargedEnergy += (int)dischargedEnergyAccumulator;
      dischargedEnergyAccumulator -= (int)dischargedEnergyAccumulator;
    }
  }

  // For alternator energy:
  if (altEnergyDelta_Wh > 0) {
    alternatorEnergyAccumulator += altEnergyDelta_Wh;
    if (alternatorEnergyAccumulator >= 1.0f) {
      AlternatorChargedEnergy += (int)alternatorEnergyAccumulator;
      alternatorEnergyAccumulator -= (int)alternatorEnergyAccumulator;
    }
  }

  alternatorIsOn = (AlternatorCurrent_scaled > CurrentThreshold_scaled);

  if (alternatorIsOn) {
    alternatorOnAccumulator += elapsedMillis;
    if (alternatorOnAccumulator >= 60000) {
      AlternatorOnTime += alternatorOnAccumulator / 60000;
      alternatorOnAccumulator %= 60000;
    }
  }

  alternatorWasOn = alternatorIsOn;

  // FIXED: Use floating point math for Ah calculations, then accumulate properly
  static float coulombAccumulator_Ah = 0.0f;

  // Calculate actual Ah change using floating point
  float batteryCurrent_A = BatteryCurrent_scaled / 100.0f;
  float deltaAh = (batteryCurrent_A * elapsedSeconds) / 3600.0f;

  if (BatteryCurrent_scaled >= 0) {
    // Apply charge efficiency
    float batteryDeltaAh = deltaAh * (ChargeEfficiency_scaled / 100.0f);
    coulombAccumulator_Ah += batteryDeltaAh;
  } else {
    // Apply Peukert compensation
    float peukertFactor = PeukertExponent_scaled / 100.0f;
    float batteryDeltaAh = deltaAh / peukertFactor;
    coulombAccumulator_Ah += batteryDeltaAh;
  }

  // Update the scaled coulomb count when we have accumulated enough change
  if (abs(coulombAccumulator_Ah) >= 0.01f) {
    int deltaAh_scaled = (int)(coulombAccumulator_Ah * 100.0f);
    CoulombCount_Ah_scaled += deltaAh_scaled;
    coulombAccumulator_Ah -= (deltaAh_scaled / 100.0f);
  }

  // Constrain and calculate SoC with decimal precision
  CoulombCount_Ah_scaled = constrain(CoulombCount_Ah_scaled, 0, BatteryCapacity_Ah * 100);
  float SoC_float = (float)CoulombCount_Ah_scaled / (BatteryCapacity_Ah * 100.0f) * 100.0f;
  SoC_percent = (int)(SoC_float * 100);  // Store as percentage × 100 for 2 decimal places

  // --- Full Charge Detection ---
  if ((abs(BatteryCurrent_scaled) <= (TailCurrent_scaled * BatteryCapacity_Ah)) && (Voltage_scaled >= ChargedVoltage_scaled)) {
    FullChargeTimer += elapsedSeconds;
    if (FullChargeTimer >= ChargedDetectionTime) {
      SoC_percent = 10000;  // 100.00% (scaled by 100)
      CoulombCount_Ah_scaled = BatteryCapacity_Ah * 100;
      FullChargeDetected = true;
      coulombAccumulator_Ah = 0.0f;
    }
  } else {
    FullChargeTimer = 0;
    FullChargeDetected = false;
  }
  // Debug message with properly scaled values
  //queueConsoleMessage("SoC Calc - A:" + String(batteryCurrent_A, 2) + " DeltaAh:" + String(deltaAh, 4) + " Accum:" + String(coulombAccumulator_Ah, 4) + " SoC:" + String(SoC_percent / 100.0f, 2) + "% Count:" + String(CoulombCount_Ah_scaled));
}

void UpdateEngineRuntime(unsigned long elapsedMillis) {
  // Check if engine is running (RPM > 100)
  bool engineIsRunning = (RPM > 100 && RPM < 6000);

  if (engineIsRunning) {
    // Accumulate running time in milliseconds
    engineRunAccumulator += elapsedMillis;

    // Update total engine run time every second
    if (engineRunAccumulator >= 1000) {  // 1 second in milliseconds
      int secondsRun = engineRunAccumulator / 1000;
      EngineRunTime += secondsRun;

      // Update engine cycles (RPM * seconds / 60)
      EngineCycles += (RPM * secondsRun) / 60;

      // Keep the remainder milliseconds
      engineRunAccumulator %= 1000;
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
  writeFile(LittleFS, "/MaxTemperatureThermistor.txt", String(MaxTemperatureThermistor).c_str());
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
  if (!LittleFS.exists("/ManualDuty.txt")) {
    writeFile(LittleFS, "/ManualDuty.txt", String(ManualDutyTarget).c_str());
  } else {
    ManualDutyTarget = readFile(LittleFS, "/ManualDuty.txt").toInt();  //
  }
  if (!LittleFS.exists("/FullChargeVoltage.txt")) {
    writeFile(LittleFS, "/FullChargeVoltage.txt", String(ChargingVoltageTarget).c_str());
  } else {
    ChargingVoltageTarget = readFile(LittleFS, "/FullChargeVoltage.txt").toFloat();
  }
  if (!LittleFS.exists("/TargetAmps.txt")) {
    writeFile(LittleFS, "/TargetAmps.txt", String(TargetAmps).c_str());
  } else {
    TargetAmps = readFile(LittleFS, "/TargetAmps.txt").toInt();
  }
  if (!LittleFS.exists("/SwitchingFrequency.txt")) {
    writeFile(LittleFS, "/SwitchingFrequency.txt", String(fffr).c_str());
  } else {
    fffr = readFile(LittleFS, "/SwitchingFrequency.txt").toInt();
  }
  if (!LittleFS.exists("/TargetFloatVoltage.txt")) {
    writeFile(LittleFS, "/TargetFloatVoltage.txt", String(TargetFloatVoltage).c_str());
  } else {
    TargetFloatVoltage = readFile(LittleFS, "/TargetFloatVoltage.txt").toFloat();
  }
  if (!LittleFS.exists("/interval.txt")) {
    writeFile(LittleFS, "/interval.txt", String(interval).c_str());
  } else {
    interval = readFile(LittleFS, "/interval.txt").toFloat();
  }
  if (!LittleFS.exists("/FieldAdjustmentinterval.txt")) {
    writeFile(LittleFS, "/FieldAdjustmentinterval.txt", String(FieldAdjustmentInterval).c_str());
  } else {
    FieldAdjustmentInterval = readFile(LittleFS, "/FieldAdjustmentinterval.txt").toFloat();
  }
  if (!LittleFS.exists("/ManualFieldToggle.txt")) {
    writeFile(LittleFS, "/ManualFieldToggle.txt", String(ManualFieldToggle).c_str());
  } else {
    ManualFieldToggle = readFile(LittleFS, "/ManualFieldToggle.txt").toInt();
  }
  if (!LittleFS.exists("/SwitchControlOverride.txt")) {
    writeFile(LittleFS, "/SwitchControlOverride.txt", String(SwitchControlOverride).c_str());
  } else {
    SwitchControlOverride = readFile(LittleFS, "/SwitchControlOverride.txt").toInt();
  }
  if (!LittleFS.exists("/IgnitionOverride.txt")) {
    writeFile(LittleFS, "/IgnitionOverride.txt", String(IgnitionOverride).c_str());
  } else {
    IgnitionOverride = readFile(LittleFS, "/IgnitionOverride.txt").toInt();
  }
  if (!LittleFS.exists("/ForceFloat1.txt")) {
    writeFile(LittleFS, "/ForceFloat1.txt", String(ForceFloat).c_str());
  } else {
    ForceFloat = readFile(LittleFS, "/ForceFloat1.txt").toInt();
  }
  if (!LittleFS.exists("/OnOff.txt")) {
    writeFile(LittleFS, "/OnOff.txt", String(OnOff).c_str());
  } else {
    OnOff = readFile(LittleFS, "/OnOff.txt").toInt();
  }
  if (!LittleFS.exists("/HiLow.txt")) {
    writeFile(LittleFS, "/HiLow.txt", String(HiLow).c_str());
  } else {
    HiLow = readFile(LittleFS, "/HiLow.txt").toInt();
  }
  if (!LittleFS.exists("/AmpSrc.txt")) {
    writeFile(LittleFS, "/AmpSrc.txt", String(AmpSrc).c_str());
  } else {
    AmpSrc = readFile(LittleFS, "/AmpSrc.txt").toInt();
  }
  if (!LittleFS.exists("/InvertAltAmps.txt")) {
    writeFile(LittleFS, "/InvertAltAmps.txt", String(InvertAltAmps).c_str());
  } else {
    InvertAltAmps = readFile(LittleFS, "/InvertAltAmps.txt").toInt();
  }
  if (!LittleFS.exists("/InvertBattAmps.txt")) {
    writeFile(LittleFS, "/InvertBattAmps.txt", String(InvertBattAmps).c_str());
  } else {
    InvertBattAmps = readFile(LittleFS, "/InvertBattAmps.txt").toInt();
  }
  if (!LittleFS.exists("/LimpHome.txt")) {
    writeFile(LittleFS, "/LimpHome.txt", String(LimpHome).c_str());
  } else {
    LimpHome = readFile(LittleFS, "/LimpHome.txt").toInt();
  }
  if (!LittleFS.exists("/VeData.txt")) {
    writeFile(LittleFS, "/VeData.txt", String(VeData).c_str());
  } else {
    VeData = readFile(LittleFS, "/VeData.txt").toInt();
  }
  if (!LittleFS.exists("/NMEA0183Data.txt")) {
    writeFile(LittleFS, "/NMEA0183Data.txt", String(NMEA0183Data).c_str());
  } else {
    NMEA0183Data = readFile(LittleFS, "/NMEA0183Data.txt").toInt();
  }
  if (!LittleFS.exists("/NMEA2KData.txt")) {
    writeFile(LittleFS, "/NMEA2KData.txt", String(NMEA2KData).c_str());
  } else {
    NMEA2KData = readFile(LittleFS, "/NMEA2KData.txt").toInt();
  }
  if (!LittleFS.exists("/TargetAmpL.txt")) {
    writeFile(LittleFS, "/TargetAmpL.txt", String(TargetAmpLA).c_str());
  } else {
    TargetAmpLA = readFile(LittleFS, "/TargetAmpL.txt").toInt();
  }

  //New May 17
  if (!LittleFS.exists("/CurrentThreshold.txt")) {
    writeFile(LittleFS, "/CurrentThreshold.txt", String(CurrentThreshold_scaled).c_str());
  } else {
    CurrentThreshold_scaled = readFile(LittleFS, "/CurrentThreshold.txt").toInt();
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
  if (!LittleFS.exists("/ChargedDetectionTime.txt")) {
    writeFile(LittleFS, "/ChargedDetectionTime.txt", String(ChargedDetectionTime).c_str());
  } else {
    ChargedDetectionTime = readFile(LittleFS, "/ChargedDetectionTime.txt").toInt();
  }
  if (!LittleFS.exists("/IgnoreTemperature.txt")) {
    writeFile(LittleFS, "/IgnoreTemperature.txt", String(IgnoreTemperature).c_str());
  } else {
    IgnoreTemperature = readFile(LittleFS, "/IgnoreTemperature.txt").toInt();
  }
  if (!LittleFS.exists("/BMSLogic.txt")) {
    writeFile(LittleFS, "/BMSLogic.txt", String(BMSlogic).c_str());
  } else {
    BMSlogic = readFile(LittleFS, "/BMSLogic.txt").toInt();
  }
  if (!LittleFS.exists("/BMSLogicLevelOff.txt")) {
    writeFile(LittleFS, "/BMSLogicLevelOff.txt", String(BMSLogicLevelOff).c_str());
  } else {
    BMSLogicLevelOff = readFile(LittleFS, "/BMSLogicLevelOff.txt").toInt();
  }
  if (!LittleFS.exists("/AlarmActivate.txt")) {
    writeFile(LittleFS, "/AlarmActivate.txt", String(AlarmActivate).c_str());
  } else {
    AlarmActivate = readFile(LittleFS, "/AlarmActivate.txt").toInt();
  }
  if (!LittleFS.exists("/TempAlarm.txt")) {
    writeFile(LittleFS, "/TempAlarm.txt", String(TempAlarm).c_str());
  } else {
    TempAlarm = readFile(LittleFS, "/TempAlarm.txt").toInt();
  }
  if (!LittleFS.exists("/VoltageAlarmHigh.txt")) {
    writeFile(LittleFS, "/VoltageAlarmHigh.txt", String(VoltageAlarmHigh).c_str());
  } else {
    VoltageAlarmHigh = readFile(LittleFS, "/VoltageAlarmHigh.txt").toInt();
  }
  if (!LittleFS.exists("/VoltageAlarmLow.txt")) {
    writeFile(LittleFS, "/VoltageAlarmLow.txt", String(VoltageAlarmLow).c_str());
  } else {
    VoltageAlarmLow = readFile(LittleFS, "/VoltageAlarmLow.txt").toInt();
  }
  if (!LittleFS.exists("/CurrentAlarmHigh.txt")) {
    writeFile(LittleFS, "/CurrentAlarmHigh.txt", String(CurrentAlarmHigh).c_str());
  } else {
    CurrentAlarmHigh = readFile(LittleFS, "/CurrentAlarmHigh.txt").toInt();
  }
  if (!LittleFS.exists("/FourWay.txt")) {
    writeFile(LittleFS, "/FourWay.txt", String(FourWay).c_str());
  } else {
    FourWay = readFile(LittleFS, "/FourWay.txt").toInt();
  }
  if (!LittleFS.exists("/RPMScalingFactor.txt")) {
    writeFile(LittleFS, "/RPMScalingFactor.txt", String(RPMScalingFactor).c_str());
  } else {
    RPMScalingFactor = readFile(LittleFS, "/RPMScalingFactor.txt").toInt();
  }

  if (!LittleFS.exists("/FieldResistance.txt")) {
    writeFile(LittleFS, "/FieldResistance.txt", String(FieldResistance).c_str());
  } else {
    FieldResistance = readFile(LittleFS, "/FieldResistance.txt").toInt();
  }


  if (!LittleFS.exists("/ResetTemp.txt")) {
    writeFile(LittleFS, "/ResetTemp.txt", String(ResetTemp).c_str());
  } else {
    ResetTemp = readFile(LittleFS, "/ResetTemp.txt").toInt();
  }
  if (!LittleFS.exists("/ResetVoltage.txt")) {
    writeFile(LittleFS, "/ResetVoltage.txt", String(ResetVoltage).c_str());
  } else {
    ResetVoltage = readFile(LittleFS, "/ResetVoltage.txt").toInt();
  }
  if (!LittleFS.exists("/ResetCurrent.txt")) {
    writeFile(LittleFS, "/ResetCurrent.txt", String(ResetCurrent).c_str());
  } else {
    ResetCurrent = readFile(LittleFS, "/ResetCurrent.txt").toInt();
  }
  if (!LittleFS.exists("/ResetEngineRunTime.txt")) {
    writeFile(LittleFS, "/ResetEngineRunTime.txt", String(ResetEngineRunTime).c_str());
  } else {
    ResetEngineRunTime = readFile(LittleFS, "/ResetEngineRunTime.txt").toInt();
  }
  if (!LittleFS.exists("/ResetAlternatorOnTime.txt")) {
    writeFile(LittleFS, "/ResetAlternatorOnTime.txt", String(ResetAlternatorOnTime).c_str());
  } else {
    ResetAlternatorOnTime = readFile(LittleFS, "/ResetAlternatorOnTime.txt").toInt();
  }
  if (!LittleFS.exists("/ResetEnergy.txt")) {
    writeFile(LittleFS, "/ResetEnergy.txt", String(ResetEnergy).c_str());
  } else {
    ResetEnergy = readFile(LittleFS, "/ResetEnergy.txt").toInt();
  }

  if (!LittleFS.exists("/MaximumAllowedBatteryAmps.txt")) {
    writeFile(LittleFS, "/MaximumAllowedBatteryAmps.txt", String(MaximumAllowedBatteryAmps).c_str());
  } else {
    MaximumAllowedBatteryAmps = readFile(LittleFS, "/MaximumAllowedBatteryAmps.txt").toInt();
  }

  if (!LittleFS.exists("/ManualSOCPoint.txt")) {
    writeFile(LittleFS, "/ManualSOCPoint.txt", String(ManualSOCPoint).c_str());
  } else {
    ManualSOCPoint = readFile(LittleFS, "/ManualSOCPoint.txt").toInt();
  }

  if (!LittleFS.exists("/BatteryVoltageSource.txt")) {
    writeFile(LittleFS, "/BatteryVoltageSource.txt", String(BatteryVoltageSource).c_str());
  } else {
    BatteryVoltageSource = readFile(LittleFS, "/BatteryVoltageSource.txt").toInt();
  }

  if (!LittleFS.exists("/AmpControlByRPM.txt")) {
    writeFile(LittleFS, "/AmpControlByRPM.txt", String(AmpControlByRPM).c_str());
  } else {
    AmpControlByRPM = readFile(LittleFS, "/AmpControlByRPM.txt").toInt();
  }
  if (!LittleFS.exists("/RPM1.txt")) {
    writeFile(LittleFS, "/RPM1.txt", String(RPM1).c_str());
  } else {
    RPM1 = readFile(LittleFS, "/RPM1.txt").toInt();
  }

  if (!LittleFS.exists("/RPM2.txt")) {
    writeFile(LittleFS, "/RPM2.txt", String(RPM2).c_str());
  } else {
    RPM2 = readFile(LittleFS, "/RPM2.txt").toInt();
  }

  if (!LittleFS.exists("/RPM3.txt")) {
    writeFile(LittleFS, "/RPM3.txt", String(RPM3).c_str());
  } else {
    RPM3 = readFile(LittleFS, "/RPM3.txt").toInt();
  }

  if (!LittleFS.exists("/RPM4.txt")) {
    writeFile(LittleFS, "/RPM4.txt", String(RPM4).c_str());
  } else {
    RPM4 = readFile(LittleFS, "/RPM4.txt").toInt();
  }

  if (!LittleFS.exists("/RPM5.txt")) {
    writeFile(LittleFS, "/RPM5.txt", String(RPM5).c_str());
  } else {
    RPM5 = readFile(LittleFS, "/RPM5.txt").toInt();
  }

  if (!LittleFS.exists("/RPM6.txt")) {
    writeFile(LittleFS, "/RPM6.txt", String(RPM6).c_str());
  } else {
    RPM6 = readFile(LittleFS, "/RPM6.txt").toInt();
  }

  if (!LittleFS.exists("/RPM7.txt")) {
    writeFile(LittleFS, "/RPM7.txt", String(RPM7).c_str());
  } else {
    RPM7 = readFile(LittleFS, "/RPM7.txt").toInt();
  }

  if (!LittleFS.exists("/Amps1.txt")) {
    writeFile(LittleFS, "/Amps1.txt", String(Amps1).c_str());
  } else {
    Amps1 = readFile(LittleFS, "/Amps1.txt").toInt();
  }

  if (!LittleFS.exists("/Amps2.txt")) {
    writeFile(LittleFS, "/Amps2.txt", String(Amps2).c_str());
  } else {
    Amps2 = readFile(LittleFS, "/Amps2.txt").toInt();
  }

  if (!LittleFS.exists("/Amps3.txt")) {
    writeFile(LittleFS, "/Amps3.txt", String(Amps3).c_str());
  } else {
    Amps3 = readFile(LittleFS, "/Amps3.txt").toInt();
  }

  if (!LittleFS.exists("/Amps4.txt")) {
    writeFile(LittleFS, "/Amps4.txt", String(Amps4).c_str());
  } else {
    Amps4 = readFile(LittleFS, "/Amps4.txt").toInt();
  }

  if (!LittleFS.exists("/Amps5.txt")) {
    writeFile(LittleFS, "/Amps5.txt", String(Amps5).c_str());
  } else {
    Amps5 = readFile(LittleFS, "/Amps5.txt").toInt();
  }

  if (!LittleFS.exists("/Amps6.txt")) {
    writeFile(LittleFS, "/Amps6.txt", String(Amps6).c_str());
  } else {
    Amps6 = readFile(LittleFS, "/Amps6.txt").toInt();
  }
  if (!LittleFS.exists("/Amps7.txt")) {
    writeFile(LittleFS, "/Amps7.txt", String(Amps7).c_str());
  } else {
    Amps7 = readFile(LittleFS, "/Amps7.txt").toInt();
  }
  if (!LittleFS.exists("/ShuntResistanceMicroOhm.txt")) {
    writeFile(LittleFS, "/ShuntResistanceMicroOhm.txt", String(ShuntResistanceMicroOhm).c_str());
  } else {
    ShuntResistanceMicroOhm = readFile(LittleFS, "/ShuntResistanceMicroOhm.txt").toInt();
  }
  if (!LittleFS.exists("/maxPoints.txt")) {
    writeFile(LittleFS, "/maxPoints.txt", String(maxPoints).c_str());
  } else {
    maxPoints = readFile(LittleFS, "/maxPoints.txt").toInt();
  }
  if (!LittleFS.exists("/MaxDuty.txt")) {
    writeFile(LittleFS, "/MaxDuty.txt", String(MaxDuty).c_str());
  } else {
    MaxDuty = readFile(LittleFS, "/MaxDuty.txt").toInt();
  }
  if (!LittleFS.exists("/MinDuty.txt")) {
    writeFile(LittleFS, "/MinDuty.txt", String(MinDuty).c_str());
  } else {
    MinDuty = readFile(LittleFS, "/MinDuty.txt").toInt();
  }
  if (!LittleFS.exists("/R_fixed.txt")) {
    writeFile(LittleFS, "/R_fixed.txt", String(R_fixed).c_str());
  } else {
    R_fixed = readFile(LittleFS, "/R_fixed.txt").toFloat();
  }
  if (!LittleFS.exists("/Beta.txt")) {
    writeFile(LittleFS, "/Beta.txt", String(Beta).c_str());
  } else {
    Beta = readFile(LittleFS, "/Beta.txt").toFloat();
  }
  if (!LittleFS.exists("/T0_C.txt")) {
    writeFile(LittleFS, "/T0_C.txt", String(T0_C).c_str());
  } else {
    T0_C = readFile(LittleFS, "/T0_C.txt").toFloat();
  }
  if (!LittleFS.exists("/TempSource.txt")) {
    writeFile(LittleFS, "/TempSource.txt", String(TempSource).c_str());
  } else {
    TempSource = readFile(LittleFS, "/TempSource.txt").toInt();
  }
  if (!LittleFS.exists("/AlternatorCOffset.txt")) {
    writeFile(LittleFS, "/AlternatorCOffset.txt", String(AlternatorCOffset).c_str());
  } else {
    AlternatorCOffset = readFile(LittleFS, "/AlternatorCOffset.txt").toInt();
  }

  if (!LittleFS.exists("/BatteryCOffset.txt")) {
    writeFile(LittleFS, "/BatteryCOffset.txt", String(BatteryCOffset).c_str());
  } else {
    BatteryCOffset = readFile(LittleFS, "/BatteryCOffset.txt").toInt();
  }

  if (!LittleFS.exists("/MaxTemperatureThermistor.txt")) {
    writeFile(LittleFS, "/MaxTemperatureThermistor.txt", String(MaxTemperatureThermistor).c_str());
  } else {
    MaxTemperatureThermistor = readFile(LittleFS, "/MaxTemperatureThermistor.txt").toInt();  // Use toInt()
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
    ChargedEnergy = readFile(LittleFS, "/ChargedEnergy.txt").toInt();  // Back to toInt()
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
  // reset buttons
  if (!LittleFS.exists("/MaxAlternatorTemperatureF.txt")) {
    writeFile(LittleFS, "/MaxAlternatorTemperatureF.txt", "0");
  } else {
    MaxAlternatorTemperatureF = readFile(LittleFS, "/MaxAlternatorTemperatureF.txt").toInt();
  }

  if (!LittleFS.exists("/IBVMax.txt")) {
    writeFile(LittleFS, "/IBVMax.txt", "0");
  } else {
    IBVMax = readFile(LittleFS, "/IBVMax.txt").toInt();
  }

  if (!LittleFS.exists("/MeasuredAmpsMax.txt")) {
    writeFile(LittleFS, "/MeasuredAmpsMax.txt", "0");
  } else {
    MeasuredAmpsMax = readFile(LittleFS, "/MeasuredAmpsMax.txt").toInt();
  }

  if (!LittleFS.exists("/EngineRunTime.txt")) {
    writeFile(LittleFS, "/EngineRunTime.txt", "0");
  } else {
    EngineRunTime = readFile(LittleFS, "/EngineRunTime.txt").toInt();
  }

  if (!LittleFS.exists("/AlternatorOnTime.txt")) {
    writeFile(LittleFS, "/AlternatorOnTime.txt", "0");
  } else {
    AlternatorOnTime = readFile(LittleFS, "/AlternatorOnTime.txt").toInt();
  }

  if (!LittleFS.exists("/ChargedEnergy.txt")) {
    writeFile(LittleFS, "/ChargedEnergy.txt", "0");
  } else {
    ChargedEnergy = readFile(LittleFS, "/ChargedEnergy.txt").toInt();
  }

  if (!LittleFS.exists("/DischargedEnergy.txt")) {
    writeFile(LittleFS, "/DischargedEnergy.txt", "0");
  } else {
    DischargedEnergy = readFile(LittleFS, "/DischargedEnergy.txt").toInt();
  }

  if (!LittleFS.exists("/AlternatorFuelUsed.txt")) {
    writeFile(LittleFS, "/AlternatorFuelUsed.txt", "0");
  } else {
    AlternatorFuelUsed = readFile(LittleFS, "/AlternatorFuelUsed.txt").toInt();
  }

  if (!LittleFS.exists("/AlternatorChargedEnergy.txt")) {
    writeFile(LittleFS, "/AlternatorChargedEnergy.txt", "0");
  } else {
    AlternatorChargedEnergy = readFile(LittleFS, "/AlternatorChargedEnergy.txt").toInt();
  }
}

int interpolateAmpsFromRPM(float currentRPM) {
  // Create arrays from the individual global variables for easier processing
  // These represent the user-configured RPM/Amps curve points from the web interface
  int rpmPoints[] = { RPM1, RPM2, RPM3, RPM4, RPM5, RPM6, RPM7 };
  int ampPoints[] = { Amps1, Amps2, Amps3, Amps4, Amps5, Amps6, Amps7 };

  // Find how many valid entries exist in the table (non-zero RPM values)
  // Users may not fill all 7 points, so we need to know where the table ends
  int validPoints = 0;
  for (int i = 0; i < 7; i++) {
    if (rpmPoints[i] > 0) {
      validPoints = i + 1;  // Count consecutive valid entries from start
    }
  }

  // Safety check: If no valid points configured, fall back to normal target amps
  if (validPoints == 0) {
    return TargetAmps;  // Use the standard charging amperage setting
  }

  // Handle edge cases: RPM below or above the configured table range

  // If current RPM is at or below the lowest configured point
  if (currentRPM <= rpmPoints[0]) {
    return ampPoints[0];  // Use the lowest configured amperage
  }

  // If current RPM is at or above the highest configured point
  if (currentRPM >= rpmPoints[validPoints - 1]) {
    return ampPoints[validPoints - 1];  // Use the highest configured amperage
  }

  // Linear interpolation between adjacent points in the table
  // Find which two points the current RPM falls between
  for (int i = 0; i < validPoints - 1; i++) {
    // Check if current RPM is between point i and point i+1
    if (currentRPM >= rpmPoints[i] && currentRPM <= rpmPoints[i + 1]) {
      // Calculate interpolation ratio (0.0 = at first point, 1.0 = at second point)
      float ratio = (currentRPM - rpmPoints[i]) / (float)(rpmPoints[i + 1] - rpmPoints[i]);

      // Linear interpolation formula: startValue + ratio * (endValue - startValue)
      // This gives smooth transitions between configured points
      return ampPoints[i] + (ratio * (ampPoints[i + 1] - ampPoints[i]));
    }
  }

  // Fallback safety net (should never reach here with valid data)
  // If somehow we get here, use the standard target amps
  return TargetAmps;
}

void queueConsoleMessage(String message) {
  consoleMessageQueue.push_back(message);
  Serial.println("Queued: " + message);  // For debugging via serial monitor
}

void processConsoleQueue() {
  unsigned long now11 = millis();

  // Check if it's time to send messages and we have messages to send
  if (now11 - lastConsoleMessageTime >= CONSOLE_MESSAGE_INTERVAL && !consoleMessageQueue.empty()) {
    // Send up to 3 messages per interval
    int messagesToSend = min(3, (int)consoleMessageQueue.size());

    for (int i = 0; i < messagesToSend; i++) {
      String message = consoleMessageQueue.front();
      consoleMessageQueue.erase(consoleMessageQueue.begin());
      events.send(message.c_str(), "console");
      Serial.println("Sent to console: " + message);  // For debugging
    }

    lastConsoleMessageTime = now11;
  }
}

float getBatteryVoltage() {
  static unsigned long lastWarningTime = 0;
  static unsigned long lastDebugTime = 0;
  const unsigned long WARNING_INTERVAL = 10000;  // 10 seconds between warnings
  const unsigned long DEBUG_INTERVAL = 5000;     // 5 seconds between debug messages

  float selectedVoltage = 0;

  // Debug current state every 5 seconds
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    queueConsoleMessage("Voltage Debug - Source:" + String(BatteryVoltageSource) + " INA:" + String(IBV, 2) + "V ADS:" + String(BatteryV, 2) + "V Victron:" + String(VictronVoltage, 2) + "V");
    lastDebugTime = millis();
  }

  switch (BatteryVoltageSource) {
    case 0:  // INA228
      if (INADisconnected == 0 && !isnan(IBV) && IBV > 8.0 && IBV < 70.0) {
        selectedVoltage = IBV;
      } else {
        if (millis() - lastWarningTime > WARNING_INTERVAL) {
          queueConsoleMessage("WARNING: INA228 unavailable (disconnected:" + String(INADisconnected) + " value:" + String(IBV, 2) + "), falling back to ADS1115");
          lastWarningTime = millis();
        }
        selectedVoltage = BatteryV;
      }
      break;

    case 1:  // ADS1115
      if (ADS1115Disconnected == 0 && !isnan(BatteryV) && BatteryV > 8.0 && BatteryV < 70.0) {
        selectedVoltage = BatteryV;
      } else {
        if (millis() - lastWarningTime > WARNING_INTERVAL) {
          queueConsoleMessage("WARNING: ADS1115 unavailable (disconnected:" + String(ADS1115Disconnected) + " value:" + String(BatteryV, 2) + "), falling back to INA228");
          lastWarningTime = millis();
        }
        selectedVoltage = IBV;
      }
      break;

    case 2:  // VictronVeDirect
      if (VictronVoltage > 8.0 && VictronVoltage < 70.0) {
        selectedVoltage = VictronVoltage;
      } else {
        if (millis() - lastWarningTime > WARNING_INTERVAL) {
          queueConsoleMessage("WARNING: Invalid Victron voltage (" + String(VictronVoltage, 2) + "V), falling back to INA228");
          lastWarningTime = millis();
        }
        selectedVoltage = IBV;
      }
      break;

    case 3:  // NMEA0183
      if (millis() - lastWarningTime > WARNING_INTERVAL) {
        queueConsoleMessage("NMEA0183 voltage source not implemented, using INA228");
        lastWarningTime = millis();
      }
      selectedVoltage = IBV;
      break;

    case 4:  // NMEA2K
      if (millis() - lastWarningTime > WARNING_INTERVAL) {
        queueConsoleMessage("NMEA2K voltage source not implemented, using INA228");
        lastWarningTime = millis();
      }
      selectedVoltage = IBV;
      break;

    default:
      if (millis() - lastWarningTime > WARNING_INTERVAL) {
        queueConsoleMessage("Invalid battery voltage source (" + String(BatteryVoltageSource) + "), using INA228");
        lastWarningTime = millis();
      }
      selectedVoltage = IBV;
      break;
  }

  // Final validation
  if (selectedVoltage < 8.0 || selectedVoltage > 70.0 || isnan(selectedVoltage)) {
    if (millis() - lastWarningTime > WARNING_INTERVAL) {
      queueConsoleMessage("CRITICAL: No valid battery voltage found! Using 999 default.");
      lastWarningTime = millis();
    }
    selectedVoltage = 999;  // This is bs, but do something later
  }

  // Log successful reads occasionally for debugging
  if (millis() - lastDebugTime > DEBUG_INTERVAL * 2) {  // Every 10 seconds
    queueConsoleMessage("Voltage OK: " + String(selectedVoltage, 2));
  }
  return selectedVoltage;
}

float getTargetAmps() {
  static unsigned long lastDebugTime = 0;
  const unsigned long DEBUG_INTERVAL = 5000;  // 5 seconds between debug messages

  float targetValue = 0;

  // Debug current state every 5 seconds
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    queueConsoleMessage("Amps Debug - Source:" + String(AmpSrc) + " MeasuredAmps:" + String(MeasuredAmps, 2) + " Bcur:" + String(Bcur, 2) + " VictronCurrent:" + String(VictronCurrent, 2));
    lastDebugTime = millis();
  }

  switch (AmpSrc) {
    case 0:  // Alt Hall Effect Sensor
      targetValue = MeasuredAmps;
      break;

    case 1:  // Battery Shunt
      targetValue = Bcur;
      break;

    case 2:  // NMEA2K Batt
      queueConsoleMessage("NMEA2K Battery current not implemented, using Battery Shunt");
      targetValue = Bcur;
      break;

    case 3:  // NMEA2K Alt
      queueConsoleMessage("NMEA2K Alternator current not implemented, using Alt Hall Sensor");
      targetValue = MeasuredAmps;
      break;

    case 4:  // NMEA0183 Batt
      queueConsoleMessage("NMEA0183 Battery current not implemented, using Battery Shunt");
      targetValue = Bcur;
      break;

    case 5:  // NMEA0183 Alt
      queueConsoleMessage("NMEA0183 Alternator current not implemented, using Alt Hall Sensor");
      targetValue = MeasuredAmps;
      break;

    case 6:                             // Victron Batt
      if (abs(VictronCurrent) > 0.1) {  // Valid reading
        targetValue = VictronCurrent;
      } else {
        queueConsoleMessage("Victron current not available, using Battery Shunt");
        targetValue = Bcur;
      }
      break;

    case 7:  // Other
      targetValue = MeasuredAmps;
      break;

    default:
      queueConsoleMessage("Invalid AmpSrc (" + String(AmpSrc) + "), using Alt Hall Sensor");
      targetValue = MeasuredAmps;
      break;
  }

  // Log the result occasionally for debugging
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    queueConsoleMessage("Target Amps: " + String(targetValue, 2) + "A from ");
  }

  return targetValue;
}

int thermistorTempC(float V_thermistor) {
  float Vcc = 5.0;
  float R_thermistor = R_fixed * (V_thermistor / (Vcc - V_thermistor));
  float T0_K = T0_C + 273.15;
  float tempK = 1.0 / ((1.0 / T0_K) + (1.0 / Beta) * log(R_thermistor / R0));
  return (int)(tempK - 273.15);  // Cast to int for whole degrees
}

void StuffToDoAtSomePoint() {
  //every reset button has a pointless flag and an echo.  I did not delete them for fear of breaking the payload and they hardly cost anything to keep
  //Battery Voltage Source drop down menu- make this text update on page re-load instead of just an echo number
  //Ask AI to go through and add many more relevent Console messages- make sure none of them come in too rapidly
  // Is it possible to power up if the igniton is off?  A bunch of setup functions may fail?
  // What happens when rpm table is screwed up by an idiot
}
