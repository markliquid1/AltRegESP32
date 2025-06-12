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


// Helper function to organize hardware initialization
void initializeHardware() {
  Serial.println("Starting hardware initialization...");

  //NMEA2K
  OutputStream = &Serial;
  //   while (!Serial)
  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50); // was commented
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);  // was false
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  Serial.println("NMEA2K Running...");

  //Victron VeDirect
  Serial1.begin(19200, SERIAL_8N1, 25, -1, 0);  // ... note the "0" at end for normal logic.  This is the reading of the combined NMEA0183 data from YachtDevices
  Serial2.begin(19200, SERIAL_8N1, 26, -1, 1);  // This is the reading of Victron VEDirect
  Serial2.flush();

  if (!INA.begin()) {
    Serial.println("Could not connect INA. Fix and Reboot");
    queueConsoleMessage("WARNING: Could not connect INA228 Battery Voltage/Amp measuring chip");
    INADisconnected = 1;
    // while (1)
    ;
  } else {
    INADisconnected = 0;
  }
  // at least 529ms for an update with these settings for average and conversion time
  INA.setMode(11);                       // Bh = Continuous shunt and bus voltage
  INA.setAverage(4);                     //0h = 1, 1h = 4, 2h = 16, 3h = 64, 4h = 128, 5h = 256, 6h = 512, 7h = 1024     Applies to all channels
  INA.setBusVoltageConversionTime(7);    // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs
  INA.setShuntVoltageConversionTime(7);  // Sets the conversion time of the bus voltage measurement: 0h = 50 µs, 1h = 84 µs, 2h = 150 µs, 3h = 280 µs, 4h = 540 µs, 5h = 1052 µs, 6h = 2074 µs, 7h = 4120 µs

  if (setupDisplay()) {
    Serial.println("Display ready for use");
  } else {
    Serial.println("Continuing without display");
  }

  unsigned long now = millis();
  for (int i = 0; i < MAX_DATA_INDICES; i++) {
    dataTimestamps[i] = now;  // Start with current time
  }

  //ADS1115
  //Connection check
  if (!adc.testConnection()) {
    Serial.println("ADS1115 Connection failed and would have triggered a return if it wasn't commented out");
    queueConsoleMessage("WARNING: ADS1115 Analog Input chip failed");
    ADS1115Disconnected = 1;
    // return;
  } else {
    ADS1115Disconnected = 0;
  }
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

  //onewire
  sensors.begin();
  sensors.setResolution(12);
  sensors.getAddress(tempDeviceAddress, 0);
  if (sensors.getDeviceCount() == 0) {
    Serial.println("WARNING: No DS18B20 sensors found on the bus.");
    queueConsoleMessage("WARNING: No DS18B20 sensors found on the bus");
    sensors.setWaitForConversion(false);  // this is critical!
  }

  xTaskCreatePinnedToCore(
    TempTask,
    "TempTask",
    4096,
    NULL,
    0,  // Priority lower than normal (execute if nothing else to do, all the 1's are idle)
    &tempTaskHandle,
    0  // Run on Core 0, which is the one doing Wifi and system tasks, and theoretically has more idle points than Core 1 and "loop()"
  );

  Serial.println("Hardware initialization complete");
}
void setDutyPercent(int percent) {  // Function to set PWM duty cycle by percentage
  percent = constrain(percent, 0, 100);
  uint32_t duty = (65535UL * percent) / 100;
  ledcWrite(pwmPin, duty);  // In v3.x, first parameter is the pin number
}
// Function 6: AdjustField() - PWM Field Control with Freshness Tracking
void AdjustField() {
  if (millis() - prev_millis22 > FieldAdjustmentInterval) {  // adjust field every FieldAdjustmentInterval milliseconds
    // Update charging stage (bulk/float logic)
    updateChargingStage();
    float currentBatteryVoltage = getBatteryVoltage();
    // Emergency field collapse - voltage spike protection
    if (currentBatteryVoltage > (ChargingVoltageTarget + 0.2) && chargingEnabled) {
      digitalWrite(4, 0);  // Immediately disable field
      dutyCycle = MinDuty;
      setDutyPercent((int)dutyCycle);
      fieldCollapseTime = millis();  // Record when collapse happened
      queueConsoleMessage("EMERGENCY: Field collapsed - voltage spike (" + String(currentBatteryVoltage, 2) + "V) - disabled for 10 seconds");
      return;  // Exit function immediately
    }
    // Check if we're still in collapse delay period
    if (fieldCollapseTime > 0 && (millis() - fieldCollapseTime) < FIELD_COLLAPSE_DELAY) {
      digitalWrite(4, 0);  // Keep field off
      dutyCycle = MinDuty;
      setDutyPercent((int)dutyCycle);
      return;  // Exit function, don't do normal field control
    }
    // Clear the collapse flag after delay expires
    if (fieldCollapseTime > 0 && (millis() - fieldCollapseTime) >= FIELD_COLLAPSE_DELAY) {
      fieldCollapseTime = 0;
      queueConsoleMessage("Field collapse delay expired - normal operation resumed");
    }
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
          uTargetAmps = TargetAmpL;  // Low target
        }
        // Step 2: Apply RPM-based modification if enabled
        if (AmpControlByRPM == 1 && RPM > 100 && RPM < 6000) {
          int rpmBasedAmps = interpolateAmpsFromRPM(RPM);

          // Apply RPM curve but respect Hi/Low setting
          if (HiLow == 0) {
            // In Low mode, use lesser of RPM curve or low setting
            uTargetAmps = min(rpmBasedAmps, (int)(TargetAmpL));
          } else {
            // In Normal mode, use RPM curve directly
            uTargetAmps = rpmBasedAmps;
          }
        }

        // Step 2.4: Apply ForceFloat override if enabled
        if (ForceFloat == 1) {
          // Force float mode: target 0 amps at battery (perfect float charging)
          uTargetAmps = 0;
        }

        // Step 2.5, figure out the actual amps reading of whichever value we are controlling on
        if (ForceFloat == 1) {
          // Force float mode: use battery current (should be ~0)
          targetCurrent = Bcur;
        } else {
          // Normal mode: use configured current source
          targetCurrent = getTargetAmps();
        }
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
        uTargetAmps = -99;  // just useful for debugging, delete later
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

    // Mark calculated values as fresh - these are always current when calculated
    MARK_FRESH(IDX_DUTY_CYCLE);
    MARK_FRESH(IDX_FIELD_VOLTS);
    MARK_FRESH(IDX_FIELD_AMPS);

    // Update timer (only once)
    prev_millis22 = millis();
    /// delete this whole thing later
    static unsigned long lastRunTime2g = 0;
    const unsigned long interval2g = 10000;  // 10 seconds

    if (millis() - lastRunTime2g >= interval2g) {
      lastRunTime2g = millis();

      if (dutyCycle <= (MinDuty + 1.0)) {
        //Serial.println();
        // String msg = " utargetAmps=" + String((float)uTargetAmps, 1)
        //           + " targetCurrent=" + String(targetCurrent, 1)
        //           + " currentBatteryVoltage=" + String(currentBatteryVoltage, 2) + "V"
        //           + " TempToUse=" + String((float)TempToUse, 1) + "F"
        //          + " dutyCycle=" + String(dutyCycle, 1);
        //  queueConsoleMessage(msg);      GREAT DEBUG TOOL ADD BACK IN LATER
      }
    }
    fieldActiveStatus = (
                          // Basic enables
                          (Ignition == 1) && (OnOff == 1) &&

                          // BMS logic (if enabled)
                          (BMSlogic == 0 || (BMSLogicLevelOff == 0 ? bmsSignalActive : !bmsSignalActive)) &&

                          // Not in emergency field collapse
                          (fieldCollapseTime == 0 || (millis() - fieldCollapseTime) >= FIELD_COLLAPSE_DELAY) &&

                          // Duty cycle is meaningfully above minimum
                          (dutyCycle > (MinDuty + 1.0)) &&

                          // Physical field enable pin is actually HIGH
                          (digitalRead(4) == HIGH))
                          ? 1
                          : 0;
  }
}

void ReadAnalogInputs() {
  // INA228 Battery Monitor
  if (millis() - lastINARead >= 900) {  // could go down to 600 here, but this logic belongs in Loop anyway
    if (INADisconnected == 0) {
      int start33 = micros();  // Start timing analog input reading
      lastINARead = millis();

      try {
        IBV = INA.getBusVoltage();
        ShuntVoltage_mV = INA.getShuntVoltage_mV();

        // Sanity check the readings
        if (!isnan(IBV) && IBV > 5.0 && IBV < 70.0 && !isnan(ShuntVoltage_mV)) {
          Bcur = ShuntVoltage_mV * 1000.0f / ShuntResistanceMicroOhm;
          if (InvertBattAmps == 1) {
            Bcur = Bcur * -1;  // swap sign if necessary
          }
          Bcur = Bcur + BatteryCOffset;
          BatteryCurrent_scaled = Bcur * 100;

          // Only mark fresh on successful, valid readings
          MARK_FRESH(IDX_IBV);
          MARK_FRESH(IDX_BCUR);
        }

        int end33 = micros();               // End timing
        AnalogReadTime2 = end33 - start33;  // Store elapsed time
        if (AnalogReadTime2 > AnalogReadTime) {
          AnalogReadTime = AnalogReadTime2;
        }
      } catch (...) {
        // INA228 read failed - do not call MARK_FRESH
        Serial.println("INA228 read failed");
        queueConsoleMessage("INA228 read failed");
      }
    }
  }

  //ADS1115 reading is based on trigger→wait→read so as to not waste time
  if (ADS1115Disconnected != 0) {
    queueConsoleMessage("theADS1115 was not connected and triggered a return");
    return;  // Early exit - no MARK_FRESH calls
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
            if (BatteryV > 5.0 && BatteryV < 70.0) {  // Sanity check
              MARK_FRESH(IDX_BATTERY_V);              // Only mark fresh on valid reading
            }
            break;
          case 1:
            Channel1V = Raw / 32767.0 * 6.144 * 2;   // voltage divider is 2:1, so this gets us to volts
            MeasuredAmps = (Channel1V - 2.5) * 100;  // alternator current
            if (InvertAltAmps == 1) {
              MeasuredAmps = MeasuredAmps * -1;  // swap sign if necessary
            }
            MeasuredAmps = MeasuredAmps - AlternatorCOffset;
            if (MeasuredAmps > -500 && MeasuredAmps < 500) {  // Sanity check
              MARK_FRESH(IDX_MEASURED_AMPS);                  // Only mark fresh on valid reading
            }
            break;
          case 2:
            Channel2V = Raw / 32767.0 * 2 * 6.144 * RPMScalingFactor;
            RPM = Channel2V;
            if (RPM < 100) {
              RPM = 0;
            }
            if (RPM >= 0 && RPM < 10000) {  // Sanity check
              MARK_FRESH(IDX_RPM);          // Only mark fresh on valid reading
            }
            break;
          case 3:
            Channel3V = Raw / 32767.0 * 6.144 * 833 * 2;
            temperatureThermistor = thermistorTempC(Channel3V);
            if (temperatureThermistor > 500) {
              temperatureThermistor = -99;
            }
            if (Channel3V > 150) {
              Channel3V = -99;
            }
            if (Channel3V > 0 && Channel3V < 100) {  // Sanity check for Channel3V
              MARK_FRESH(IDX_CHANNEL3V);
            }
            if (temperatureThermistor > -50 && temperatureThermistor < 200) {  // Sanity check for temp
              MARK_FRESH(IDX_THERMISTOR_TEMP);
            }
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

  calculateChargeTimes();  // calculate charge/discharge times
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
  for (;;) {
    // Step 1: Trigger a conversion
    sensors.requestTemperaturesByAddress(tempDeviceAddress);

    // Step 2: Wait for conversion to complete while other things run
    vTaskDelay(pdMS_TO_TICKS(5000));  // This is the spacing between reads

    // Step 3: Read the completed result
    uint8_t scratchPad[9];
    if (sensors.readScratchPad(tempDeviceAddress, scratchPad)) {
      int16_t raw = (scratchPad[1] << 8) | scratchPad[0];
      float tempC = raw / 16.0;
      float tempF = tempC * 1.8 + 32.0;

      // Sanity check the temperature reading
      if (tempF > -50 && tempF < 300) {  // Reasonable temperature range
        AlternatorTemperatureF = tempF;
        MARK_FRESH(IDX_ALTERNATOR_TEMP);  // Only mark fresh on valid reading
      } else {
        AlternatorTemperatureF = -99;  // Invalid reading
        queueConsoleMessage("WARNING: Temp sensor reading out of range");
      }
    } else {
      AlternatorTemperatureF = -99;  // Consistent with your error value pattern
      queueConsoleMessage("WARNING: Temp sensor read failed");
    }
  }
}
void UpdateDisplay() {
  // Double-check display availability
  if (!displayAvailable) {
    return;
  }

  // Add a try-catch around all display operations
  try {
    if (millis() - prev_millis66 > 3000) {  // update display every 3 seconds
      unsigned long displayStart = millis();

      // Try display operations with timeout
      u8g2.clearBuffer();
      if (millis() - displayStart > 2000) {
        Serial.println("Display timeout - disabling display");
        queueConsoleMessage("Display timeout - disabling display");

        displayAvailable = false;
        prev_millis66 = millis();
        return;
      }

      u8g2.setFont(u8g2_font_6x10_tf);

      // Row 1 (y=10)
      u8g2.drawStr(0, 10, "Vlts:");
      u8g2.setCursor(35, 10);
      u8g2.print(BatteryV, 2);

      u8g2.drawStr(79, 10, "R:");
      u8g2.setCursor(90, 10);
      u8g2.print(RPM, 0);

      // Row 2 (y=20)
      u8g2.drawStr(0, 20, "Acur:");
      u8g2.setCursor(35, 20);
      u8g2.print(MeasuredAmps, 1);

      u8g2.drawStr(79, 20, "VV:");
      u8g2.setCursor(90, 20);
      u8g2.print(VictronVoltage, 2);

      // Check timeout partway through
      if (millis() - displayStart > 2000) {
        Serial.println("Display timeout during updates - disabling display");
        queueConsoleMessage("Display timeout during updates - disabling display");
        displayAvailable = false;
        prev_millis66 = millis();
        return;
      }

      // Row 3 (y=30)
      u8g2.drawStr(0, 30, "Temp:");
      u8g2.setCursor(35, 30);
      u8g2.print(AlternatorTemperatureF, 1);

      u8g2.drawStr(79, 30, "t:");
      u8g2.setCursor(90, 30);
      u8g2.print("extra");

      // Row 4 (y=40)
      u8g2.drawStr(0, 40, "PWM%:");
      u8g2.setCursor(35, 40);
      u8g2.print(DutyCycle, 1);

      u8g2.drawStr(79, 40, "H:");
      u8g2.setCursor(90, 40);
      u8g2.print(HeadingNMEA);

      // Row 5 (y=50)
      u8g2.drawStr(0, 50, "Vout:");
      u8g2.setCursor(35, 50);
      u8g2.print(vvout, 2);

      // Row 6 (y=60)
      u8g2.drawStr(0, 60, "Bcur:");
      u8g2.setCursor(35, 60);
      u8g2.print(Bcur, 1);

      // Final timeout check before sendBuffer()
      if (millis() - displayStart > 2000) {
        Serial.println("Display timeout before sendBuffer - disabling display");
        queueConsoleMessage("Display timeout before sendBuffer - disabling display");

        displayAvailable = false;
        prev_millis66 = millis();
        return;
      }

      u8g2.sendBuffer();

      // Log if display operations took a long time
      unsigned long totalTime = millis() - displayStart;
      if (totalTime > 1000) {
        Serial.println("Display took: " + String(totalTime) + "ms");
      }

      prev_millis66 = millis();
    }
  } catch (...) {
    Serial.println("Display operation failed - disabling display");
    queueConsoleMessage("Display operation failed - disabling display");
    displayAvailable = false;
    prev_millis66 = millis();
  }
}
void ReadVEData() {
  if (VeData == 1) {
    if (millis() - prev_millis33 > 2000) {  // read VE data every 2 seconds

      int start1 = micros();      // Start timing VeData
      bool dataReceived = false;  // Track if we got any valid data

      while (Serial2.available()) {
        myve.rxData(Serial2.read());
        for (int i = 0; i < myve.veEnd; i++) {
          if (strcmp(myve.veName[i], "V") == 0) {
            float newVoltage = (atof(myve.veValue[i]) / 1000);
            if (newVoltage > 0 && newVoltage < 100) {  // Sanity check
              VictronVoltage = newVoltage;
              MARK_FRESH(IDX_VICTRON_VOLTAGE);  // Only mark fresh on valid data
              dataReceived = true;
            }
          }
          if (strcmp(myve.veName[i], "I") == 0) {
            float newCurrent = (atof(myve.veValue[i]) / 1000);
            if (newCurrent > -1000 && newCurrent < 1000) {  // Sanity check
              VictronCurrent = newCurrent;
              MARK_FRESH(IDX_VICTRON_CURRENT);  // Only mark fresh on valid data
              dataReceived = true;
            }
          }
        }
        yield();  // not sure what this does
      }
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
    //Uncomment below to get serial montior back
    // PrintLabelValWithConversionCheckUnDef("Rudder: ", Instance, 0, true);
    // PrintLabelValWithConversionCheckUnDef("  position (deg): ", RudderPosition, &RadToDeg, true);
    // OutputStream->print("  direction order: ");
    // PrintN2kEnumType(RudderDirectionOrder, OutputStream);
    // PrintLabelValWithConversionCheckUnDef("  angle order (deg): ", AngleOrder, &RadToDeg, true);
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
    // Parsing succeeded - update variable and mark fresh
    HeadingNMEA = Heading * 180.0 / PI;  // Convert radians to degrees
    MARK_FRESH(IDX_HEADING_NMEA);        // Only called on successful parse

    // Uncomment below to get serial monitor output back
    // OutputStream->println("Heading:");
    // PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    // OutputStream->print("  reference: ");
    // PrintN2kEnumType(HeadingReference, OutputStream);
    // PrintLabelValWithConversionCheckUnDef("  Heading (deg): ", Heading, &RadToDeg, true);
    // PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ", Deviation, &RadToDeg, true);
    // PrintLabelValWithConversionCheckUnDef("  Variation (deg): ", Variation, &RadToDeg, true);
  } else {
    // Parsing failed - do NOT call MARK_FRESH, data will go stale automatically
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
  //Serial.println("=== GNSS function called ===");
  //Serial.printf("PGN: %lu\n", N2kMsg.PGN);

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

    //Serial.println("GNSS parsing SUCCESS!");
    //Serial.printf("Raw values - Lat: %f, Lon: %f, Sats: %d\n", Latitude, Longitude, nSatellites);

    // Check if we have valid GPS data (not NaN and reasonable values)
    if (!isnan(Latitude) && !isnan(Longitude) && Latitude != 0.0 && Longitude != 0.0 && abs(Latitude) <= 90.0 && abs(Longitude) <= 180.0 && nSatellites > 0) {

      // Store values globally for web interface
      LatitudeNMEA = Latitude;
      LongitudeNMEA = Longitude;
      SatelliteCountNMEA = nSatellites;

      // Mark all GPS data as fresh - only called on valid data
      MARK_FRESH(IDX_LATITUDE_NMEA);
      MARK_FRESH(IDX_LONGITUDE_NMEA);
      MARK_FRESH(IDX_SATELLITE_COUNT);

      //Serial.printf("Valid GNSS data stored - LatNMEA: %f, LonNMEA: %f, SatNMEA: %d\n", LatitudeNMEA, LongitudeNMEA, SatelliteCountNMEA);
    } else {
      //Serial.println("GNSS data invalid - values are NaN, zero, or out of range");
      // Invalid data - do NOT call MARK_FRESH, let data go stale
    }
  } else {
    //Serial.println("GNSS parsing FAILED!");
    // Parse failed - do NOT call MARK_FRESH, let data go stale
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
  if (NMEA2KVerbose == 1) {
    OutputStream->print("In Main Handler: ");
    OutputStream->println(N2kMsg.PGN);
  }
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++)
    ;

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}
String readFile(fs::FS &fs, const char *path) {
  if (!littleFSMounted && !ensureLittleFS()) {
    Serial.println("Cannot read file - LittleFS not available: " + String(path));
    return String();
  }

  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file: " + String(path));
    return String();
  }

  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  return fileContent;
}
void writeFile(fs::FS &fs, const char *path, const char *message) {
  if (!littleFSMounted && !ensureLittleFS()) {
    Serial.println("Cannot write file - LittleFS not available: " + String(path));
    return;
  }
  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing: " + String(path));
    return;
  }
  if (file.print(message)) {
    // Success - file written
  } else {
    Serial.println("- write failed for: " + String(path));
  }
  delay(2);
  file.close();
}

int SafeInt(float f, int scale = 1) {
  // where this is matters!!   Put utility functions like SafeInt() above setup() and loop() , according to ChatGPT.  And I proved it matters.
  return isnan(f) || isinf(f) ? -1 : (int)(f * scale);
}

void SendWifiData() {
  static unsigned long prev_millis5 = 0;
  static unsigned long lastTimestampSend = 0;
  unsigned long now = millis();
  bool sendingPayloadThisCycle = false;

  // ===== Main CSV Payload =====
  if (now - prev_millis5 > webgaugesinterval) {
    WifiStrength = WiFi.RSSI();
    WifiHeartBeat++;
    processConsoleQueue();  // Process console message queue - send one message per interval
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
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               // ... ALL YOUR EXISTING SafeInt() calls stay exactly the same ...
               // I'm not repeating them here since they're already working

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
               SafeInt(TargetAmpL),                   // 48

               // More Settings
               SafeInt(CurrentThreshold_scaled),    // 49
               SafeInt(PeukertExponent_scaled),     // 50
               SafeInt(ChargeEfficiency_scaled),    // 51
               SafeInt(ChargedVoltage_Scaled),      // 52
               SafeInt(TailCurrent_Scaled),         // 53
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
               SafeInt(VictronCurrent, 100),       // 107
                                                   //AlarmStuff
               SafeInt(AlarmTest),                 // New index: 108
               SafeInt(AlarmLatchEnabled),         // New index: 109
               SafeInt(alarmLatch ? 1 : 0),        // New index: 110 (current latch state)
               SafeInt(ResetAlarmLatch),           // New index: 111
               SafeInt(ForceFloat),                //  112
               SafeInt(bulkCompleteTime),          // 113
               SafeInt(FLOAT_DURATION),            // 114
               SafeInt(LatitudeNMEA * 1000000),    // 115 - Convert to integer with 6 decimal precision
               SafeInt(LongitudeNMEA * 1000000),   // 116 - Convert to integer with 6 decimal precision
               SafeInt(SatelliteCountNMEA),        // 117
               SafeInt(GPIO33_Status),             //  118
               SafeInt(fieldActiveStatus),         // 119
               SafeInt(timeToFullChargeMin),       // 120
               SafeInt(timeToFullDischargeMin)     // 121


      );
      // Send main sensor data payload
      events.send(payload, "CSVData");    // Changed event name to reflect new format
      SendWifiTime = micros() - start66;  // Calculate WiFi Send Time
      sendingPayloadThisCycle = true;     // Flag this as a bad function iteration to also send timestampPayload
    }
    prev_millis5 = now;
  }
  //            timestampPayload - NOW SENDS AGES INSTEAD OF TIMESTAMPS
  if (!sendingPayloadThisCycle && now - lastTimestampSend > 3000) {  // Every 3 seconds
    char timestampPayload[400];                                      // Smaller buffer for ages
    // Calculate ages (how long ago each sensor was updated) instead of absolute timestamps
    snprintf(timestampPayload, sizeof(timestampPayload),
             "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
             (dataTimestamps[IDX_HEADING_NMEA] == 0) ? 999999 : (now - dataTimestamps[IDX_HEADING_NMEA]),        // 0 - Age of heading data
             (dataTimestamps[IDX_LATITUDE_NMEA] == 0) ? 999999 : (now - dataTimestamps[IDX_LATITUDE_NMEA]),      // 1 - Age of latitude data
             (dataTimestamps[IDX_LONGITUDE_NMEA] == 0) ? 999999 : (now - dataTimestamps[IDX_LONGITUDE_NMEA]),    // 2 - Age of longitude data
             (dataTimestamps[IDX_SATELLITE_COUNT] == 0) ? 999999 : (now - dataTimestamps[IDX_SATELLITE_COUNT]),  // 3 - Age of satellite count
             (dataTimestamps[IDX_VICTRON_VOLTAGE] == 0) ? 999999 : (now - dataTimestamps[IDX_VICTRON_VOLTAGE]),  // 4 - Age of Victron voltage
             (dataTimestamps[IDX_VICTRON_CURRENT] == 0) ? 999999 : (now - dataTimestamps[IDX_VICTRON_CURRENT]),  // 5 - Age of Victron current
             (dataTimestamps[IDX_ALTERNATOR_TEMP] == 0) ? 999999 : (now - dataTimestamps[IDX_ALTERNATOR_TEMP]),  // 6 - Age of alternator temp
             (dataTimestamps[IDX_THERMISTOR_TEMP] == 0) ? 999999 : (now - dataTimestamps[IDX_THERMISTOR_TEMP]),  // 7 - Age of thermistor temp
             (dataTimestamps[IDX_RPM] == 0) ? 999999 : (now - dataTimestamps[IDX_RPM]),                          // 8 - Age of RPM data
             (dataTimestamps[IDX_MEASURED_AMPS] == 0) ? 999999 : (now - dataTimestamps[IDX_MEASURED_AMPS]),      // 9 - Age of alternator current
             (dataTimestamps[IDX_BATTERY_V] == 0) ? 999999 : (now - dataTimestamps[IDX_BATTERY_V]),              // 10 - Age of ADS battery voltage
             (dataTimestamps[IDX_IBV] == 0) ? 999999 : (now - dataTimestamps[IDX_IBV]),                          // 11 - Age of INA battery voltage
             (dataTimestamps[IDX_BCUR] == 0) ? 999999 : (now - dataTimestamps[IDX_BCUR]),                        // 12 - Age of battery current
             (dataTimestamps[IDX_CHANNEL3V] == 0) ? 999999 : (now - dataTimestamps[IDX_CHANNEL3V]),              // 13 - Age of ADS Ch3 voltage
             (dataTimestamps[IDX_DUTY_CYCLE] == 0) ? 999999 : (now - dataTimestamps[IDX_DUTY_CYCLE]),            // 14 - Age of duty cycle calculation
             (dataTimestamps[IDX_FIELD_VOLTS] == 0) ? 999999 : (now - dataTimestamps[IDX_FIELD_VOLTS]),          // 15 - Age of field voltage calculation
             (dataTimestamps[IDX_FIELD_AMPS] == 0) ? 999999 : (now - dataTimestamps[IDX_FIELD_AMPS])             // 16 - Age of field current calculation
    );
    events.send(timestampPayload, "TimestampData");
    lastTimestampSend = now;
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
    // Send console message immediately instead of queuing
    events.send("Performing scheduled restart for system maintenance", "console");
    events.send("Device restarting for maintenance. Will reconnect shortly.", "status");
    // Longer delay to ensure messages are sent
    delay(2500);
    ESP.restart();                    // Restart the ESP32
    lastRestartTime = currentMillis;  // This line won't be reached, but for clarity...
  }
}
// Function to set up WiFi - tries to connect to saved network, falls back to AP mode
void setupWiFi() {
  Serial.println("\n=== WiFi Setup Starting ===");
  // PRESERVES: Check for permanent AP mode FIRST (your original logic)
  if (LittleFS.exists(WIFI_MODE_FILE)) {
    String savedMode = readFile(LittleFS, WIFI_MODE_FILE);
    savedMode.trim();
    Serial.println("Found WiFi mode: '" + savedMode + "'");
    if (savedMode == "ap") {
      permanentAPMode = 1;
      Serial.println("=== PERMANENT AP MODE DETECTED ===");
      Serial.println("Should use stored password: '" + esp32_ap_password + "'");
      setupAccessPoint();
      currentWiFiMode = AWIFI_MODE_AP;
      // PRESERVES: Your interface selection logic for permanent AP mode
      if (LittleFS.exists("/index.html")) {
        Serial.println("index.html exists - serving MAIN INTERFACE in permanent AP mode");
        setupServer();  // Full alternator interface
      } else {
        Serial.println("No index.html - serving captive portal landing");
        setupCaptivePortalLanding();  // Simple landing page
      }
      return;
    }
  }
  // PRESERVES: Try client mode - load saved credentials (your original logic)
  String saved_ssid = "";
  String saved_password = "";
  if (LittleFS.exists(WIFI_SSID_FILE)) {
    saved_ssid = readFile(LittleFS, WIFI_SSID_FILE);
    saved_ssid.trim();
    Serial.println("Found SSID: '" + saved_ssid + "'");
  } else {
    Serial.println("No SSID file found");
  }
  if (LittleFS.exists(WIFI_PASS_FILE)) {
    saved_password = readFile(LittleFS, WIFI_PASS_FILE);
    saved_password.trim();
    Serial.println("Found WiFi password (length: " + String(saved_password.length()) + ")");
  } else {
    Serial.println("No WiFi password file found");
  }
  // ENHANCED: Better handling of missing credentials
  if (saved_ssid.length() == 0 || saved_ssid == "WRONG") {
    Serial.println("=== FIRST BOOT DETECTED ===");
    Serial.println("No valid WiFi credentials - entering configuration mode");
    Serial.println("Will serve WiFi configuration form");
    setupAccessPoint();
    setupWiFiConfigServer();  // Serve config interface only
    currentWiFiMode = AWIFI_MODE_AP;
    return;
  } else {
    Serial.println("=== CLIENT MODE ATTEMPT ===");
    Serial.println("Using saved WiFi credentials for: " + saved_ssid);
  }
  // PRESERVES: Client mode attempt (reduced timeout to prevent watchdog issues)
  Serial.println("Attempting WiFi connection to: " + saved_ssid);

  if (connectToWiFi(saved_ssid.c_str(), saved_password.c_str(), 8000)) {  // Reduced from 20s to 8s
    currentWiFiMode = AWIFI_MODE_CLIENT;
    Serial.println("=== CLIENT MODE SUCCESS ===");
    Serial.println("WiFi connected successfully!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    setupServer();  // PRESERVES: Serve full interface in client mode
  } else {
    Serial.println("=== CLIENT MODE FAILED ===");
    Serial.println("WiFi connection failed - starting configuration mode");
    setupAccessPoint();
    setupWiFiConfigServer();  // PRESERVES: Serve config interface only
    currentWiFiMode = AWIFI_MODE_AP;
  }
  Serial.println("=== WiFi Setup Complete ===");
}
// 3. FIX connectToWiFi() TO PREVENT WATCHDOG TIMEOUT
// PRESERVES: All your connection logic, just adds watchdog feeding and better error handling
bool connectToWiFi(const char *ssid, const char *password, unsigned long timeout) {
  if (!ssid || strlen(ssid) == 0) {
    Serial.println("ERROR: No SSID provided for WiFi connection");  // PRESERVES: Your error message style
    return false;
  }

  Serial.printf("Connecting to WiFi: %s\n", ssid);
  Serial.printf("Password length: %d\n", strlen(password));

  // PRESERVES: Your WiFi setup sequence
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Note: Removed WiFi.setAutoConnect(false) as you noted it doesn't exist

  // PRESERVES: Your connection logic
  if (strlen(password) > 0) {
    WiFi.begin(ssid, password);
  } else {
    WiFi.begin(ssid);  // Open network
  }

  unsigned long startTime = millis();
  int attempts = 0;
  const int maxAttempts = timeout / 500;  // Check every 500ms instead of 250ms

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    attempts++;

    // Print progress every 2 seconds (less spam than your original)
    if (attempts % 4 == 0) {
      Serial.printf("WiFi Status: %d, attempt %d/%d\n", WiFi.status(), attempts, maxAttempts);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connection successful!");                         // PRESERVES: Your success message
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());  // PRESERVES: Your IP logging
    Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());               // PRESERVES: Your signal logging

    // PRESERVES: Your mDNS setup
    if (MDNS.begin("alternator")) {
      Serial.println("mDNS responder started");  //
      MDNS.addService("http", "tcp", 80);
    }
    return true;
  } else {
    Serial.printf("WiFi connection failed after %lu ms\n", timeout);  // PRESERVES: Your failure message style
    Serial.printf("Final status: %d\n", WiFi.status());               // PRESERVES: Your debug info
    return false;
  }
}
// Function to set up the device as an access point
void setupAccessPoint() {
  Serial.println("=== SETTING UP ACCESS POINT ===");
  Serial.println("Using SSID: '" + esp32_ap_ssid + "'");
  Serial.println("Using password: '" + esp32_ap_password + "'");
  Serial.println("Password length: " + String(esp32_ap_password.length()));
  Serial.println("SSID length: " + String(esp32_ap_ssid.length()));

  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(esp32_ap_ssid.c_str(), esp32_ap_password.c_str());

  if (apStarted) {
    Serial.println("=== ACCESS POINT STARTED SUCCESSFULLY ===");
    Serial.print("AP SSID: ");
    Serial.println(esp32_ap_ssid);
    Serial.print("AP Password: ");
    Serial.println(esp32_ap_password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Start DNS server for captive portal
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.println("DNS server started for captive portal");

    Serial.println("=== AP SETUP COMPLETE ===");
  } else {
    Serial.println("=== ACCESS POINT FAILED TO START ===");
    Serial.println("This is a critical error!");
    // Try with default settings as fallback
    Serial.println("Trying with default settings as fallback...");
    WiFi.softAP("ALTERNATOR_WIFI", "alternator123");
  }
}
// Function to set up the configuration web server in AP mode
void setupWiFiConfigServer() {
  Serial.println("\n=== SETTING UP WIFI CONFIG SERVER ===");

  // Main config page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("=== CONFIG PAGE REQUEST ===");
    Serial.println("Client IP: " + request->client()->remoteIP().toString());
    Serial.println("User-Agent: " + request->header("User-Agent"));
    Serial.println("Serving WiFi configuration page");
    request->send_P(200, "text/html", WIFI_CONFIG_HTML);
  });

  // Enhanced WiFi configuration handler with custom SSID support
  server.on("/wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== WIFI CONFIG POST RECEIVED ===");
    Serial.println("Client IP: " + request->client()->remoteIP().toString());

    // Debug: Print all received parameters
    int params = request->params();
    Serial.println("Parameters received: " + String(params));
    for (int i = 0; i < params; i++) {
      auto p = request->getParam(i);
      if (p->name() == "ap_password" || p->name() == "password") {
        //Serial.println("Param " + String(i) + ": " + p->name() + " = '[REDACTED]' (length: " + String(p->value().length()) + ")");
      } else {
        // Serial.println("Param " + String(i) + ": " + p->name() + " = '" + p->value() + "'");
      }
    }

    String mode = "client";
    String ssid = "";
    String password = "";
    String ap_password = "";
    String hotspot_ssid = "";

    // Get parameters
    if (request->hasParam("mode", true)) {
      mode = request->getParam("mode", true)->value();
      mode.trim();
      Serial.println("Selected mode: " + mode);
    }

    if (request->hasParam("ap_password", true)) {
      ap_password = request->getParam("ap_password", true)->value();
      ap_password.trim();
      Serial.println("New AP password length: " + String(ap_password.length()));
    }

    if (request->hasParam("hotspot_ssid", true)) {
      hotspot_ssid = request->getParam("hotspot_ssid", true)->value();
      hotspot_ssid.trim();
      Serial.println("Custom hotspot SSID: '" + hotspot_ssid + "'");
    }

    if (request->hasParam("ssid", true)) {
      ssid = request->getParam("ssid", true)->value();
      ssid.trim();
      Serial.println("Ship WiFi SSID: '" + ssid + "'");
    }

    if (request->hasParam("password", true)) {
      password = request->getParam("password", true)->value();
      password.trim();
      Serial.println("Ship WiFi password length: " + String(password.length()));
    }

    // Validate AP password
    if (ap_password.length() == 0) {
      Serial.println("ERROR: Empty AP password provided");
      request->send(400, "text/html",
                    "<html><body><h2>Error</h2><p>Alternator hotspot password cannot be empty.</p>"
                    "<a href='/'>Go Back</a></body></html>");
      return;
    }

    if (ap_password.length() < 8) {
      Serial.println("ERROR: AP password too short (WPA2 requires 8+ characters)");
      request->send(400, "text/html",
                    "<html><body><h2>Error</h2><p>Password must be at least 8 characters for WPA2.</p>"
                    "<a href='/'>Go Back</a></body></html>");
      return;
    }


    // Save configuration
    Serial.println("=== SAVING CONFIGURATION ===");
    Serial.println("Updating in-memory values first...");
    esp32_ap_password = ap_password;  // Update memory FIRST
    Serial.println("New AP password in memory: length=" + String(esp32_ap_password.length()));

    writeFile(LittleFS, AP_PASSWORD_FILE, ap_password.c_str());
    Serial.println("AP password saved to file");

    // Save custom SSID if provided
    if (hotspot_ssid.length() > 0) {
      writeFile(LittleFS, AP_SSID_FILE, hotspot_ssid.c_str());
      esp32_ap_ssid = hotspot_ssid;
      Serial.println("Custom AP SSID saved: " + esp32_ap_ssid);
    } else {
      if (LittleFS.exists(AP_SSID_FILE)) {
        LittleFS.remove(AP_SSID_FILE);
      }
      esp32_ap_ssid = "ALTERNATOR_WIFI";
      Serial.println("Using default SSID");
    }

    if (mode == "ap") {
      // Permanent AP mode
      Serial.println("=== CONFIGURING PERMANENT AP MODE ===");
      writeFile(LittleFS, WIFI_MODE_FILE, "ap");
      permanentAPMode = 1;
      Serial.println("Permanent AP mode saved");

      // Send simple success response
      request->send(200, "text/plain", "Configuration saved! Please restart device.");

      Serial.println("=== RESTARTING FOR AP MODE ===");
      Serial.println("Waiting 5 seconds for filesystem sync...");
      delay(5000);  // Give filesystem time to sync
      ESP.restart();

    } else {
      // Client mode
      if (ssid.length() == 0) {
        Serial.println("ERROR: Empty SSID for client mode");
        request->send(400, "text/html",
                      "<html><body><h2>Error</h2><p>Ship WiFi credentials required for client mode.</p>"
                      "<a href='/'>Go Back</a></body></html>");
        return;
      }

      Serial.println("=== CONFIGURING CLIENT MODE ===");
      writeFile(LittleFS, "/ssid.txt", ssid.c_str());
      writeFile(LittleFS, "/pass.txt", password.c_str());
      writeFile(LittleFS, WIFI_MODE_FILE, "client");
      permanentAPMode = 0;

      // Send simple success response
      request->send(200, "text/plain", "Configuration saved! Please restart device.");

      Serial.println("=== RESTARTING FOR CLIENT MODE ===");
      ESP.restart();
    }
  });

  // Enhanced 404 handler (your existing one with debugging)
  server.onNotFound([](AsyncWebServerRequest *request) {
    String path = request->url();
    Serial.println("=== SERVER REQUEST DEBUG ===");
    Serial.print("Request for: ");
    Serial.println(path);
    Serial.println("Client IP: " + request->client()->remoteIP().toString());
    Serial.println("Method: " + String(request->method() == HTTP_GET ? "GET" : "POST"));

    if (LittleFS.exists(path)) {
      String contentType = "text/html";
      if (path.endsWith(".css")) contentType = "text/css";
      else if (path.endsWith(".js")) contentType = "application/javascript";
      else if (path.endsWith(".json")) contentType = "application/json";
      else if (path.endsWith(".png")) contentType = "image/png";
      else if (path.endsWith(".jpg")) contentType = "image/jpeg";

      Serial.println("File found in LittleFS, serving with content-type: " + contentType);
      request->send(LittleFS, path, contentType);
    } else {
      Serial.print("File not found in LittleFS: ");
      Serial.println(path);
      Serial.println("Redirecting to captive portal: " + WiFi.softAPIP().toString());
      request->redirect("http://" + WiFi.softAPIP().toString());
    }
  });

  server.begin();
  Serial.println("=== CONFIG SERVER STARTED ===");
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
      "/AlternatorTemperatureLimitF.txt", "/ManualDuty.txt", "/FullChargeVoltage.txt",
      "/TargetAmps.txt", "/SwitchingFrequency.txt", "/TargetFloatVoltage.txt", "/R_fixed.txt", "/Beta.txt", "/T0_C.txt", "/TempSource.txt", "/interval.txt",
      "/FieldAdjustmentInterval.txt", "/ManualFieldToggle.txt",
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
      "/AlternatorCOffset.txt", "/BatteryCOffset.txt", "/bulkCompleteTime.txt", "/FLOAT_DURATION.txt"
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
      bool foundParameter = false;
      String inputMessage;
      if (request->hasParam("AlternatorTemperatureLimitF")) {
        foundParameter = true;
        inputMessage = request->getParam("AlternatorTemperatureLimitF")->value();
        writeFile(LittleFS, "/AlternatorTemperatureLimitF.txt", inputMessage.c_str());
        AlternatorTemperatureLimitF = inputMessage.toInt();
      } else if (request->hasParam("ManualDuty")) {
        foundParameter = true;
        inputMessage = request->getParam("ManualDuty")->value();
        writeFile(LittleFS, "/ManualDuty.txt", inputMessage.c_str());
        ManualDutyTarget = inputMessage.toInt();
      } else if (request->hasParam("FullChargeVoltage")) {
        foundParameter = true;
        inputMessage = request->getParam("FullChargeVoltage")->value();
        writeFile(LittleFS, "/FullChargeVoltage.txt", inputMessage.c_str());
        ChargingVoltageTarget = inputMessage.toFloat();
      } else if (request->hasParam("TargetAmps")) {
        foundParameter = true;
        inputMessage = request->getParam("TargetAmps")->value();
        writeFile(LittleFS, "/TargetAmps.txt", inputMessage.c_str());
        TargetAmps = inputMessage.toInt();
      } else if (request->hasParam("SwitchingFrequency")) {
        foundParameter = true;
        inputMessage = request->getParam("SwitchingFrequency")->value();
        int requestedFreq = inputMessage.toInt();
        // Limit to 1100Hz to prevent ESP32 PWM shutdown
        //There are solutions for this later if necessary, use lower-level ESP-IDF functions, not worth it right now
        if (requestedFreq > 1100) {
          requestedFreq = 1100;
          queueConsoleMessage("Frequency limited to 1100Hz maximum");
        }
        writeFile(LittleFS, "/SwitchingFrequency.txt", String(requestedFreq).c_str());
        fffr = requestedFreq;
        ledcDetach(pwmPin);
        delay(50);
        ledcAttach(pwmPin, fffr, pwmResolution);
        queueConsoleMessage("Switching frequency set to " + String(fffr) + "Hz");
      } else if (request->hasParam("TargetFloatVoltage")) {
        foundParameter = true;
        inputMessage = request->getParam("TargetFloatVoltage")->value();
        writeFile(LittleFS, "/TargetFloatVoltage.txt", inputMessage.c_str());
        TargetFloatVoltage = inputMessage.toFloat();
      } else if (request->hasParam("interval")) {
        foundParameter = true;
        inputMessage = request->getParam("interval")->value();
        writeFile(LittleFS, "/interval.txt", inputMessage.c_str());
        interval = inputMessage.toFloat();
        dutyStep = interval;  // Apply new step size immediately
      } else if (request->hasParam("FieldAdjustmentInterval")) {
        foundParameter = true;
        inputMessage = request->getParam("FieldAdjustmentInterval")->value();
        writeFile(LittleFS, "/FieldAdjustmentInterval.txt", inputMessage.c_str());
        FieldAdjustmentInterval = inputMessage.toFloat();
      } else if (request->hasParam("ManualFieldToggle")) {
        foundParameter = true;
        inputMessage = request->getParam("ManualFieldToggle")->value();
        writeFile(LittleFS, "/ManualFieldToggle.txt", inputMessage.c_str());
        ManualFieldToggle = inputMessage.toInt();
      } else if (request->hasParam("SwitchControlOverride")) {
        foundParameter = true;
        inputMessage = request->getParam("SwitchControlOverride")->value();
        writeFile(LittleFS, "/SwitchControlOverride.txt", inputMessage.c_str());
        SwitchControlOverride = inputMessage.toInt();
      } else if (request->hasParam("ForceFloat")) {
        foundParameter = true;
        inputMessage = request->getParam("ForceFloat")->value();
        writeFile(LittleFS, "/ForceFloat.txt", inputMessage.c_str());
        ForceFloat = inputMessage.toInt();
        queueConsoleMessage("ForceFloat mode " + String(ForceFloat ? "enabled" : "disabled"));
      } else if (request->hasParam("OnOff")) {
        foundParameter = true;
        inputMessage = request->getParam("OnOff")->value();
        writeFile(LittleFS, "/OnOff.txt", inputMessage.c_str());
        OnOff = inputMessage.toInt();
      } else if (request->hasParam("HiLow")) {
        foundParameter = true;
        inputMessage = request->getParam("HiLow")->value();
        writeFile(LittleFS, "/HiLow.txt", inputMessage.c_str());
        HiLow = inputMessage.toInt();
      } else if (request->hasParam("InvertAltAmps")) {
        foundParameter = true;
        inputMessage = request->getParam("InvertAltAmps")->value();
        writeFile(LittleFS, "/InvertAltAmps.txt", inputMessage.c_str());
        InvertAltAmps = inputMessage.toInt();
      } else if (request->hasParam("InvertBattAmps")) {
        foundParameter = true;
        inputMessage = request->getParam("InvertBattAmps")->value();
        writeFile(LittleFS, "/InvertBattAmps.txt", inputMessage.c_str());
        InvertBattAmps = inputMessage.toInt();
      } else if (request->hasParam("MaxDuty")) {
        foundParameter = true;
        inputMessage = request->getParam("MaxDuty")->value();
        writeFile(LittleFS, "/MaxDuty.txt", inputMessage.c_str());
        MaxDuty = inputMessage.toInt();
      } else if (request->hasParam("MinDuty")) {
        foundParameter = true;
        inputMessage = request->getParam("MinDuty")->value();
        writeFile(LittleFS, "/MinDuty.txt", inputMessage.c_str());
        MinDuty = inputMessage.toInt();
      } else if (request->hasParam("LimpHome")) {
        foundParameter = true;
        inputMessage = request->getParam("LimpHome")->value();
        writeFile(LittleFS, "/LimpHome.txt", inputMessage.c_str());
        LimpHome = inputMessage.toInt();
      } else if (request->hasParam("VeData")) {
        foundParameter = true;
        inputMessage = request->getParam("VeData")->value();
        writeFile(LittleFS, "/VeData.txt", inputMessage.c_str());
        VeData = inputMessage.toInt();
      } else if (request->hasParam("NMEA0183Data")) {
        foundParameter = true;
        inputMessage = request->getParam("NMEA0183Data")->value();
        writeFile(LittleFS, "/NMEA0183Data.txt", inputMessage.c_str());
        NMEA0183Data = inputMessage.toInt();
      } else if (request->hasParam("NMEA2KData")) {
        foundParameter = true;
        inputMessage = request->getParam("NMEA2KData")->value();
        writeFile(LittleFS, "/NMEA2KData.txt", inputMessage.c_str());
        NMEA2KData = inputMessage.toInt();
      } else if (request->hasParam("TargetAmpL")) {
        foundParameter = true;
        inputMessage = request->getParam("TargetAmpL")->value();
        writeFile(LittleFS, "/TargetAmpL.txt", inputMessage.c_str());
        TargetAmpL = inputMessage.toInt();
      } else if (request->hasParam("CurrentThreshold")) {
        foundParameter = true;
        inputMessage = request->getParam("CurrentThreshold")->value();
        writeFile(LittleFS, "/CurrentThreshold.txt", inputMessage.c_str());
        CurrentThreshold_scaled = inputMessage.toInt();
      } else if (request->hasParam("PeukertExponent")) {
        foundParameter = true;
        inputMessage = request->getParam("PeukertExponent")->value();
        writeFile(LittleFS, "/PeukertExponent.txt", inputMessage.c_str());
        PeukertExponent_scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargeEfficiency")) {
        foundParameter = true;
        inputMessage = request->getParam("ChargeEfficiency")->value();
        writeFile(LittleFS, "/ChargeEfficiency.txt", inputMessage.c_str());
        ChargeEfficiency_scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargedVoltage")) {
        foundParameter = true;
        inputMessage = request->getParam("ChargedVoltage")->value();
        writeFile(LittleFS, "/ChargedVoltage.txt", inputMessage.c_str());
        ChargedVoltage_Scaled = inputMessage.toInt();
      } else if (request->hasParam("TailCurrent")) {
        foundParameter = true;
        inputMessage = request->getParam("TailCurrent")->value();
        writeFile(LittleFS, "/TailCurrent.txt", inputMessage.c_str());
        TailCurrent_Scaled = inputMessage.toInt();
      } else if (request->hasParam("ChargedDetectionTime")) {
        foundParameter = true;
        inputMessage = request->getParam("ChargedDetectionTime")->value();
        writeFile(LittleFS, "/ChargedDetectionTime.txt", inputMessage.c_str());
        ChargedDetectionTime = inputMessage.toInt();
      } else if (request->hasParam("IgnoreTemperature")) {
        foundParameter = true;
        inputMessage = request->getParam("IgnoreTemperature")->value();
        writeFile(LittleFS, "/IgnoreTemperature.txt", inputMessage.c_str());
        IgnoreTemperature = inputMessage.toInt();
      } else if (request->hasParam("BMSLogic")) {
        foundParameter = true;
        inputMessage = request->getParam("BMSLogic")->value();
        writeFile(LittleFS, "/BMSLogic.txt", inputMessage.c_str());
        BMSlogic = inputMessage.toInt();
      } else if (request->hasParam("BMSLogicLevelOff")) {
        foundParameter = true;
        inputMessage = request->getParam("BMSLogicLevelOff")->value();
        writeFile(LittleFS, "/BMSLogicLevelOff.txt", inputMessage.c_str());
        BMSLogicLevelOff = inputMessage.toInt();
      } else if (request->hasParam("AlarmActivate")) {
        foundParameter = true;
        inputMessage = request->getParam("AlarmActivate")->value();
        writeFile(LittleFS, "/AlarmActivate.txt", inputMessage.c_str());
        AlarmActivate = inputMessage.toInt();
      } else if (request->hasParam("TempAlarm")) {
        foundParameter = true;
        inputMessage = request->getParam("TempAlarm")->value();
        writeFile(LittleFS, "/TempAlarm.txt", inputMessage.c_str());
        TempAlarm = inputMessage.toInt();
      } else if (request->hasParam("VoltageAlarmHigh")) {
        foundParameter = true;
        inputMessage = request->getParam("VoltageAlarmHigh")->value();
        writeFile(LittleFS, "/VoltageAlarmHigh.txt", inputMessage.c_str());
        VoltageAlarmHigh = inputMessage.toInt();
      } else if (request->hasParam("VoltageAlarmLow")) {
        foundParameter = true;
        inputMessage = request->getParam("VoltageAlarmLow")->value();
        writeFile(LittleFS, "/VoltageAlarmLow.txt", inputMessage.c_str());
        VoltageAlarmLow = inputMessage.toInt();
      } else if (request->hasParam("CurrentAlarmHigh")) {
        foundParameter = true;
        inputMessage = request->getParam("CurrentAlarmHigh")->value();
        writeFile(LittleFS, "/CurrentAlarmHigh.txt", inputMessage.c_str());
        CurrentAlarmHigh = inputMessage.toInt();
      } else if (request->hasParam("FourWay")) {
        foundParameter = true;
        inputMessage = request->getParam("FourWay")->value();
        writeFile(LittleFS, "/FourWay.txt", inputMessage.c_str());
        FourWay = inputMessage.toInt();
      } else if (request->hasParam("RPMScalingFactor")) {
        foundParameter = true;
        inputMessage = request->getParam("RPMScalingFactor")->value();
        writeFile(LittleFS, "/RPMScalingFactor.txt", inputMessage.c_str());
        RPMScalingFactor = inputMessage.toInt();
      } else if (request->hasParam("FieldResistance")) {
        foundParameter = true;
        inputMessage = request->getParam("FieldResistance")->value();
        writeFile(LittleFS, "/FieldResistance.txt", inputMessage.c_str());
        FieldResistance = inputMessage.toInt();
      } else if (request->hasParam("AlternatorCOffset")) {
        foundParameter = true;
        inputMessage = request->getParam("AlternatorCOffset")->value();
        writeFile(LittleFS, "/AlternatorCOffset.txt", inputMessage.c_str());
        AlternatorCOffset = inputMessage.toFloat();
      } else if (request->hasParam("BatteryCOffset")) {
        foundParameter = true;
        inputMessage = request->getParam("BatteryCOffset")->value();
        writeFile(LittleFS, "/BatteryCOffset.txt", inputMessage.c_str());
        BatteryCOffset = inputMessage.toFloat();
      } else if (request->hasParam("AmpSrc")) {
        foundParameter = true;
        inputMessage = request->getParam("AmpSrc")->value();
        writeFile(LittleFS, "/AmpSrc.txt", inputMessage.c_str());
        AmpSrc = inputMessage.toInt();
        queueConsoleMessage("AmpSrc changed to: " + String(AmpSrc));  // debug line
      } else if (request->hasParam("BatteryVoltageSource")) {
        foundParameter = true;
        inputMessage = request->getParam("BatteryVoltageSource")->value();
        writeFile(LittleFS, "/BatteryVoltageSource.txt", inputMessage.c_str());
        BatteryVoltageSource = inputMessage.toInt();
        queueConsoleMessage("Battery voltage source changed to: " + String(BatteryVoltageSource));  // Debug line
      } else if (request->hasParam("R_fixed")) {
        foundParameter = true;
        inputMessage = request->getParam("R_fixed")->value();
        writeFile(LittleFS, "/R_fixed.txt", inputMessage.c_str());
        R_fixed = inputMessage.toFloat();
      } else if (request->hasParam("Beta")) {
        foundParameter = true;
        inputMessage = request->getParam("Beta")->value();
        writeFile(LittleFS, "/Beta.txt", inputMessage.c_str());
        Beta = inputMessage.toFloat();
      } else if (request->hasParam("T0_C")) {
        foundParameter = true;
        inputMessage = request->getParam("T0_C")->value();
        writeFile(LittleFS, "/T0_C.txt", inputMessage.c_str());
        T0_C = inputMessage.toFloat();
      } else if (request->hasParam("TempSource")) {
        foundParameter = true;
        inputMessage = request->getParam("TempSource")->value();
        writeFile(LittleFS, "/TempSource.txt", inputMessage.c_str());
        TempSource = inputMessage.toInt();
      } else if (request->hasParam("IgnitionOverride")) {
        foundParameter = true;
        inputMessage = request->getParam("IgnitionOverride")->value();
        writeFile(LittleFS, "/IgnitionOverride.txt", inputMessage.c_str());
        IgnitionOverride = inputMessage.toInt();
      }

      else if (request->hasParam("AlarmLatchEnabled")) {
        foundParameter = true;
        inputMessage = request->getParam("AlarmLatchEnabled")->value();
        writeFile(LittleFS, "/AlarmLatchEnabled.txt", inputMessage.c_str());
        AlarmLatchEnabled = inputMessage.toInt();
      }

      else if (request->hasParam("AlarmTest")) {
        foundParameter = true;
        AlarmTest = 1;  // Set the flag - don't save to file as it's momentary
        queueConsoleMessage("ALARM TEST: Initiated from web interface");
        inputMessage = "1";  // Return confirmation
      }

      else if (request->hasParam("ResetAlarmLatch")) {
        foundParameter = true;
        ResetAlarmLatch = 1;  // Set the flag - don't save to file as it's momentary
        queueConsoleMessage("ALARM LATCH: Reset requested from web interface NO FUNCTION!");
        inputMessage = "1";  // Return confirmation
      }

      // else if (request->hasParam("ResetAlarmLatch")) { // this whole thing is hopefullly obsolete
      //   inputMessage = request->getParam("ResetAlarmLatch")->value();
      //  ResetAlarmLatch = inputMessage.toInt();  // Don't save to file - momentary action
      //   resetAlarmLatch();                       // Call the reset function
      //  }

      else if (request->hasParam("bulkCompleteTime")) {
        foundParameter = true;
        inputMessage = request->getParam("bulkCompleteTime")->value();
        writeFile(LittleFS, "/bulkCompleteTime.txt", inputMessage.c_str());
        bulkCompleteTime = inputMessage.toInt();
      }  // When sending to ESP32
      else if (request->hasParam("FLOAT_DURATION")) {
        foundParameter = true;
        inputMessage = request->getParam("FLOAT_DURATION")->value();
        int hours = inputMessage.toInt();
        int seconds = hours * 3600;  // Convert to seconds
        writeFile(LittleFS, "/FLOAT_DURATION.txt", String(seconds).c_str());
        FLOAT_DURATION = seconds;
      }

      // Reset button Code
      else if (request->hasParam("ResetThermTemp")) {
        foundParameter = true;
        MaxTemperatureThermistor = 0;
        writeFile(LittleFS, "/MaxTemperatureThermistor.txt", "0");
        queueConsoleMessage("Max Thermistor Temp: Reset requested from web interface");
      } else if (request->hasParam("ResetTemp")) {
        foundParameter = true;
        MaxAlternatorTemperatureF = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/MaxAlternatorTemperatureF.txt", "0");  // not pointless
        queueConsoleMessage("Max Alterantor Temp: Reset requested from web interface");
      } else if (request->hasParam("ResetVoltage")) {
        foundParameter = true;
        IBVMax = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/IBVMax.txt", "0");  // not pointless
        queueConsoleMessage("Max Voltage: Reset requested from web interface");
      } else if (request->hasParam("ResetCurrent")) {
        foundParameter = true;
        MeasuredAmpsMax = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/MeasuredAmpsMax.txt", "0");  // not pointless
        queueConsoleMessage("Max Battery Current: Reset requested from web interface");
      } else if (request->hasParam("ResetEngineRunTime")) {
        foundParameter = true;
        EngineRunTime = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/EngineRunTime.txt", "0");  // not pointless
        queueConsoleMessage("Engine Run Time: Reset requested from web interface");
      } else if (request->hasParam("ResetAlternatorOnTime")) {
        foundParameter = true;
        AlternatorOnTime = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorOnTime.txt", "0");  // not pointless
        queueConsoleMessage("Alternator On Time: Reset requested from web interface");
      } else if (request->hasParam("ResetEnergy")) {
        foundParameter = true;
        ChargedEnergy = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/ChargedEnergy.txt", "0");  // not pointless
        queueConsoleMessage("Battery Charged Energy: Reset requested from web interface");
      } else if (request->hasParam("ResetDischargedEnergy")) {
        foundParameter = true;
        DischargedEnergy = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/DischargedEnergy.txt", "0");  // not pointless
        queueConsoleMessage("Battery Discharged Energy: Reset requested from web interface");
      } else if (request->hasParam("ResetFuelUsed")) {
        foundParameter = true;
        AlternatorFuelUsed = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorFuelUsed.txt", "0");  // not pointless
        queueConsoleMessage("Fuel Used: Reset requested from web interface");
      } else if (request->hasParam("ResetAlternatorChargedEnergy")) {
        foundParameter = true;
        AlternatorChargedEnergy = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/AlternatorChargedEnergy.txt", "0");  // not pointless
        queueConsoleMessage("Alternator Charged Energy: Reset requested from web interface");
      } else if (request->hasParam("ResetEngineCycles")) {
        foundParameter = true;
        EngineCycles = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/EngineCycles.txt", "0");  // not pointless
        queueConsoleMessage("Engine Cycles: Reset requested from web interface");
      } else if (request->hasParam("ResetRPMMax")) {
        foundParameter = true;
        RPMMax = 0;                               // reset the variable on ESP32 mem
        writeFile(LittleFS, "/RPMMax.txt", "0");  // not pointless
        queueConsoleMessage("Max Engine Speed: Reset requested from web interface");
      } else if (request->hasParam("MaximumAllowedBatteryAmps")) {
        foundParameter = true;
        inputMessage = request->getParam("MaximumAllowedBatteryAmps")->value();
        writeFile(LittleFS, "/MaximumAllowedBatteryAmps.txt", inputMessage.c_str());
        MaximumAllowedBatteryAmps = inputMessage.toInt();
      } else if (request->hasParam("ManualSOCPoint")) {
        foundParameter = true;
        inputMessage = request->getParam("ManualSOCPoint")->value();
        writeFile(LittleFS, "/ManualSOCPoint.txt", inputMessage.c_str());
        ManualSOCPoint = inputMessage.toInt();
      } else if (request->hasParam("AmpControlByRPM")) {
        foundParameter = true;
        inputMessage = request->getParam("AmpControlByRPM")->value();
        writeFile(LittleFS, "/AmpControlByRPM.txt", inputMessage.c_str());
        AmpControlByRPM = inputMessage.toInt();
      } else if (request->hasParam("ShuntResistanceMicroOhm")) {
        foundParameter = true;
        inputMessage = request->getParam("ShuntResistanceMicroOhm")->value();
        writeFile(LittleFS, "/ShuntResistanceMicroOhm.txt", inputMessage.c_str());
        ShuntResistanceMicroOhm = inputMessage.toInt();
      } else if (request->hasParam("maxPoints")) {
        foundParameter = true;
        inputMessage = request->getParam("maxPoints")->value();
        writeFile(LittleFS, "/maxPoints.txt", inputMessage.c_str());
        maxPoints = inputMessage.toInt();
      }
      // Handle RPM/AMPS table - use separate if statements, not else if, becuase we are sending more than 1 value at a time, unlike all the others!
      if (request->hasParam("RPM1")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM1")->value();
        writeFile(LittleFS, "/RPM1.txt", inputMessage.c_str());
        RPM1 = inputMessage.toInt();
      }
      if (request->hasParam("RPM2")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM2")->value();
        writeFile(LittleFS, "/RPM2.txt", inputMessage.c_str());
        RPM2 = inputMessage.toInt();
      }
      if (request->hasParam("RPM3")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM3")->value();
        writeFile(LittleFS, "/RPM3.txt", inputMessage.c_str());
        RPM3 = inputMessage.toInt();
      }
      if (request->hasParam("RPM4")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM4")->value();
        writeFile(LittleFS, "/RPM4.txt", inputMessage.c_str());
        RPM4 = inputMessage.toInt();
      }
      if (request->hasParam("RPM5")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM5")->value();
        writeFile(LittleFS, "/RPM5.txt", inputMessage.c_str());
        RPM5 = inputMessage.toInt();
      }
      if (request->hasParam("RPM6")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM6")->value();
        writeFile(LittleFS, "/RPM6.txt", inputMessage.c_str());
        RPM6 = inputMessage.toInt();
      }
      if (request->hasParam("RPM7")) {
        foundParameter = true;
        inputMessage = request->getParam("RPM7")->value();
        writeFile(LittleFS, "/RPM7.txt", inputMessage.c_str());
        RPM7 = inputMessage.toInt();
      }
      if (request->hasParam("Amps1")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps1")->value();
        writeFile(LittleFS, "/Amps1.txt", inputMessage.c_str());
        Amps1 = inputMessage.toInt();
      }
      if (request->hasParam("Amps2")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps2")->value();
        writeFile(LittleFS, "/Amps2.txt", inputMessage.c_str());
        Amps2 = inputMessage.toInt();
      }
      if (request->hasParam("Amps3")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps3")->value();
        writeFile(LittleFS, "/Amps3.txt", inputMessage.c_str());
        Amps3 = inputMessage.toInt();
      }
      if (request->hasParam("Amps4")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps4")->value();
        writeFile(LittleFS, "/Amps4.txt", inputMessage.c_str());
        Amps4 = inputMessage.toInt();
      }
      if (request->hasParam("Amps5")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps5")->value();
        writeFile(LittleFS, "/Amps5.txt", inputMessage.c_str());
        Amps5 = inputMessage.toInt();
      }
      if (request->hasParam("Amps6")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps6")->value();
        writeFile(LittleFS, "/Amps6.txt", inputMessage.c_str());
        Amps6 = inputMessage.toInt();
      }
      if (request->hasParam("Amps7")) {
        foundParameter = true;
        inputMessage = request->getParam("Amps7")->value();
        writeFile(LittleFS, "/Amps7.txt", inputMessage.c_str());
        Amps7 = inputMessage.toInt();
      }
      if (!foundParameter) {
        inputMessage = "No message sent, the request_hasParam found no match";
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

    if (LittleFS.exists(path)) {
      String contentType = "text/html";
      if (path.endsWith(".css")) contentType = "text/css";
      else if (path.endsWith(".js")) contentType = "application/javascript";
      else if (path.endsWith(".json")) contentType = "application/json";
      else if (path.endsWith(".png")) contentType = "image/png";
      else if (path.endsWith(".jpg")) contentType = "image/jpeg";

      Serial.println("File found in LittleFS, serving with content-type: " + contentType);
      request->send(LittleFS, path, contentType);
    } else {
      Serial.print("File not found in LittleFS: ");
      Serial.println(path);
      Serial.println("Redirecting to captive portal: " + WiFi.softAPIP().toString());
      request->redirect("http://" + WiFi.softAPIP().toString());
    }
  });

  // Setup event source for real-time updates
  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
  Serial.println("=== CONFIG SERVER STARTED ===");
}

//process dns request for captive portals
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
    float batteryDeltaAh = deltaAh * peukertFactor;
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
  if ((abs(BatteryCurrent_scaled) <= (TailCurrent_Scaled * BatteryCapacity_Ah)) && (Voltage_scaled >= ChargedVoltage_Scaled)) {
    FullChargeTimer += elapsedSeconds;
    if (FullChargeTimer >= ChargedDetectionTime) {
      SoC_percent = 10000;  // 100.00% (scaled by 100)
      CoulombCount_Ah_scaled = BatteryCapacity_Ah * 100;
      FullChargeDetected = true;
      coulombAccumulator_Ah = 0.0f;
      queueConsoleMessage("BATTERY: Full charge detected - SoC reset to 100%");
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
    queueConsoleMessage("Password hash saved to LittleFS");
  } else {
    Serial.println("Failed to open password.hash for writing");
    queueConsoleMessage("Failed to open password.hash for writing");
  }
}

void savePasswordPlaintext(const char *password) {
  File file = LittleFS.open("/password.txt", "w");
  if (file) {
    file.println(password);
    file.close();
    Serial.println("Password saved to LittleFS");
    queueConsoleMessage("Password saved");

  } else {
    Serial.println("Failed to open password.txt for writing");
    queueConsoleMessage("Password save failed");
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

  if (!LittleFS.exists("/ChargeEfficiency.txt")) {
    writeFile(LittleFS, "/ChargeEfficiency.txt", String(ChargeEfficiency_scaled).c_str());
  } else {
    ChargeEfficiency_scaled = readFile(LittleFS, "/ChargeEfficiency.txt").toInt();
  }


  if (!LittleFS.exists("/TailCurrent.txt")) {
    writeFile(LittleFS, "/TailCurrent.txt", String(TailCurrent_Scaled).c_str());
  } else {
    TailCurrent_Scaled = readFile(LittleFS, "/TailCurrent.txt").toInt();
  }

  if (!LittleFS.exists("/FuelEfficiency.txt")) {
    writeFile(LittleFS, "/FuelEfficiency.txt", String(FuelEfficiency_scaled).c_str());
  } else {
    FuelEfficiency_scaled = readFile(LittleFS, "/FuelEfficiency.txt").toInt();
  }
  //////////////////////////////////
  if (!LittleFS.exists("/AlternatorTemperatureLimitF.txt")) {
    writeFile(LittleFS, "/AlternatorTemperatureLimitF.txt", String(AlternatorTemperatureLimitF).c_str());
  } else {
    AlternatorTemperatureLimitF = readFile(LittleFS, "/AlternatorTemperatureLimitF.txt").toInt();
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
  if (!LittleFS.exists("/FieldAdjustmentInterval.txt")) {
    writeFile(LittleFS, "/FieldAdjustmentInterval.txt", String(FieldAdjustmentInterval).c_str());
  } else {
    FieldAdjustmentInterval = readFile(LittleFS, "/FieldAdjustmentInterval.txt").toFloat();
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
  if (!LittleFS.exists("/ForceFloat.txt")) {
    writeFile(LittleFS, "/ForceFloat.txt", String(ForceFloat).c_str());
  } else {
    ForceFloat = readFile(LittleFS, "/ForceFloat.txt").toInt();
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
    writeFile(LittleFS, "/TargetAmpL.txt", String(TargetAmpL).c_str());
  } else {
    TargetAmpL = readFile(LittleFS, "/TargetAmpL.txt").toInt();
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
    writeFile(LittleFS, "/ChargedVoltage.txt", String(ChargedVoltage_Scaled).c_str());
  } else {
    ChargedVoltage_Scaled = readFile(LittleFS, "/ChargedVoltage.txt").toInt();
  }
  if (!LittleFS.exists("/TailCurrent.txt")) {
    writeFile(LittleFS, "/TailCurrent.txt", String(TailCurrent_Scaled).c_str());
  } else {
    TailCurrent_Scaled = readFile(LittleFS, "/TailCurrent.txt").toInt();
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
  if (!LittleFS.exists("/AlarmLatchEnabled.txt")) {
    writeFile(LittleFS, "/AlarmLatchEnabled.txt", String(AlarmLatchEnabled).c_str());
  } else {
    AlarmLatchEnabled = readFile(LittleFS, "/AlarmLatchEnabled.txt").toInt();
  }

  if (!LittleFS.exists("/bulkCompleteTime.txt")) {
    writeFile(LittleFS, "/bulkCompleteTime.txt", String(bulkCompleteTime).c_str());
  } else {
    bulkCompleteTime = readFile(LittleFS, "/bulkCompleteTime.txt").toInt();
  }
  if (!LittleFS.exists("/FLOAT_DURATION.txt")) {
    writeFile(LittleFS, "/FLOAT_DURATION.txt", String(FLOAT_DURATION).c_str());
  } else {
    FLOAT_DURATION = readFile(LittleFS, "/FLOAT_DURATION.txt").toInt();
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
    }
    lastConsoleMessageTime = now11;
  }
}
float getBatteryVoltage() {
  static unsigned long lastWarningTime = 0;
  const unsigned long WARNING_INTERVAL = 10000;  // 10 seconds between warnings
  float selectedVoltage = 0;
  switch (BatteryVoltageSource) {
    case 0:  // INA228
      if (INADisconnected == 0 && !isnan(IBV) && IBV > 8.0 && IBV < 70.0 && millis() > 5000) {
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

  return selectedVoltage;
}

float getTargetAmps() {

  float targetValue = 0;

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

  return targetValue;
}

int thermistorTempC(float V_thermistor) {
  float Vcc = 5.0;
  float R_thermistor = R_fixed * (V_thermistor / (Vcc - V_thermistor));
  float T0_K = T0_C + 273.15;
  float tempK = 1.0 / ((1.0 / T0_K) + (1.0 / Beta) * log(R_thermistor / R0));
  return (int)(tempK - 273.15);  // Cast to int for whole degrees
}

void checkWiFiConnection() {
  static unsigned long lastWiFiCheckTime = 0;
  static bool reconnecting = false;

  if (currentWiFiMode == AWIFI_MODE_CLIENT && WiFi.status() != WL_CONNECTED) {
    if (reconnecting) return;  // Prevent overlapping reconnection attempts

    if (millis() - lastWiFiCheckTime > 5000) {  // Check every 5 seconds
      lastWiFiCheckTime = millis();
      reconnecting = true;

      Serial.println("WiFi connection lost. Attempting to reconnect...");

      String saved_ssid = readFile(LittleFS, WIFI_SSID_FILE);
      String saved_password = readFile(LittleFS, WIFI_PASS_FILE);

      if (connectToWiFi(saved_ssid.c_str(), saved_password.c_str(), 2000)) {
        Serial.println("Reconnected to WiFi!");
        queueConsoleMessage("Reconnected to WiFi!");
      } else {
        Serial.println("Failed to reconnect. Will try again in 5 seconds.");
      }

      reconnecting = false;
    }
  }
}
bool setupDisplay() {
  // Add delay for ESP32 stabilization
  delay(100);

  try {
    // Initialize SPI carefully
    SPI.begin();
    delay(50);                  // Let SPI settle
    SPI.setFrequency(1000000);  // Start slow for stability
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    // Test if SPI is working by trying a simple transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();

    // Initialize U8g2 with error checking
    u8g2.begin();

    // Test if display is responding
    u8g2.clearBuffer();
    u8g2.sendBuffer();

    // If we get here without crashing, it's working
    displayAvailable = true;
    Serial.println("Display initialized successfully");
    queueConsoleMessage("Display initialized successfully");

    return true;

  } catch (...) {
    Serial.println("Display initialization failed - exception caught");
    queueConsoleMessage("Display initialization failed - exception caught");
    displayAvailable = false;
    return false;
  }
}

void CheckAlarms() {
  static unsigned long lastRunTime = 0;
  if (millis() - lastRunTime < 250) return;  // Only run every 250ms
  lastRunTime = millis();

  static bool previousAlarmState = false;
  bool currentAlarmCondition = false;
  bool outputAlarmState = false;
  String alarmReason = "";

  // Handle alarm test - this can trigger regardless of AlarmActivate
  if (AlarmTest == 1) {
    if (alarmTestStartTime == 0) {
      alarmTestStartTime = millis();
      queueConsoleMessage("ALARM TEST: Testing buzzer for 2 seconds");
    }

    if (millis() - alarmTestStartTime < ALARM_TEST_DURATION) {
      currentAlarmCondition = true;  // Treat test as a real alarm condition
      alarmReason = "Alarm test active";
    } else {
      // Test completed, reset
      AlarmTest = 0;
      alarmTestStartTime = 0;
      queueConsoleMessage("ALARM TEST: Complete");
    }
  }

  // Normal alarm checking - ONLY when AlarmActivate is enabled
  if (AlarmActivate == 1) {
    // Set TempToUse based on source (ADD THIS)
    if (TempSource == 0) {
      TempToUse = AlternatorTemperatureF;
    } else if (TempSource == 1) {
      TempToUse = temperatureThermistor;
    }
    // Temperature alarm
    if (TempAlarm > 0 && TempToUse > TempAlarm) {
      currentAlarmCondition = true;
      alarmReason = "High alternator temperature: " + String(TempToUse) + "°F (limit: " + String(TempAlarm) + "°F)";
    }

    // Voltage alarms
    float currentVoltage = getBatteryVoltage();
    if (VoltageAlarmHigh > 0 && currentVoltage > VoltageAlarmHigh) {
      currentAlarmCondition = true;
      alarmReason = "High battery voltage: " + String(currentVoltage, 2) + "V (limit: " + String(VoltageAlarmHigh) + "V)";
    }

    if (VoltageAlarmLow > 0 && currentVoltage < VoltageAlarmLow && currentVoltage > 8.0) {
      currentAlarmCondition = true;
      alarmReason = "Low battery voltage: " + String(currentVoltage, 2) + "V (limit: " + String(VoltageAlarmLow) + "V)";
    }

    // Current alarm (alternator)
    if (CurrentAlarmHigh > 0 && MeasuredAmps > CurrentAlarmHigh) {
      currentAlarmCondition = true;
      alarmReason = "High alternator current: " + String(MeasuredAmps, 1) + "A (limit: " + String(CurrentAlarmHigh) + "A)";
    }
    // Current alarm (battery)
    if (MaximumAllowedBatteryAmps > 0 && (Bcur) > MaximumAllowedBatteryAmps) {
      currentAlarmCondition = true;
      alarmReason = "High battery current: " + String(abs(Bcur), 1) + "A (limit: " + String(MaximumAllowedBatteryAmps) + "A)";
    }
  }

  // Handle manual latch reset
  if (ResetAlarmLatch == 1) {
    alarmLatch = false;
    ResetAlarmLatch = 0;
    queueConsoleMessage("ALARM LATCH: Manually reset");
  }

  // Handle latching logic (applies to ALL alarm conditions including test)
  if (AlarmLatchEnabled == 1) {
    // Latch mode: Once alarm trips, stays on until manually cleared
    if (currentAlarmCondition) {
      alarmLatch = true;  // Set latch
    }
    outputAlarmState = alarmLatch;  // Output follows latch state
  } else {
    // Non-latch mode: Output follows current conditions
    outputAlarmState = currentAlarmCondition;
  }

  // Final output control - respects AlarmActivate EXCEPT for test
  bool finalOutput = false;
  if (AlarmTest == 1) {
    // Test always works regardless of AlarmActivate
    finalOutput = outputAlarmState;
  } else if (AlarmActivate == 1) {
    // Normal alarms only work when AlarmActivate is on
    finalOutput = outputAlarmState;
  }
  // else: AlarmActivate is off and not testing = no output

  digitalWrite(33, finalOutput ? HIGH : LOW);  // get rid of this bs
  // digitalWrite(33, finalOutput);     // go back to this later
  // Console messaging
  if (currentAlarmCondition != previousAlarmState) {
    if (currentAlarmCondition) {
      queueConsoleMessage("ALARM ACTIVATED: " + alarmReason);
    } else if (AlarmLatchEnabled == 0) {
      queueConsoleMessage("ALARM CLEARED");
    }
    previousAlarmState = currentAlarmCondition;
  }
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {  // Every 5 seconds
    lastDebugTime = millis();
    if (AlarmActivate == 1) {
      // queueConsoleMessage("ALARM DEBUG: GPIO33=" + String(digitalRead(33)) + ", AlarmActivate=" + String(AlarmActivate) + ", TempAlarm=" + String(TempAlarm) + ", CurrentTemp=" + String(TempSource == 0 ? AlternatorTemperatureF : temperatureThermistor));
    }
  }
}
void logDashboardValues() {
  static unsigned long lastDashboardLog = 0;
  if (millis() - lastDashboardLog >= 10000) {  // Every 10 seconds
    lastDashboardLog = millis();
    String dashboardLog = "DASHBOARD: ";
    dashboardLog += "IBV=" + String(IBV, 2) + "V ";
    dashboardLog += "SoC=" + String(SoC_percent / 100) + "% ";
    dashboardLog += "AltI=" + String(MeasuredAmps, 1) + "A ";
    dashboardLog += "BattI=" + String(Bcur, 1) + "A ";
    dashboardLog += "AltT=" + String((int)AlternatorTemperatureF) + "°F ";
    dashboardLog += "RPM=" + String((int)RPM);
    queueConsoleMessage(dashboardLog);
  }
}

void updateChargingStage() {
  float currentVoltage = getBatteryVoltage();

  if (inBulkStage) {
    // Currently in bulk charging
    ChargingVoltageTarget = FullChargeVoltage;

    if (currentVoltage >= ChargingVoltageTarget) {
      if (bulkCompleteTimer == 0) {
        bulkCompleteTimer = millis();                                // Start timer
      } else if (millis() - bulkCompleteTimer > bulkCompleteTime) {  // 1 second above bulk voltage
        inBulkStage = false;
        floatStartTime = millis();
        queueConsoleMessage("CHARGING: Bulk stage complete, switching to float");
      }
    } else {
      bulkCompleteTimer = 0;  // Reset timer if voltage drops
    }
  } else {
    // Currently in float charging
    ChargingVoltageTarget = TargetFloatVoltage;

    // Return to bulk after time expires OR if voltage drops significantly
    if ((millis() - floatStartTime > FLOAT_DURATION * 1000) || (currentVoltage < TargetFloatVoltage - 0.5)) {
      inBulkStage = true;
      bulkCompleteTimer = 0;
      floatStartTime = millis();
      queueConsoleMessage("CHARGING: Returning to bulk stage");
    }
  }
}

bool checkFactoryReset() {
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
  delay(100);  // Let pin settle

  if (digitalRead(FACTORY_RESET_PIN) == LOW) {  // GPIO35 shorted to GND
    Serial.println("FACTORY RESET: GPIO35 detected shorted to GND");

    // Delete ALL settings files
    LittleFS.remove(WIFI_MODE_FILE);
    LittleFS.remove("/ssid.txt");
    LittleFS.remove("/pass.txt");
    LittleFS.remove(AP_PASSWORD_FILE);
    LittleFS.remove("/password.txt");
    LittleFS.remove("/password.hash");
    // Reset variables to defaults
    permanentAPMode = 0;
    esp32_ap_password = "alternator123";  // Back to default
    Serial.println("FACTORY RESET: All settings cleared, entering factory config mode");
    queueConsoleMessage("FACTORY RESET: All settings cleared via hardware reset");
    return true;  // Factory reset performed
  }
  return false;  // Normal boot
}
void loadESP32APPassword() {
  // Load AP password
  if (LittleFS.exists(AP_PASSWORD_FILE)) {
    String savedPassword = readFile(LittleFS, AP_PASSWORD_FILE);
    savedPassword.trim();
    if (savedPassword.length() > 0) {
      esp32_ap_password = savedPassword;
      Serial.println("ESP32 AP password loaded from storage: length=" + String(savedPassword.length()));
    } else {
      Serial.println("Stored AP password is empty, keeping current value: " + esp32_ap_password);
      // Don't overwrite - keep whatever was set during configuration
    }
  } else {
    Serial.println("No stored AP password, using default");
    esp32_ap_password = "alternator123";
  }

  // Load custom SSID
  if (LittleFS.exists(AP_SSID_FILE)) {
    String savedSSID = readFile(LittleFS, AP_SSID_FILE);
    savedSSID.trim();
    if (savedSSID.length() > 0) {
      esp32_ap_ssid = savedSSID;
      Serial.println("ESP32 AP SSID loaded from storage: " + esp32_ap_ssid);
    } else {
      Serial.println("Stored AP SSID is empty, keeping current value: " + esp32_ap_ssid);
      // Don't overwrite - keep whatever was set during configuration
    }
  } else {
    Serial.println("No stored AP SSID, using default");
    esp32_ap_ssid = "ALTERNATOR_WIFI";
  }
}
bool ensureLittleFS() {
  if (littleFSMounted) {
    return true;
  }
  Serial.println("Initializing LittleFS...");
  if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) {
    Serial.println("CRITICAL: LittleFS mount failed! Attempting format...");
    if (!LittleFS.begin(true)) {
      Serial.println("CRITICAL: LittleFS format failed - filesystem unavailable");
      littleFSMounted = false;
      return false;
    } else {
      Serial.println("LittleFS formatted and mounted successfully");
      littleFSMounted = true;
      return true;
    }
  } else {
    Serial.println("LittleFS mounted successfully");
  }
  littleFSMounted = true;
  return true;
}
void setupCaptivePortalLanding() {
  Serial.println("Setting up captive portal landing...");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Much smaller landing page to reduce memory usage
    String page = "<!DOCTYPE html><html><head><title>Alternator Regulator Connected</title>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<style>";
    page += "body{font-family:Arial;text-align:center;padding:20px;background:#f5f5f5;line-height:1.6}";
    page += ".card{background:white;padding:24px;border-left:4px solid #ff6600;border-radius:8px;max-width:500px;margin:0 auto;box-shadow:0 2px 4px rgba(0,0,0,0.1)}";
    page += "h1{color:#333;margin-bottom:1rem;font-size:24px}";
    page += ".success-box{background:#d4edda;border:1px solid #c3e6cb;color:#155724;padding:16px;border-radius:8px;margin:20px 0}";
    page += ".big-button{display:inline-block;background:#ff6600;color:white;padding:16px 32px;text-decoration:none;border-radius:8px;font-weight:bold;font-size:18px;margin:20px 0}";
    page += ".big-button:hover{background:#e65c00}";
    page += ".bookmark-info{background:#f8f9fa;border:1px solid #dee2e6;padding:12px;border-radius:8px;margin:16px 0;font-size:14px}";
    page += ".ip-address{font-family:monospace;font-size:20px;font-weight:bold;color:#ff6600;background:#f8f9fa;padding:8px 12px;border-radius:8px;display:inline-block;margin:8px 0}";
    page += "</style></head><body>";
    page += "<div class='card'>";
    page += "<h1>Alternator Regulator</h1>";
    page += "<div class='success-box'><strong>Successfully Connected!</strong><br>You are now connected to the alternator regulator's WiFi network.</div>";
    page += "<p>Access the full alternator control interface:</p>";
    page += "<a href='http://192.168.4.1' class='big-button'>Open Alternator Interface</a>";
    page += "<div class='bookmark-info'><strong>For easy future access:</strong><br>Bookmark this address: <span class='ip-address'>192.168.4.1</span></div>";
    page += "<p style='margin-top:24px;font-size:14px;color:#666'><strong>Network:</strong> ALTERNATOR_WIFI<br><strong>Device IP:</strong> 192.168.4.1</p>";
    page += "</div></body></html>";
    request->send(200, "text/html", page);
  });
  server.begin();
  Serial.println("Landing page ready");
}
void calculateChargeTimes() {
  if (targetCurrent > 0.01) {  // charging
    float remainingAh = BatteryCapacity_Ah * (100.0 - SoC_percent) / 100.0;
    timeToFullChargeMin = (int)(remainingAh / targetCurrent * 60.0);
    timeToFullDischargeMin = -999;
  } else if (targetCurrent < -0.01) {  // discharging
    float remainingAh = BatteryCapacity_Ah * SoC_percent / 100.0;
    timeToFullDischargeMin = (int)(remainingAh / (-targetCurrent) * 60.0);
    timeToFullChargeMin = -999;
  } else {
    timeToFullChargeMin = -999;
    timeToFullDischargeMin = -999;
  }
}
void StuffToDoAtSomePoint() {
  //every reset button has a pointless flag and an echo.  I did not delete them for fear of breaking the payload and they hardly cost anything to keep
  //Battery Voltage Source drop down menu- make this text update on page re-load instead of just an echo number
  // Is it possible to power up if the igniton is off?  A bunch of setup functions may fail?
  // What happens when rpm table is screwed up by an idiot
}
