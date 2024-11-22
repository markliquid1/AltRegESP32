#include "SiC45x.h"

SiC45x sic45x(0x1D);
  
float vout = 4;

void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT);  // This pin is used to provide a high signal to SiC450 Enable pin
  digitalWrite(4, HIGH);  // Enable the SIC450
  sic45x.begin();
  sic45x.sendClearFaults();
  sic45x.setFrequencySwitch(1000); //range is 300 kHz to 1500 kHz, resolution is 50 kHz,
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);

  //sic45x.setOperation(ON_OFF_DISABLED | OFFB_IMMEDIATE | MARGIN_COMMAND | MRGNFLT_FOLLOW);

  //sic45x.setVoutOvFaultResponse(VOUT_OV_FAULT_RESPONSE_B);
  //sic45x.setVoutUvFaultResponse(VOUT_UV_FAULT_RESPONSE_B);

}

void loop() {

 Serial.print("Please enter voltage target: ");
  while (Serial.available() == 0) {
}
  vout = Serial.parseFloat();

  Serial.println();
  Serial.println();

  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);

//The range of settings allowed for each register varies stupidly with this chip, must stay within the below limits:
  //The VIN_OV_FAULT_LIMIT,VIN_UV_WARN_LIMIT  range is 1 V to 80 V, resolution is 0.5 V
  //output voltage’s range is 0.3 V to 14 V
  ///same for vout, margin high, margin low,VOUT_OV_FAULT_LIMIT,VOUT_OV_WARN_LIMIT...
  //The POWER_GOOD_ON and POWER_GOOD_OFF range is 0.24 V to 14 V
  //0V to 14V for VOUT_UV_WARN_LIMIT ,VOUT_UV_FAULT_LIMIT
  //The VIN_ON and VIN_OFF range is 1 V to 80 V, resolution is 0.5 V

  sic45x.setVinOvFaultLimit(60);
  sic45x.setPowerGoodOn(0.24); // .9
  sic45x.setPowerGoodOff(0.24); // .85
  sic45x.setVoutOvFaultLimit(14);   // 115 The default value for this was very low according to datasheet, so I raise it here
  sic45x.setVoutOvWarnLimit(14); // 110
  sic45x.setVoutUvWarnLimit(0); // .9
  sic45x.setVoutUvFaultLimit(0); // .8
  sic45x.setVoutMarginLow(0.3); //.95
  sic45x.setVoutMarginHigh(0.3); //105

  sic45x.setVoutCommand(vout);  // Any voltage here produces a similar result-  messy waveforms with average <1 volt

  // sic45x.printStatusWord();
  // sic45x.printStatusVout();
  // sic45x.printStatusIout();
  // sic45x.printStatusInput();
  // sic45x.printStatusCml();
  Serial.println();
  Serial.print("getReadVin:");
  Serial.println(sic45x.getReadVin());
  Serial.print("getReadTemperature:");
  Serial.println(sic45x.getReadTemperature());
  Serial.print("getReadDutyCycle:");
  Serial.println(sic45x.getReadDutyCycle());
  Serial.print("getReadIout:");
  Serial.println(sic45x.getReadIout());


  delay(5000);
}