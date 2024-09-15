  int pinB=2;


void setup() {
  // put your setup code here, to run once:
  pinMode(pinB, OUTPUT);     // This pin is used to provide a high signal to SiC450 Enable pin

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(pinB, HIGH);  // Enable the SIC450
delay(500);
    digitalWrite(pinB, LOW);  // Enable the SIC450

delay(10000);

}
