# AltRegESP32
Open Source Alternator Regulator for Lithium and other battery types

For additional discussion, see:
www.xengineering.net
and
https://www.cruisersforum.com/forums/f14/open-source-arduino-alternator-regulator-282939.html

File descriptions:
AltforESP32b ----- this is an incomplete first draft of the software to run on ESP32.  This is somewhat useless right now because it requires a lot of hardware to test

UserInterface------  This is the beginning of a user interface to configure and display sensor readings from the Regulator.  It should run on any ESP32 if you change the two lines noted to specify your own Wifi network and password.   Once the ESP32 is programmed, get the IP address from the serial monitor, then type it into your web browser.  You should find a result like this:
<img width="543" alt="image" src="https://github.com/markliquid1/AltRegESP32/assets/139247086/b612e7e0-d308-43a9-a9d8-d44f8c1563a6">


(I'm using Chrome, other browsers may or may not work)




