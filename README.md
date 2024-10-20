# AltRegESP32
Open Source Alternator Regulator for Lithium and other battery types

For additional discussion, see:
www.xengineering.net
and
https://www.cruisersforum.com/forums/f14/open-source-arduino-alternator-regulator-282939.html

Contact me at mark@xengineering.net 

File descriptions:

sketch_oct6_.ino ----- this is a working first draft of the software to run on ESP32.    There are two remaining hardware problems: SiC450 buck regulator causing noise on 12V power line, and ISO1050 CAN chip is outputting a strange squarewave that's not recognized by the ESP32 CAN controller.  Both may be hardware rather than software issues, and otherwise, this code works well.

UserInterface.ino ------  This is the beginning of a user interface to configure and display sensor readings from the Regulator.  It should run on any ESP32 if you change the two lines noted to specify your own Wifi network and password.   Once the ESP32 is programmed, get the IP address from the serial monitor, then type it into your web browser.  You should find a result like this:
<img width="574" alt="image" src="https://github.com/markliquid1/AltRegESP32/assets/139247086/6e010dc7-424c-4e6e-9d16-19d61927aeff">


(I'm using Chrome, other browsers may or may not work)




