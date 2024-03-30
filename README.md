# AltRegESP32
Open Source Alternator Regulator for Lithium and other battery types

For additional discussion, see:
www.xengineering.net
and
https://www.cruisersforum.com/forums/f14/open-source-arduino-alternator-regulator-282939.html

Contact me at mark@xengineering.net 

File descriptions:
AltforESP32b ----- this is an incomplete first draft of the software to run on ESP32.  This is somewhat useless right now because it requires a lot of hardware to test, and the main algorithm is not pasted in yet (work in progress)
<img width="586" alt="image" src="https://github.com/markliquid1/AltRegESP32/assets/139247086/20c75e36-946e-445e-ade6-ef993da9e146">


UserInterface------  This is the beginning of a user interface to configure and display sensor readings from the Regulator.  It should run on any ESP32 if you change the two lines noted to specify your own Wifi network and password.   Once the ESP32 is programmed, get the IP address from the serial monitor, then type it into your web browser.  You should find a result like this:
<img width="574" alt="image" src="https://github.com/markliquid1/AltRegESP32/assets/139247086/6e010dc7-424c-4e6e-9d16-19d61927aeff">



(I'm using Chrome, other browsers may or may not work)




