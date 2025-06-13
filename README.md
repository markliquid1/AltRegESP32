# AltRegESP32
Open Source Alternator Regulator for Lithium and other battery types

For additional discussion, see:
www.xengineering.net
and
https://www.cruisersforum.com/forums/f14/open-source-arduino-alternator-regulator-282939.html

Contact me at mark@xengineering.net 

Folder descriptions:
--------------------------------------------------------------------------
Regulator Programs ----- contains drafts of the code to run on ESP32.  

There are 3 files involved

1) The two .INO's- compile them together with Arduino IDE version 2 and upload onto the ESP32/ESPDuino.  MainProgram is self explanatory.  2_Functions contains all functions, just a way of improving organization a little.   Both files must both be stored in a folder called MainProgram located within the Arduino folder.  Open through Arduino IDE version 2 and they should show up as tabs.  Both will compile and upload to ESP32 together automatically.   If this fails, you can just paste all the functions into the bottom of MainProgram.  This might be easier- the Arduino IDE is messy and unrelaible at compiling from multiple files unless things are just right.

2) .HTML file- flash to ESP32's LittleFS (flash memory).  Tutorial to do that was here: https://randomnerdtutorials.com/arduino-ide-2-install-esp32-littlefs/#installing-MAC  but is likely to become a broken link at some point.  Best bet is to google "how to upload files to ESP32 Littel FS" and use the latest and greatest methods.

---------------------------------------------------------------------------
ComponentDebuggingNotes ------  smaller programs that were used for debugging specific features- probably not of any use to anyone.


Notes on Code Philosophy:
--------------------------------------------------------------------------
In order to minimally discourage inexperienced programmers, this code is not split up into .h and .cpp files, multiple HTML files, etc, which would have greatly improved organization.  The penalty to pay is long programs.  I find it to be an ok tradeoff because it keeps the code recognizeable to beginners. Update: another (lucky, unintentional, huge) benefit is that it's easier to copy and paste these large files into AI chats.  AI (mostly Claude, a little ChatGPT) wrote 90% of this code.  Absolutely incredible what it can do in 2025, and the future will be even better.
