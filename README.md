# AltRegESP32
Open Source Alternator Regulator for Lithium and other battery types

For additional discussion, see:
www.xengineering.net
and
https://www.cruisersforum.com/forums/f14/open-source-arduino-alternator-regulator-282939.html

Contact me at mark@xengineering.net 

Folder descriptions:

Regulator Programs ----- contains drafts of the code to run on ESP32.  Choose the newest .INOs for most complete functionality.  These the ones that goes onto the ESP32/ESPDuino.  Also needed is the newest HTML file, which must be uploaded to ESP32's LittleFS (flash memory).  Tutorial to do that was here: https://randomnerdtutorials.com/arduino-ide-2-install-esp32-littlefs/#installing-MAC  but is likely to become a broken link soon.  Best bet is to google "how to upload files to ESP32 Littel FS" and use the latest and greatest methods.
Working files as of April 15 2025 are "Index.html" (.html file) and "April132025" (.INO) and "2_Functions" (another .INO).    The 2nd file contains all functions, just a way of improving organization a little.   The two files must both be stored in a folder called April132025 located within the Arduino folder.  Open through Arduino IDE version 2 and they should both show up as tabs.  Both will compile and upload to ESP32 together automatically.   If this fails, you can just paste all the functions into the bottom of April132025.  This might be easier- the Arduino IDE is messy and unrelaible unless things are just right.

ComponentDebuggingNotes ------  smaller programs that were used for debugging specific features- probably not of any use to anyone.




