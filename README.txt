eLoom: an automation system for the Glimakra "Julia" floor loom
 
Tying up a loom with treadles and lamms is complicated, and so is the 
weaving. I have replaced all the treadles and lamms in our Glimakra 
"Julia" countermarche floor loom with stepper motors that independently 
operate up to 8 shafts. That not only makes setup and weaving easier, 
but it also allows patterns that would otherwise not be possible. 

The hardware added to the loom consists of:
 - up to 8 stepper motors, 23HS22-1504S, NEMA 23, 5.4V, 1.5A, 164 oz-in holding torque
 - up to 8 DRV8825 stepper motor drivers (Pololu or equiv.)
 - a 24VDC 200W power supply
 - 25-pitch sprokets and roller chain to connect the motors to the shaft lines
 - a control box that contains:
   - 1 4-row x 20-character LCD display (Adafruit 198)
   - 1 4-digit 7-segment I2C big digit numerical display (Adafruit 1268)
   - 2 rotary encoders with RGB LEDs and pushbuttons (Sparkfun COM-10982 and BOB-11722)
   - 4 general-purpose pushbuttons (MPJA 32730 or equiv.)
   - 1 attached foot pedal (MPJA 18150 or equiv.)
   - 1 Teensy 3.5 microprocessor, with 512KB flash, 192KB RAM, 4KB EEPROM, and 40 I/O pins