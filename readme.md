# Radio Demo Code Repo
This contains the code I used to remotely drive a small car using a joystick and 900 MHz radio. If you use this code, please change the frequency to 915 MHz and remove mentions to "KN4FVI" (my amateur radio license number).

Each part of the code is split into 2 parts: computerSide and robotSide. ComputerSide is designed to run on an Adafruit Feather with radio built in, and robotSide is designed to run on a Teensy 4.0 with the external radio module from Adafruit.

Code labeled under "test" is used to demo radio communication and requires no accessories. Code labeled "drive" is the full code for controlling the car. If you wish to use it without modification, you will need a 7-segment display, joystick, RGB LED, two motors, and an h-bridge.

Check out my website for more information!
