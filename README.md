# SAMCO Fork - Arduino Powered IR Light Gun

For general information on SAMCO, visit the original author YouTube channel https://www.youtube.com/c/samcorulz.

The SAMCO is a Namco light gun with the insides replaced with an Arduino micro controller and DF Robot IR positioning camera and works as HID mouse on LCD (flat screen) TV/monitors.  
This fork works with no calibration (but you currently need to hardcode the position of the sensor bar and the monitor dimensions) and also supports a native Guncon mode that works with the PS2.

Parts:  
- Adafruit ItsyBitsy 32u4 (3V or 5V), M0 & M4  
- DF Robot IR positioning camera  
- Wii USB sensor bar (not dolphin bar, dolphin bar needs wiimote to turn it on)

Notes if you are using the SAMCO PCB:
- The C/Reload button is wired to pin 13, but pin 13 is also wired to the on-board LED which makes it unreliable with internal pull-ups. The Arduino documentaiton recommends using an external pull-down. Or simply rewire the button to a different pin (I am using A5).
- The optional pedal pin is wired to pin 5, but pin 5 is output only on Adafruit ItsyBitsy 32u4 (3V), M0 & M4.
- This fork has only been tested on Adafruit ItsyBitsy M4. It is pretty heavy on floating point operations. I would expect it to perform pretty badly on an Adafruit ItsyBitsy 32u4 which does not have a FPU.

In this project I've used modified versions of various libraries:
- https://github.com/adafruit/Adafruit_TinyUSB_Arduino
- https://github.com/DFRobot/DFRobotIRPosition  
- https://github.com/hathach/tinyusb
- https://github.com/jonathanedgecombe/absmouse
- https://sourceforge.net/projects/wiiwhiteboard/

If you haven't brought a PCB but enjoyed this code please consider making a small donation to help the original author continue developing and supplying support for this project https://www.paypal.me/sammywizbang.
