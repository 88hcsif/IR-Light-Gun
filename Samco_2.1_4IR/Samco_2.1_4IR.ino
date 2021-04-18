/*
 * Samco 2.1
 * Pull the trigger while powering up the lightgun to enter Guncon mode.
 *
 * Copyright 2021 88hcsif
 *
 * Main function taken from Adafruit_TinyUSB_SAMD:
 * Copyright (c) 2019, hathach for Adafruit
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Arduino.h>
#include <DFRobotIRPosition.h>
#include <Perspective.h>
#include <Samco.h>
#include <TinyUSB_AbsMouse_and_Keyboard.h>

DFRobotIRPosition myDFRobotIRPosition;
Perspective per;
Samco mySamco;

#define DEBOUNCE_DELAY 5

int currState[11];
int prevState[11];
int lastReportedState[11];
unsigned long lastDebounceTime[11];
int pinMap[] = {7, 11, 9, 10, 12, A1, A0, A2, A3, A5, 4};
int keyMap[] = {
  -MOUSE_LEFT,     // TRIGGER
  KEY_UP_ARROW,    // DPAD UP
  KEY_DOWN_ARROW,  // DPAD DOWN
  KEY_LEFT_ARROW,  // DPAD LEFT
  KEY_RIGHT_ARROW, // DPAD RIGHT
  'a',             // A
  'b',             // B
  KEY_RETURN,      // START
  KEY_BACKSPACE,   // SELECT
  -MOUSE_RIGHT,    // RELOAD / C
  -MOUSE_RIGHT     // PEDAL
};

#define DEBUG_FPS 0
#if DEBUG_FPS
#define FPS_CHECK 2000
int count = 0;
unsigned long lastFps = 0;
#endif

#define GUNCON_SUPPORT 1 && (defined USE_TINYUSB)
#if GUNCON_SUPPORT
#include <Guncon2Interface.h>

uint16_t gunconMap[] = {
  GUNCON_TRIGGER,
  GUNCON_DPAD_UP,
  GUNCON_DPAD_DOWN,
  GUNCON_DPAD_LEFT,
  GUNCON_DPAD_RIGHT,
  GUNCON_A,
  GUNCON_B,
  GUNCON_START,
  GUNCON_SELECT,
  GUNCON_C,
  GUNCON_C
};

bool isGuncon = false;
int gunconPin = 7; // TRIGGER

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

// Initialize C library
extern "C" void __libc_init_array(void);

/*
 * Main entry point of Arduino application
 * Need to redefine the main functionto stop the Adafruit implementantion from
 * adding a serial interface to the USB device. Also it is better to add USB
 * interfaces before we allow the USB host to poll.
 */
int main( void ) {
  init();
  __libc_init_array();
  initVariant();
  delay(1);

  pinMode(gunconPin, INPUT_PULLUP);
  isGuncon = (digitalRead(gunconPin) == LOW);

  if (!isGuncon) {
    // Adafruit_TinyUSB_Core_init();
    Serial.setStringDescriptor("TinyUSB Serial");
    USBDevice.addInterface(Serial);
    USBDevice.setID(USB_VID, USB_PID);
    USBDevice.begin();
  } else {
    Guncon.begin();
    USBDevice.setID(0x0b9a, 0x016a);
    USBDevice.begin();
  }

  usb_hardware_init();
  tusb_init(); // Init tinyusb stack
  setup();

  for (;;) {
    loop();
    yield(); // yield run usb background task

    if (serialEventRun) serialEventRun();
  }

  return 0;
}

// run TinyUSB background task when yield()
extern  "C" void yield(void)
{
  tud_task();
  tud_cdc_write_flush();
}

// Init usb hardware when starting up. Softdevice is not enabled yet
static void usb_hardware_init(void)
{
#ifdef PIN_LED_TXL
//	txLEDPulse = 0;
	pinMode(PIN_LED_TXL, OUTPUT);
	digitalWrite(PIN_LED_TXL, HIGH);
#endif

#ifdef PIN_LED_RXL
//	rxLEDPulse = 0;
	pinMode(PIN_LED_RXL, OUTPUT);
	digitalWrite(PIN_LED_RXL, HIGH);
#endif

	/* Enable USB clock */
#if defined(__SAMD51__)
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
	MCLK->AHBMASK.reg |= MCLK_AHBMASK_USB;

	// Set up the USB DP/DN pins
	PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA24H_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24H_USB_DM & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA24H_USB_DM/2].reg |= MUX_PA24H_USB_DM << (4 * (PIN_PA24H_USB_DM & 0x01u));
	PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA25H_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25H_USB_DP & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA25H_USB_DP/2].reg |= MUX_PA25H_USB_DP << (4 * (PIN_PA25H_USB_DP & 0x01u));


	GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

	NVIC_SetPriority(USB_0_IRQn, 0UL);
	NVIC_SetPriority(USB_1_IRQn, 0UL);
	NVIC_SetPriority(USB_2_IRQn, 0UL);
	NVIC_SetPriority(USB_3_IRQn, 0UL);
#else
	PM->APBBMASK.reg |= PM_APBBMASK_USB;

	// Set up the USB DP/DN pins
	PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
	PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));

	// Put Generic Clock Generator 0 as source for Generic Clock Multiplexer 6 (USB reference)
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(6)     | // Generic Clock Multiplexer 6
	GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
	GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY)
	;

	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
#endif
}

#else
const bool isGuncon = false;
#endif /* (defined USE_TINYUSB) && GUNCON_SUPPORT */

void setup() {
  if (!isGuncon) {
    AbsMouse.init(1000, 1000, false);
#ifdef USE_TINYUSB
    Keyboard.init(false);
#endif
    Keyboard.begin();
#if DEBUG_FPS
    Serial.begin(9600);
#endif
  } else {
#if GUNCON_SUPPORT
    Guncon.init(1000, 1000, false);
#endif
  }

  for (int i = 0; i < 11; i++) {
    pinMode(pinMap[i], INPUT_PULLUP);
    currState[i] = HIGH;
    prevState[i] = HIGH;
    lastReportedState[i] = HIGH;
  }

  myDFRobotIRPosition.begin();
}

void loop() {
  bool mouseChanged = false;
  bool keyboardChanged = false;
  bool gunconChanged = false;

  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.readPosition()) {
    if (mySamco.track(
          myDFRobotIRPosition.getX(0), myDFRobotIRPosition.getY(0),
          myDFRobotIRPosition.getX(1), myDFRobotIRPosition.getY(1),
          myDFRobotIRPosition.getX(2), myDFRobotIRPosition.getY(2),
          myDFRobotIRPosition.getX(3), myDFRobotIRPosition.getY(3))) {
      per.warp(
          mySamco.getX(0), mySamco.getY(0),
          mySamco.getX(1), mySamco.getY(1),
          mySamco.getX(2), mySamco.getY(2),
          mySamco.getX(3), mySamco.getY(3));
      if (!isGuncon) {
        if (0 <= per.getX() && per.getX() <= 1000 && 0 <= per.getY() && per.getY() <= 1000) {
          mouseChanged = true;
          AbsMouse.move(per.getX(), per.getY());
        }
      } else {
#if GUNCON_SUPPORT
        int x = per.getX();
        int y = per.getY();
        if (0 <= per.getX() && per.getX() <= 1000 && 0 <= per.getY() && per.getY() <= 1000) {
          Guncon.move(x, y);
        } else {
          Guncon.reset();
        }
#endif
      }
    }
  }
  unsigned long now = millis();
  for (int i = 0; i < 11; i++) {
    currState[i] = digitalRead(pinMap[i]);
    if (currState[i] != lastReportedState[i]) {
      if (currState[i] != prevState[i]) {
        // reset the debouncing timer
        lastDebounceTime[i] = now;
      }
      if ((now - lastDebounceTime[i]) >= DEBOUNCE_DELAY) {
        if (!isGuncon) {
          if (keyMap[i] < 0) {
            mouseChanged = true;
            if (currState[i] == LOW) {
              AbsMouse.press(-keyMap[i]);
            } else {
              AbsMouse.release(-keyMap[i]);
            }
          } else {
            // TODO: Investigate why keyboard only works when mouse not active (seen with TinyUSB)
            keyboardChanged = true;
            if (currState[i] == LOW) {
              Keyboard.press(keyMap[i]);
            } else {
              Keyboard.release(keyMap[i]);
            }
          }
        } else {
#if GUNCON_SUPPORT
          gunconChanged = true;
          if (currState[i] == LOW) {
            Guncon.press(gunconMap[i]);
          } else {
            Guncon.release(gunconMap[i]);
          }
#endif
        }
        lastReportedState[i] = currState[i];
      }
    }
    prevState[i] = currState[i];
  }
  if (mouseChanged) {
    AbsMouse.report();
  }
  if (keyboardChanged) {
#ifdef USE_TINYUSB
    Keyboard.report();
#endif
  }
  if (isGuncon) {
#if GUNCON_SUPPORT
    Guncon.report();
#endif
  }
#if DEBUG_FPS
  unsigned long here = millis();
  count++;
  if (here - lastFps >= FPS_CHECK) {
    float fps = ((float)count/(here - lastFps))*1000;
    Serial.println(fps);
    lastFps = here;
    count = 0;
  }
#endif
}
