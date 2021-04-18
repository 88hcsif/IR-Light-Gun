/*
 * Samco 2.1
 *
 * Copyright 2021 88hcsif
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
#include <Guncon2Interface.h>
#include <Perspective.h>
#include <Samco.h>
#include <TinyUSB_AbsMouse_and_Keyboard.h>

DFRobotIRPosition myDFRobotIRPosition;
Perspective per;
Samco mySamco;

int rawX[4];
int rawY[4];

int finalX[4];
int finalY[4];

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

#define DEBUG_FPS 1
#if DEBUG_FPS
#define FPS_CHECK 2000
int count = 0;
unsigned long lastFps = 0;
#endif

#define GUNCON_SUPPORT 1
#if (defined USE_TINYUSB) && GUNCON_SUPPORT
bool isGuncon = false;
int gunconPin = 7; // TRIGGER

bool isOn = false;
void flip_led(void) {
  if (!isOn) {
    digitalWrite(13, HIGH);
    isOn = true;
  } else {
    digitalWrite(13, LOW);
    isOn = false;
  }
}

#if CFG_TUSB_DEBUG
  #include <SPI.h>
  #include <SdFat.h>
  #include <Adafruit_SPIFlash.h>
  #include "Adafruit_TinyUSB.h"

  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;

  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif

  Adafruit_SPIFlash flash(&flashTransport);

  // file system object from SdFat
  FatFileSystem fatfs;

  // Configuration for the datalogging file:
  #define FILE_NAME "log.txt"

  // USB Mass Storage object
  Adafruit_USBD_MSC usb_msc;

  // Callback invoked when received READ10 command.
  int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize) {
    return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
  }

  // Callback invoked when received WRITE10 command.
  int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
    return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
  }

  // Callback invoked when WRITE10 command is completed
  void msc_flush_cb (void) {
    flash.syncBlocks();
    fatfs.cacheClear();
  }

  void print_to_file(const char* format, ...) {
    char buf[PRINTF_BUF];
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(buf, sizeof(buf), format, ap);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
    if (dataFile) {
      dataFile.write(buf);
      dataFile.close();
      flash.syncBlocks();
      fatfs.cacheClear();
    }
    va_end(ap);
  }

  #define CACHE_SIZE 4000
  #define CACHE_FLUSH 5000
  // #define MAX_FLUSH_COUNT 4
  char cache_buf[CACHE_SIZE];
  int cache_len = 0;
  unsigned long cache_flush = 0;
  int flush_count = 0;
  int print_count = 0;
  void flush_cache(void) {
    // flip_led();
    // if (flush_count >= MAX_FLUSH_COUNT) {
    //   return;
    // }
    if (cache_len > 0) {
      File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
      if (dataFile) {
        dataFile.print(flush_count);
        dataFile.print(" ");
        dataFile.print(print_count);
        dataFile.println();
        dataFile.write(cache_buf, cache_len);
        dataFile.close();
        flash.syncBlocks();
        fatfs.cacheClear();
      }
      cache_len = 0;
      cache_flush = millis();
      flush_count++;
    }
  }
  extern "C" int serial1_printf(const char* format, ...) {
    if (isGuncon) {
      char buf[PRINTF_BUF];
      va_list ap;
      va_start(ap, format);
      int n = vsnprintf(buf, PRINTF_BUF, format, ap);
      if (n >= sizeof(buf)) n = PRINTF_BUF - 1;
      memcpy(&cache_buf[cache_len], buf, n);
      cache_len += n;
      va_end(ap);
      print_count++;
      if ((cache_len > CACHE_SIZE - PRINTF_BUF) ||
          (millis() - cache_flush > CACHE_FLUSH)) {
        flush_cache();
      }
      return n;
    }
    return 0;
  }
#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

// Initialize C library
extern "C" void __libc_init_array(void);

/*
 * \brief Main entry point of Arduino application
 */
int main( void ) {
  init();
  __libc_init_array();
  initVariant();
  delay(1);

  pinMode(gunconPin, INPUT_PULLUP);
  isGuncon = (digitalRead(gunconPin) == LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  if (!isGuncon) {
    // Adafruit_TinyUSB_Core_init();
    Serial.setStringDescriptor("TinyUSB Serial");
    USBDevice.addInterface(Serial);
    USBDevice.setID(USB_VID, USB_PID);
    USBDevice.begin();
#if CFG_TUSB_DEBUG
    flash.begin();
    usb_msc.setID("Adafruit", "External Flash", "1.0");
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    usb_msc.setCapacity(flash.size()/512, 512);
    usb_msc.setUnitReady(true);
    usb_msc.begin();
    fatfs.begin(&flash);
    print_to_file("NOT GUNCON\n");
#endif
  } else {
#if CFG_TUSB_DEBUG
    flash.begin();
    fatfs.begin(&flash);
    print_to_file("GUNCON\n");
#endif
    Guncon.begin();
    USBDevice.setID(0x0b9a, 0x016a);
    // USBDevice.setVersion(0x0100);
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
#endif

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
    // while(!USBDevice.mounted()) delay(1);
    Guncon.init(1000, 1000, false);
  }

  for (int i = 0; i < 11; i++) {
    pinMode(pinMap[i], INPUT_PULLUP);
    currState[i] = HIGH;
    prevState[i] = HIGH;
    lastReportedState[i] = HIGH;
  }

  myDFRobotIRPosition.begin();
}

#define BLINK_PERIOD 3000
unsigned long last_blink = 0;
void loop() {
  if (millis() - last_blink > BLINK_PERIOD) {
    flip_led();
    last_blink = millis();
  }
  bool mouseChanged = false;
  bool keyboardChanged = false;
  bool gunconChanged = false;
  if (getRawPosition()) {
    if (getEstimatePosition()) {
      if (!isGuncon) {
        if (0 <= per.getX() && per.getX() <= 1000 && 0 <= per.getY() && per.getY() <= 1000) {
          mouseChanged = true;
          AbsMouse.move(per.getX(), per.getY());
        }
      } else {
        int x = per.getX();
        int y = per.getY();
        if (0 <= per.getX() && per.getX() <= 1000 && 0 <= per.getY() && per.getY() <= 1000) {
          Guncon.move(x, y);
        } else {
          Guncon.reset();
        }
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
            keyboardChanged = true;
            if (currState[i] == LOW) {
              Keyboard.press(keyMap[i]);
            } else {
              Keyboard.release(keyMap[i]);
            }
          }
        } else {
          gunconChanged = true;
          if (currState[i] == LOW) {
            Guncon.press(gunconMap[i]);
          } else {
            Guncon.release(gunconMap[i]);
          }
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
    Guncon.report();
#if CFG_TUSB_DEBUG
    if (millis() - cache_flush > CACHE_FLUSH) {
      flush_cache();
    }
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

bool getRawPosition() {
  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.readPosition()) {
    for (int i = 0; i < 4; i++) {
      rawX[i] = myDFRobotIRPosition.getX(i);
      rawY[i] = myDFRobotIRPosition.getY(i);
    }
    return true;
  } else {
    return false;
  }
}

bool getEstimatePosition() {
  if (mySamco.track(rawX[0], rawY[0], rawX[1], rawY[1], rawX[2], rawY[2], rawX[3], rawY[3])) {
    for (int i = 0; i < 4; i++) {
      finalX[i] = mySamco.getX(i);
      finalY[i] = mySamco.getY(i);
    }
    per.warp(finalX, finalY);
    return true;
  } else {
    return false;
  }
}

