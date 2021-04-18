/*
 * TinyUSB Guncon 2 interface
 *
 * Copyright 2021 88hcsif
 *
 * Based on Adafruit_USBD_HID:
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef GUNCON2INTERFACE_H_
#define GUNCON2INTERFACE_H_

#ifndef USE_TINYUSB
#error TinyUSB is not selected, please select it in Tools->Menu->USB Stack
#endif

#include "Adafruit_TinyUSB_Core.h"

#ifndef CFG_TUD_VENDOR
#error Not supported
#endif

#define GUNCON_DPAD_LEFT  0x0080
#define GUNCON_DPAD_DOWN  0x0040
#define GUNCON_DPAD_RIGHT 0x0020
#define GUNCON_DPAD_UP    0x0010
#define GUNCON_A          0x0008
#define GUNCON_B          0x0004
#define GUNCON_C          0x0002
#define GUNCON_START      0x8000
#define GUNCON_SELECT     0x4000
#define GUNCON_TRIGGER    0x2000

#define GUNCON_X_MIN      150
#define GUNCON_X_MAX      724
#define GUNCON_Y_MIN       32
#define GUNCON_Y_MAX      295

typedef struct {
  uint16_t buttons;
  uint16_t x;
  uint16_t y;
} GunconReport;

class Guncon2Interface : public Adafruit_USBD_Interface {
private:
  GunconReport _report;
  uint8_t _itfnum;
  uint32_t _width;
  uint32_t _height;
  bool _autoReport;

public:
  Guncon2Interface(void);
  bool begin(void);
  void init(uint16_t width = 32767, uint16_t height = 32767, bool autoReport = true);
  void report(void);
  void reset();
  void move(uint16_t x, uint16_t y);
  void press(uint16_t button);
  void release(uint16_t button);

  // from Adafruit_USBD_Interface
  virtual uint16_t getDescriptor(uint8_t itfnum, uint8_t *buf,
                                 uint16_t bufsize);
};

extern Guncon2Interface Guncon;

#endif /* GUNCON2INTERFACE_H_ */
