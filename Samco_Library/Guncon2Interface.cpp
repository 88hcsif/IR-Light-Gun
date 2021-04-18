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

#include <Arduino.h>
#include "Guncon2Interface.h"
#include "Guncon2Driver.h"

#define TUD_GUNCON_DESCRIPTOR(_itfnum) \
  /*   INTERFACE 0: Vendor Specific                                  */\
  0x09,                       /*   bLength (9 bytes)                 */\
  TUSB_DESC_INTERFACE,        /*   bDescriptorType (Interface)       */\
  _itfnum,                    /*   bInterfaceNumber                  */\
  0x00,                       /*   bAlternateSetting                 */\
  0x01,                       /*   bNumEndpoints                     */\
  TUSB_CLASS_VENDOR_SPECIFIC, /*   bInterfaceClass (Vendor Specific) */\
  TUSB_SUBCLASS_GUNCON,       /*   bInterfaceSubClass                */\
  0x00,                       /*   bInterfaceProtocol                */\
  0x00,                       /*   iInterface                        */\
  /*     ENDPOINT 0x81: Interrupt IN                                 */\
  0x07,                       /*     bLength (7 bytes)               */\
  TUSB_DESC_ENDPOINT,         /*     bDescriptorType (Endpoint)      */\
  0x81,                       /*     bEndpointAddress (IN)           */\
  TUSB_XFER_INTERRUPT,        /*     bmAttributes (Interrupt)        */\
  U16_TO_U8S_LE(8),           /*     wMaxPacketSize (8 bytes)        */\
  0x08                        /*     bInterval                       */

//------------- IMPLEMENTATION -------------//
Guncon2Interface::Guncon2Interface(void) {
  _report.buttons = 0xFFFF;
  _report.x = 0;
  _report.y = 0;
}

uint16_t Guncon2Interface::getDescriptor(uint8_t itfnum, uint8_t *buf,
                                          uint16_t bufsize) {
  // usb core will automatically update endpoint number
  // uint8_t const desc[] = {TUD_GUNCON_DESCRIPTOR(itfnum)};
  uint8_t const desc[] = {TUD_GUNCON_DESCRIPTOR(itfnum)};
  uint16_t len = sizeof(desc);

  if (bufsize < len) {
    return 0;
  }

  _itfnum = itfnum;
  memcpy(buf, desc, len);
  return len;
}

bool Guncon2Interface::begin(void) {
  if (!USBDevice.addInterface(*this))
    return false;

  return true;
}

void Guncon2Interface::init(uint16_t width, uint16_t height, bool autoReport) {
  _width = width;
  _height = height;
  _autoReport = autoReport;
}

void Guncon2Interface::report(void) {
  if (!tud_guncon_ready()) return;
  tud_guncon_report(&_report, sizeof(GunconReport));
}

void Guncon2Interface::reset() {
  _report.x = 0;
  _report.y = 0;

  if (_autoReport) {
    report();
  }
}

void Guncon2Interface::move(uint16_t x, uint16_t y) {
  _report.x = (uint16_t) (((GUNCON_X_MAX - GUNCON_X_MIN) * ((uint32_t) x)) / _width ) + GUNCON_X_MIN;
  _report.y = (uint16_t) (((GUNCON_Y_MAX - GUNCON_Y_MIN) * ((uint32_t) y)) / _height) + GUNCON_Y_MIN;

  if (_autoReport) {
    report();
  }
}

// #define INCREMENT 1
void Guncon2Interface::press(uint16_t button) {
  // switch(button) {
  //   case GUNCON_DPAD_UP:
  //     _report.y -= INCREMENT;
  //     break;
  //   case GUNCON_DPAD_DOWN:
  //     _report.y += INCREMENT;
  //     break;
  //   case GUNCON_DPAD_LEFT:
  //     _report.x -= INCREMENT;
  //     break;
  //   case GUNCON_DPAD_RIGHT:
  //     _report.x += INCREMENT;
  //     break;
  //   case GUNCON_SELECT:
  //     _report.x = GUNCON_X_MIN;
  //     _report.y = GUNCON_Y_MIN;
  //     break;
  //   case GUNCON_START:
  //     _report.x = GUNCON_X_MAX;
  //     _report.y = GUNCON_Y_MAX;
  //     break;
  //   default:
  //     _report.buttons &= ~button;
  //     break;
  // }
  _report.buttons &= ~button;

  if (_autoReport) {
    report();
  }
}

void Guncon2Interface::release(uint16_t button) {
  _report.buttons |= button;

  if (_autoReport) {
    report();
  }
}

Guncon2Interface Guncon;
