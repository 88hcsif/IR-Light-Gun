/*
 * TinyUSB Guncon 2 driver
 *
 * Copyright 2021 88hcsif
 *
 * Based on examples from TinyUSB:
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "tusb_option.h"
#include <Arduino.h>

#define CFG_TUD_GUNCON 1
#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_GUNCON)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "common/tusb_common.h"
#include "device/usbd_pvt.h"
#include "Guncon2Driver.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct {
  bool ready;
  uint8_t ep_in;
  CFG_TUSB_MEM_ALIGN uint8_t epin_buf[CFG_TUD_GUNCON_EP_BUFSIZE];
} guncond_interface_t;

CFG_TUSB_MEM_SECTION static guncond_interface_t _guncond_itf;

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
bool tud_guncon_ready(void) {
  uint8_t const ep_in = _guncond_itf.ep_in;
  return tud_ready() && _guncond_itf.ready && (ep_in != 0) && !usbd_edpt_busy(TUD_OPT_RHPORT, ep_in);
}

bool tud_guncon_report(void const* report, uint8_t len) {
  if (!_guncond_itf.ready) return false;

  // claim endpoint
  TU_VERIFY( usbd_edpt_claim(TUD_OPT_RHPORT, _guncond_itf.ep_in) );

  // prepare data
  len = tu_min8(len, CFG_TUD_GUNCON_EP_BUFSIZE);
  memcpy(_guncond_itf.epin_buf, report, len);

  return usbd_edpt_xfer(TUD_OPT_RHPORT, _guncond_itf.ep_in, _guncond_itf.epin_buf, len);
}

//--------------------------------------------------------------------+
// USBD-CLASS API
//--------------------------------------------------------------------+
void guncond_init(void) {
  guncond_reset(TUD_OPT_RHPORT);
}

void guncond_reset(uint8_t rhport) {
  (void) rhport;
  tu_memclr(&_guncond_itf, sizeof(guncond_interface_t));
}

uint16_t guncond_open(uint8_t rhport, tusb_desc_interface_t const * desc_itf, uint16_t max_len) {
  TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_itf->bInterfaceClass, 0);
  TU_VERIFY(TUSB_SUBCLASS_GUNCON == desc_itf->bInterfaceSubClass, 0);

  // len = interface + n*endpoints
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + desc_itf->bNumEndpoints*sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(max_len >= drv_len, 0);

  // Find available interface
  guncond_interface_t * p_hid = NULL;
  TU_ASSERT(_guncond_itf.ep_in == 0, 0);

  uint8_t const *p_desc = (uint8_t const *) desc_itf;

  // Endpoint Descriptor
  p_desc = tu_desc_next(p_desc);
  TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, desc_itf->bNumEndpoints, TUSB_XFER_INTERRUPT, NULL, &_guncond_itf.ep_in), 0);

  return drv_len;
}

bool guncond_control_request(uint8_t rhport, tusb_control_request_t const * request) {
  (void) rhport;
  (void) request;

  _guncond_itf.ready = true;
  return false;
}

bool guncond_control_complete(uint8_t rhport, tusb_control_request_t const * request) {
  (void) rhport;
  (void) request;

  return true;
}

bool guncond_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) rhport;
  (void) ep_addr;
  (void) result;
  (void) xferred_bytes;

  return true;
}

//--------------------------------------------------------------------+
// Class Driver
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= 2
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif

// Built-in class drivers
static usbd_class_driver_t const _app_driver[] =
{
  {
      DRIVER_NAME("GUNCON")
      .init             = guncond_init,
      .reset            = guncond_reset,
      .open             = guncond_open,
      .control_request  = guncond_control_request,
      .control_complete = NULL,
      .xfer_cb          = guncond_xfer_cb,
      .sof              = NULL
  }
};

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
  *driver_count = 1;
  return _app_driver;
}

#endif
