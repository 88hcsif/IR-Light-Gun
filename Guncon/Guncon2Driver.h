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

#ifndef _TUSB_GUNCON_DEVICE_H_
#define _TUSB_GUNCON_DEVICE_H_

#include "common/tusb_common.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Class Driver Default Configure & Validation
//--------------------------------------------------------------------+

#define CFG_TUD_GUNCON_EP_BUFSIZE 6

#define TUSB_SUBCLASS_GUNCON 0x6A

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Check if the interface is ready to use
bool tud_guncon_ready(void);

// Send report to host
bool tud_guncon_report(void const* report, uint8_t len);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void     guncond_init             (void);
void     guncond_reset            (uint8_t rhport);
uint16_t guncond_open             (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     guncond_control_request  (uint8_t rhport, tusb_control_request_t const * request);
bool     guncond_control_complete (uint8_t rhport, tusb_control_request_t const * request);
bool     guncond_xfer_cb          (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_GUNCON_DEVICE_H_ */

