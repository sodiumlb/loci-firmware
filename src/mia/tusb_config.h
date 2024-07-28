/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _TUSB_CONFIG_H
#define _TUSB_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#define CFG_TUSB_MCU OPT_MCU_RP2040
#define CFG_TUSB_OS OPT_OS_PICO
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_HOST

#define CFG_TUH_ENUMERATION_BUFSIZE (1024)

//#define CFG_TUSB_DEBUG (1)
//#define CFG_TUH_LOG_LEVEL 1
//#define CFG_TUH_CDC_LOG_LEVEL 3

#define CFG_TUH_HUB (2)
#define CFG_TUH_CDC (2)
#define CFG_TUH_CDC_FTDI (1)
#define CFG_TUH_CDC_CP210X (1)
#define CFG_TUH_CDC_CH34X (1)
#define CFG_TUH_HID (4)
#define CFG_TUH_MSC (4)
#define CFG_TUH_VENDOR (0)

#define CFG_TUH_DEVICE_MAX (8)
#define CFG_TUH_ENDPOINT_MAX (16)
#define CFG_TUH_MAX_SPEED (OPT_MODE_FULL_SPEED)
//#define TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX 1

#define CFG_TUH_HID_EPIN_BUFSIZE (64)
#define CFG_TUH_HID_EPOUT_BUFSIZE (64)

#define CFG_TUH_CDC_RX_BUFSIZE 64
//Needs to be big enough to hold all stdio output before next call to tuh_task() 
#define CFG_TUH_CDC_TX_BUFSIZE 1200

#define CFG_TUH_CDC_LINE_CONTROL_ON_ENUM (0x03)
//#define CFG_TUH_CDC_LINE_CODING_ON_ENUM { 115200, CDC_LINE_CODING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM { 9600, CDC_LINE_CODING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }

#ifdef __cplusplus
}
#endif


#endif
