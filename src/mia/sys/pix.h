/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PIX_H_
#define _PIX_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Kernel events
 */

void pix_init(void);
void pix_stop(void);
void pix_reclock(uint16_t clkdiv_int, uint8_t clkdiv_frac);

/* API to set XREGs
 */

void pix_api_xreg(void);
void pix_ack(void);
void pix_nak(void);

// Well known PIX devices. 2-6 are for user expansion.
// MIA device 0 is virtual, not on the physical PIX bus.

#define PIX_DEVICE_XRAM 0
#define PIX_DEVICE_MIA 0
#define PIX_DEVICE_VGA 1
#define PIX_DEVICE_IDLE 7

// Test for free space in the PIX transmit FIFO.
static inline bool pix_ready(void)
{
    return true;
}

// Test for empty transmit FIFO.
static inline bool pix_fifo_empty(void)
{
    return true;
}

// Unconditionally attempt to send a PIX message.
// Meant for use with pix_ready() to fill the FIFO in a task handler.
static inline void pix_send(uint8_t dev3, uint8_t ch4, uint8_t byte, uint16_t word)
{
}

// Send a single PIX message, block if necessary. Normally, blocking is bad, but
// this unblocks so fast that it's not a problem for a few messages.
static inline void pix_send_blocking(uint8_t dev3, uint8_t ch4, uint8_t byte, uint16_t word)
{
}

#endif /* _PIX_H_ */
