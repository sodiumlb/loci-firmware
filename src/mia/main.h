/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>
#include <stdbool.h>

/* This is the main kernel event loop.
 */

// Request to "start the 6502".
// It will safely do nothing if the 6502 is already running.
void main_run(void);

// Request to "stop the 6502".
// It will safely do nothing if the 6502 is already stopped.
void main_stop(void);

// Request to "break the system".
// A break is triggered by CTRL-ALT-DEL and UART breaks.
// If the 6502 is running, stop events will be called first.
// Kernel modules should reset to a state similar to after
// init() was first run.
void main_break(void);

// This is true when the 6502 is running or there's a pending
// request to start it.
bool main_active(void);

/* Special events dispatched in main.c
 */

void main_task(void);
void main_reclock(uint32_t sys_clk_khz, uint16_t clkdiv_int, uint8_t clkdiv_frac);
bool main_pix(uint8_t ch, uint8_t addr, uint16_t word);
bool main_api(uint8_t operation);

/* All pin assignments
 */
//Exhausted GPIO pins. Native UART only used in development. CDC Host UARTS supported in production in stead.
#define COM_UART uart0
#define COM_UART_BAUD_RATE 115200
#define COM_UART_TX_PIN 0
#define COM_UART_RX_PIN 1

#define A_PIN_BASE 0
#define A0_PIN (A_PIN_BASE + 0)
#define A1_PIN (A_PIN_BASE + 1)
#define A2_PIN (A_PIN_BASE + 2)
#define A3_PIN (A_PIN_BASE + 3)
#define A4_PIN (A_PIN_BASE + 4)
#define A5_PIN (A_PIN_BASE + 5)
#define A6_PIN (A_PIN_BASE + 6)
#define A7_PIN (A_PIN_BASE + 7)
#define A8_PIN (A_PIN_BASE + 8)
#define A9_PIN (A_PIN_BASE + 9)
#define A10_PIN (A_PIN_BASE + 10)
#define A11_PIN (A_PIN_BASE + 11)
#define A12_PIN (A_PIN_BASE + 12)
#define A13_PIN (A_PIN_BASE + 13)
#define A14_PIN (A_PIN_BASE + 14)
#define A15_PIN (A_PIN_BASE + 15)

#define D_PIN_BASE 16
#define D0_PIN (D_PIN_BASE + 0)
#define D1_PIN (D_PIN_BASE + 1)
#define D2_PIN (D_PIN_BASE + 2)
#define D3_PIN (D_PIN_BASE + 3)
#define D4_PIN (D_PIN_BASE + 4)
#define D5_PIN (D_PIN_BASE + 5)
#define D6_PIN (D_PIN_BASE + 6)
#define D7_PIN (D_PIN_BASE + 7)

//Inputs
#define RnW_PIN 24
#define PHI2_PIN 25

//Outputs
#define DIR_PIN 26
#define MAP_PIN 27

//EXTension (I2C)
#define EXT_I2C i2c0
#define SDA_PIN 28
#define SCL_PIN 29

//#define MIA_WRITE_PIO pio0
//#define MIA_WRITE_SM 0
#define MIA_ULA_PIO pio0
#define MIA_ULA_SM 0
#define MIA_ACT_PIO pio0
#define MIA_ACT_SM 1
#define MIA_MAP_PIO pio0
#define MIA_MAP_SM1 2
#define MIA_MAP_SM2 3

#define MIA_READ_PIO pio1
#define MIA_READ_DATA_SM 0
#define MIA_READ_ADDR_SM 1
#define MIA_IO_READ_PIO pio1
#define MIA_IO_READ_SM 2
#define MIA_ROM_READ_PIO pio1
#define MIA_ROM_READ_SM 3
//#define MIA_ROM_READ_SM2 3
#endif /* _MAIN_H_ */
