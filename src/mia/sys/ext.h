/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _EXT_H_
#define _EXT_H_

#define HW_REV_1_2 1

#ifdef HW_REV_1_1
#define EXT_nRESET  0b00000001
#define EXT_ROMDIS  0b00000010
#define EXT_IRQ     0b00000100
#define EXT_BTN_A   0b00001000
#define EXT_BTN_B   0b00010000
#define EXT_BTN_C   0b00100000
#define EXT_BTN_D   0b01000000
#define EXT_LED     0b10000000
#endif

#ifdef HW_REV_1_2
#define EXT_SWVBUS  0b00000001
#define EXT_SWVEXT  0b00000010
#define EXT_BTN_A   0b00000100
#define EXT_LED     0b00001000
#define EXT_RESET   0b00010000
#define EXT_IRQ     0b00100000
#define EXT_ROMDIS  0b01000000
#define EXT_nOE     0b10000000
#endif

/* Kernel events
 */

void ext_init(void);
void ext_task(void);

void ext_put(uint8_t pin, bool value);
bool ext_get(uint8_t pin);
bool ext_get_cached(uint8_t pin);
void ext_set_dir(uint8_t pin, bool output);
void ext_pulse(uint8_t pin);

#endif /* _EXT_H_ */
