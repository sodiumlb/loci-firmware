/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LED_H_
#define _LED_H_

#include <stdbool.h>

void led_toggle(void);
void led_set(bool on);

/* Kernel events
 */

void led_init(void);
void led_task(void);
#endif /* _LED_H_ */
