/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MAP_H_
#define _MAP_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel events
 */

void map_init(void);
void map_task(void);
void map_run();
void map_stop();

// Trigger IRQ when enabled
void map_trigger_irq(void);

void map_api_tune_tmap(void);
void map_api_tune_tior(void);
void map_api_tune_tiow(void);
void map_api_tune_tiod(void);
void map_api_tune_tadr(void);

#endif /* _MAP_H_ */
