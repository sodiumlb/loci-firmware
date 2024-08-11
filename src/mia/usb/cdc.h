/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CDC_H_
#define _CDC_H_

#include "tusb.h"

/* Kernel events
 */

void cdc_init(void);
void cdc_task(void);

bool cdc_is_modem(uint8_t idx);


#endif /* _CDC_H_ */
