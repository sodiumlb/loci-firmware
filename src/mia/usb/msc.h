/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MSC_H_
#define _MSC_H_

#include "tusb.h"

/* Kernel events
 */

void msc_init(void);
void msc_task(void);

#endif /* _MSC_H_ */
