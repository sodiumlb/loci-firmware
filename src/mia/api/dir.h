/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DIR_H_
#define _DIR_H_

#include <stdbool.h>

/* Kernel events
 */

void dir_task(void);
void dir_stop(void);

/* The API implementation for directory support.
 */

void dir_api_opendir(void);
void dir_api_closedir(void);
void dir_api_readdir(void);
void dir_api_readdir(void);

#endif /* _DIR_H_ */
