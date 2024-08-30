/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _TAP_H_
#define _TAP_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sys/lfs.h"
#include "fatfs/ff.h"

#define TAP_IO_CMD  0x0315
#define TAP_IO_STAT 0x0316
#define TAP_IO_DATA 0x0317

/* Kernel events
 */

void tap_init(void);
void tap_task(void);
void tap_stop();

void tap_act(uint8_t data);

bool tap_mount_lfs(lfs_file_t *lfs_file);
bool tap_mount_fat(FIL* fat_file);
void tap_umount(void);

void tap_api_seek(void);
void tap_api_tell(void);
void tap_api_read_header(void);
#endif /* _TAP_H_ */
