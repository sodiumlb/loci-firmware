/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _ORIC_ROM_H_
#define _ORIC_ROM_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sys/lfs.h"
#include "fatfs/ff.h"

bool rom_mount_lfs(lfs_file_t *lfs_file);
bool rom_mount_fat(FIL *fat_file);
void rom_umount();

bool rom_is_mounted(void);
bool rom_load_mounted(void);
/* Kernel events
 */

#endif /* _ORIC_ROM_H_ */