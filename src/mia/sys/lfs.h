/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LFS_H_
#define _LFS_H_

//CUmini uses 16MB flash, and 15MB for our LSF storage
//#define PICO_FLASH_SIZE_BYTES 16777216

#include "hardware/flash.h"
#include "littlefs/lfs.h"

// Our only volume is mounted here for all to use.
extern lfs_t lfs_volume;

// Use this to obtain a temporary lfs_file_config on the stack.
#define _LFS_FILE_CONFIG_NAME(name) lfs_file_config_buffer_##name
#define LFS_FILE_CONFIG(name)                             \
    uint8_t _LFS_FILE_CONFIG_NAME(name)[FLASH_PAGE_SIZE]; \
    struct lfs_file_config name = {                       \
        .buffer = _LFS_FILE_CONFIG_NAME(name),            \
    };

// Use this to allocate, use and free lfs_file_config from the central pool
#define LFS_MAX_FILE_CONFIGS 8
extern struct lfs_file_config lfs_file_configs[LFS_MAX_FILE_CONFIGS];
struct lfs_file_config* lfs_alloc_file_config(void);
void lfs_free_file_config(lfs_file_t *fp);

/* Kernel events
 */

void lfs_init(void);

// Test is file position is at the end of the file.
int lfs_eof(lfs_file_t *file);

// Print formatted characters to the file.
int lfs_printf(lfs_t *lfs, lfs_file_t *file, const char *format, ...);

// Safe gets.
char *lfs_gets(char *str, int n, lfs_t *lfs, lfs_file_t *file);

#endif /* _LFS_H_ */
