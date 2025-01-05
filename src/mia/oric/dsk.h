/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DSK_H_
#define _DSK_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sys/lfs.h"
#include "fatfs/ff.h"


#define DSK_IO_CMD   0x0310
#define DSK_IO_TRACK 0x0311
#define DSK_IO_SECT  0x0312
#define DSK_IO_DATA  0x0313
#define DSK_IO_CTRL  0x0314
#define DSK_IO_DRQ   0x0318

bool dsk_mount_lfs(uint8_t drive, lfs_file_t *lfs_file);
bool dsk_mount_fat(uint8_t drive, FIL *fat_file);
void dsk_umount(uint8_t drive);
/* Kernel events
 */

void dsk_init(void);
void dsk_task(void);
void dsk_run(void);
void dsk_stop(void);
void dsk_act(uint8_t raw_cmd);
void dsk_rw(bool is_write,uint8_t data);
void dsk_set_ctrl(uint8_t raw_reg);
void dsk_pause(bool on);

extern volatile uint8_t dsk_reg_status, dsk_next_busy, dsk_reg_cmd;
#define dsk_reg_irq    IOREGS(DSK_IO_CTRL)

enum DSK_STATE { 
    DSK_IDLE, 
    DSK_SEEK, 
    DSK_READ_PREP, DSK_READ, 
    DSK_WRITE_PREP, DSK_WRITE, 
    DSK_READ_ADDR,
    DSK_TOGGLE_IRQ,
    DSK_CLEANUP 
};
extern volatile enum DSK_STATE dsk_state;

#endif /* _DSK_H_ */
