/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _ACIA_H_
#define _ACIA_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef union acia_io_union {
    struct {
        uint8_t data;
        uint8_t stat;
        uint8_t cmd;
        uint8_t ctrl;
    };
    uint32_t regs;
} acia_io_t;

extern volatile acia_io_t* acia_io;

/*
#define ACIA_IO_DATA 0x0380
#define ACIA_IO_STAT 0x0381
#define ACIA_IO_CMD  0x0382
#define ACIA_IO_CTRL 0x0383
*/

/* Kernel events
 */

void acia_init(void);
void acia_task(void);
void acia_stop();

void __not_in_flash() acia_reset(bool hw_reset);
void __not_in_flash() acia_clr_irq(void);
void __not_in_flash() acia_read(void);
void __not_in_flash() acia_write(uint8_t data);
void __not_in_flash() acia_cmd(uint8_t data);
void __not_in_flash() acia_ctrl(uint8_t data);

#endif /* _ACIA_H_ */
