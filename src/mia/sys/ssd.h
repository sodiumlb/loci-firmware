/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SSD_H_
#define _SSD_H_

/* Kernel events
 */

void ssd_init(void);
void ssd_task(void);

int ssd_putc(char ch);
void ssd_write_text(uint8_t x, uint8_t y, bool invert, char *text);

extern volatile bool ssd_got_action_word;
extern volatile uint32_t ssd_action_word;
extern volatile uint32_t ssd_action_rword;
extern volatile bool ssd_action_is_wr;

#endif /* _SSD_H_ */
