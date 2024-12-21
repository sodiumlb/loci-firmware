/*
 * Copyright (c) 2023 Rumbledethumps
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MIA_H_
#define _MIA_H_

/* RP6502 Interface Adapter for Oric computers.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel events
 */

void mia_init(void);
void mia_task(void);
void mia_main_task(void);
void mia_run();
void mia_stop();
void mia_reclock(uint16_t clkdiv_int, uint8_t clkdiv_frac);

// Trigger IRQ when enabled
void mia_trigger_irq(void);

// Move data from the 6502 to mbuf.
void mia_read_buf(uint16_t addr);

// Move data from mbuf to the 6502.
void mia_write_buf(uint16_t addr);

// Verify the mbuf matches 6502 memory.
void mia_verify_buf(uint16_t addr);

// The MIA is active when it's performing an mbuf action.
bool mia_active();

// The MIA is performing ROM loading for booting
bool mia_boot_active();

// Prints a "?" error and returns true if last mbuf action failed.
bool mia_print_error_message();

// Compute CRC32 of mbuf to match zlib.
uint32_t mia_buf_crc32();

//Enable read responce of ROM address space
//void mia_set_rom_read_enable(bool enable);
void mia_set_rom_ram_enable(bool device_rom, bool basic_rom);

//Call boot from Oric - typical from CUmini ROM
void mia_api_boot(void);

unsigned int mia_get_act_prg_offset(void);
unsigned int mia_get_io_read_prg_offset(void);
unsigned int mia_get_read_addr_prg_offset(void);
unsigned int mia_get_map_prg_offset(void);
unsigned int mia_get_ula_prg_offset(void);

//Snoop flag for IRQ capture loop
void mia_clear_snoop_flag(void);
bool mia_get_snoop_flag(void);
void mia_save_map_flags(void);

#define MIA_BOOTSET_FDC 0x01
#define MIA_BOOTSET_TAP 0x02
#define MIA_BOOTSET_B11 0x04
#define MIA_BOOTSET_TAP_BIT 0x08
#define MIA_BOOTSET_TAP_ALD 0x10
#define MIA_BOOTSET_FAST 0x80
#define MIA_BOOTSET_RESUME 0x40
//Get the boot settings info
uint8_t mia_get_boot_settings(void);

#endif /* _MIA_H_ */
