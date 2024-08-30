/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MEM_H_
#define _MEM_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// 64KB Extended RAM
/*
#ifdef NDEBUG
extern uint8_t xram[0x10000];
#else
extern uint8_t *const xram;
#endif
*/
extern volatile uint8_t xram[0x10000];
//Oric 16kB bank overlays on xram
extern volatile uint8_t oric_bank0[0x4000];
extern volatile uint8_t oric_bank1[0x4000];
extern volatile uint8_t oric_bank2[0x4000];
extern volatile uint8_t oric_bank3[0x4000];

//Reserved space in ld script to ensure alignment
asm(".equ xram, 0x20000000");
asm(".equ oric_bank0, 0x20000000");     //Oric write shadow
asm(".equ oric_bank1, 0x20004000");     //cumini rom
asm(".equ oric_bank2, 0x20008000");     //device rom
asm(".equ oric_bank3, 0x2000C000");     //basic rom 

#define XRAMW(addr) ((uint16_t *)&xram[addr])[0]

//Oric IO space register address macro
// IO page and registers are located at the bottom of cpu1 stack.
// cpu1 runs the action loop and uses very little stack.
extern volatile uint8_t iopage[0x100];
asm(".equ iopage, 0x20040000");
#define IOREGS(addr) iopage[(addr) & 0xFF]
#define IOREGSW(addr) ((uint16_t *)&IOREGS(addr))[0]

// The xstack is:
// 512 bytes, enough to hold two CC65 stack frames.
// 1 byte at end+1 always zero for cstring and safety.
// Using xstack for cstrings doesn't require sending the zero termination.
#define XSTACK_SIZE 0x200
extern uint8_t xstack[];
extern volatile size_t xstack_ptr;

// MIA registers are located within the iopage memory
extern volatile uint8_t regs[0x20];
#define REGS(addr) regs[(addr) & 0x1F]
#define REGSW(addr) ((uint16_t *)&REGS(addr))[0]
//asm(".equ regs, 0x20040000");
asm(".equ regs, 0x200400A0");   //Oric address 0x03A0-0x03BF

// Misc memory buffer for moving things around.
// 6502 <-> RAM, USB <-> RAM, UART <-> RAM, etc.
#define MBUF_SIZE 1024
extern uint8_t mbuf[];
extern size_t mbuf_len;

#endif /* _MEM_H_ */
