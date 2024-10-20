/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _API_H_
#define _API_H_

#include "sys/mem.h"
#include <string.h>

/* The 18 base errors come directly from CC65. Use them when you can.
 * FatFs has its own errors, which should be used when obtained from FatFs.
 * We can have both by using API_EFATFS(fresult) to return FatFs errors.
 * See the CC65 SDK method osmaperrno for how this is made portable.
 */

#define API_ENOENT 1    /* No such file or directory */
#define API_ENOMEM 2    /* Out of memory */
#define API_EACCES 3    /* Permission denied */
#define API_ENODEV 4    /* No such device */
#define API_EMFILE 5    /* Too many open files */
#define API_EBUSY 6     /* Device or resource busy */
#define API_EINVAL 7    /* Invalid argument */
#define API_ENOSPC 8    /* No space left on device */
#define API_EEXIST 9    /* File exists */
#define API_EAGAIN 10   /* Try again */
#define API_EIO 11      /* I/O error */
#define API_EINTR 12    /* Interrupted system call */
#define API_ENOSYS 13   /* Function not implemented */
#define API_ESPIPE 14   /* Illegal seek */
#define API_ERANGE 15   /* Range error */
#define API_EBADF 16    /* Bad file number */
#define API_ENOEXEC 17  /* Exec format error */
#define API_EUNKNOWN 18 /* Unknown OS specific error */
/* Start of FatFs errors */
#define API_EFATFS(fresult)  \
    (fresult + 32)
/* LFS errors are negative. We map to positive from 128 */
#define API_ELFSFS(lfs_result)  \
    (128 - lfs_result)

/* MIA fastcall registers
 */

#define API_OP IOREGS(0x03AF)
#define API_ERRNO IOREGSW(0x03AD)
#define API_STACK IOREGS(0x03AC)
#define API_BUSY (IOREGS(0x03B2) & 0x80)
#define API_A IOREGS(0x03B4)
#define API_X IOREGS(0x03B6)
#define API_SREG IOREGSW(0x03B8)
#define API_AX (API_A | (API_X << 8))
#define API_AXSREG (API_AX | (API_SREG << 16))

/* Kernel events
 */

void api_task(void);
void api_run(void);

// How to build an API handler:
// 1. The last fastcall argument is in API_A, API_AX or API_AXSREG.
// 2. Stack was pushed "in order". Like any top-down stack.
// 3. First parameter supports a "short stack".
//    e.g. a uint16 is sent for fseek instead of a uint64.
// 4. Be careful with the stack. Use this API.
// 5. Registers must be refreshed if XSTACK data changes.
// 6. Use the return functions always!

/* The last stack value, which is the first argument on the CC65 side,
 * may be a "short stack" to keep 6502 code as small as possible.
 * These will fail if the stack would not be empty after the pop.
 */

bool api_pop_uint8_end(uint8_t *data);
bool api_pop_uint16_end(uint16_t *data);
bool api_pop_uint32_end(uint32_t *data);
bool api_pop_int8_end(int8_t *data);
bool api_pop_int16_end(int16_t *data);
bool api_pop_int32_end(int32_t *data);

// Safely pop n bytes off the xstack. Fails with false if will underflow.
static inline bool api_pop_n(void *data, size_t n)
{
    if (XSTACK_SIZE - xstack_ptr < n)
        return false;
    memcpy(data, &xstack[xstack_ptr], n);
    xstack_ptr += n;
    return true;
}

/* Ordinary xstack popping. Use these for all but the final argument.
 */

static inline bool api_pop_uint8(uint8_t *data) { return api_pop_n(data, sizeof(uint8_t)); }
static inline bool api_pop_uint16(uint16_t *data) { return api_pop_n(data, sizeof(uint16_t)); }
static inline bool api_pop_uint32(uint32_t *data) { return api_pop_n(data, sizeof(uint32_t)); }
static inline bool api_pop_int8(int8_t *data) { return api_pop_n(data, sizeof(int8_t)); }
static inline bool api_pop_int16(int16_t *data) { return api_pop_n(data, sizeof(int16_t)); }
static inline bool api_pop_int32(int32_t *data) { return api_pop_n(data, sizeof(int32_t)); }

// Safely push n bytes to the xstack. Fails with false if no room.
static inline bool api_push_n(const void *data, size_t n)
{
    if (n > xstack_ptr)
        return false;
    xstack_ptr -= n;
    memcpy(&xstack[xstack_ptr], data, n);
    return true;
}

/* Ordinary xstack pushing.
 */

static inline bool api_push_uint8(const uint8_t *data) { return api_push_n(data, sizeof(uint8_t)); }
static inline bool api_push_uint16(const uint16_t *data) { return api_push_n(data, sizeof(uint16_t)); }
static inline bool api_push_uint32(const uint32_t *data) { return api_push_n(data, sizeof(uint32_t)); }
static inline bool api_push_int8(const int8_t *data) { return api_push_n(data, sizeof(int8_t)); }
static inline bool api_push_int16(const int16_t *data) { return api_push_n(data, sizeof(int16_t)); }
static inline bool api_push_int32(const int32_t *data) { return api_push_n(data, sizeof(int32_t)); }

// Returning data on XSTACK requires the
// read/write register to have the latest data.
static inline void api_sync_xstack(void)
{
    API_STACK = xstack[xstack_ptr];
}

// Same as opcode 0 from the 6502 side.
static inline void api_zxstack(void)
{
    API_STACK = 0;
    xstack_ptr = XSTACK_SIZE;
}

// Useful for variadic functions or other stack shenanigans.
static inline bool api_is_xstack_empty(void)
{
    return xstack_ptr == XSTACK_SIZE;
}

// Return works by manipulating 10 bytes of registers.
// 03B0 B8      CLV
// 03B1 50 FE   BVC -2
// 03B3 A9 FF   LDA #$FF
// 03B5 A2 FF   LDX #$FF
// 03B7 60      RTS
// 03B8 FF FF   .SREG $FF $FF

static inline void api_return_blocked() { *(uint32_t *)&regs[0x10] = 0xA9FE50B8; }
static inline void api_return_released() { *(uint32_t *)&regs[0x10] = 0xA90050B8; }

// Fast boot by jumping to reset vector
// 03B0   B8                   CLV
// 03B1   50 00                BVC +0
// 03B3   6C FC FF             JMP ($FFFC)
// 03B6   00                   BRK
// 03B7   60                   RTS
static inline void api_return_boot() { 
    *(uint32_t *)&regs[0x14] = 0x6000FFFC; 
    *(uint32_t *)&regs[0x10] = 0x6C0050B8; 
}

// Resume by calling RTI
// 03B0   B8                   CLV
// 03B1   50 00                BVC +0
// 03B3   40                   RTI
//static inline void api_return_resume() { *(uint32_t *)&regs[0x10] = 0x400050B8; }

// Resume by jumping to IRQ vector 
// 03B0   B8                   CLV
// 03B1   50 00                BVC +0
// 03B3   6C FE FF             JMP ($FFFE)
// 03B6   00                   BRK
// 03B7   60                   RTS
static inline void api_return_resume() { 
    *(uint32_t *)&regs[0x14] = 0x6000FFFE; 
    *(uint32_t *)&regs[0x10] = 0x6C0050B8; 
}

static inline void api_set_ax(uint16_t val)
{
    *(uint32_t *)&regs[0x14] = 0x6000A200 | (val & 0xFF) | ((val << 8) & 0xFF0000);
}


static inline void api_set_axsreg(uint32_t val)
{
    api_set_ax(val);
    API_SREG = val >> 16;
}

// Call one of these at the very end. These signal
// the 6502 that the operation is complete.

static inline void api_return_ax(uint16_t val)
{
    api_set_ax(val);
    api_return_released();
}
static inline void api_return_axsreg(uint32_t val)
{
    api_set_axsreg(val);
    api_return_released();
}
static inline void api_return_errno(uint16_t errno)
{
    api_zxstack();
    API_ERRNO = errno;
    api_return_axsreg(-1);
}

#endif /* _API_H_ */
