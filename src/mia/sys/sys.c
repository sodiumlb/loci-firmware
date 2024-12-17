/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "api/api.h"
#include "sys/mem.h"
#include "sys/sys.h"
#include "usb/usb.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>

/* CC65 utsname 
struct utsname {
    char sysname[17];
    char nodename[9];
    char release[9];
    char version[9];
    char machine[25];
};
*/
#define UTS_SYSNAME_SIZE 17
#define UTS_NODENAME_SIZE 9
#define UTS_RELEASE_SIZE 9
#define UTS_VERSION_SIZE 9
#define UTS_MACHINE_SIZE 25
static char __in_flash() uts_sysname[UTS_SYSNAME_SIZE] = LOCIFW_NAME;
static char uts_nodename[UTS_NODENAME_SIZE] = "";
static char __in_flash() uts_release[UTS_RELEASE_SIZE] = 
#ifdef LOCIFW_VERSION
    LOCIFW_VERSION;
#else 
    "dev";
#endif
static char __in_flash() uts_version[UTS_VERSION_SIZE] = __TIME__;
static char __in_flash() uts_machine[UTS_MACHINE_SIZE] = LOCIFW_BOARD;

static void sys_print_status(void)
{
    puts(LOCIFW_NAME);
    #ifdef LOCIFW_VERSION
        puts("LOCI firmware " LOCIFW_VERSION);
    #else
        puts("MIA " __DATE__ " " __TIME__);
    #endif
}

void sys_mon_reboot(const char *args, size_t len)
{
    (void)(args);
    (void)(len);
    watchdog_reboot(0, 0, 0);
}

void sys_mon_reset(const char *args, size_t len)
{
    (void)(args);
    (void)(len);
    main_run();
}

void sys_mon_status(const char *args, size_t len)
{
    (void)(args);
    (void)(len);
    sys_print_status();
    printf("Oric Addr %04X\n", sio_hw->gpio_in & 0x0000FFFF);
    //vga_print_status();
    usb_print_status();
}

void sys_init(void)
{
    // Reset terminal.
    puts("\30\33[0m\f");
    // Hello, world.
    sys_print_status();
}

void sys_api_uname(void){
    api_zxstack();
    api_push_n(uts_machine,UTS_MACHINE_SIZE);
    api_push_n(uts_version,UTS_VERSION_SIZE);
    api_push_n(uts_release,UTS_RELEASE_SIZE);
    api_push_n(uts_nodename,UTS_NODENAME_SIZE);
    api_push_n(uts_sysname,UTS_SYSNAME_SIZE);
    api_sync_xstack();
    return api_return_ax(xstack_ptr);    
}
