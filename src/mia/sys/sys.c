/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "sys/sys.h"
#include "usb/usb.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>

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
