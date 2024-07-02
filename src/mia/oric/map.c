/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "oric/map.h"
#include "sys/ext.h"
#include "sys/mem.h"
#include "sys/ssd.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "littlefs/lfs_util.h"


void map_init(void)
{
    char status[20];
    sprintf(status, "RAM at 0x%08x",(unsigned int)xram);
    ssd_write_text(0,5,false,(uint8_t*)status);
    gpio_init(MAP_PIN);
    gpio_set_dir(MAP_PIN, GPIO_OUT);
    //gpio_pull_up(nMAP_PIN);
    gpio_put(MAP_PIN, 0);
    //gpio_put(MAP_PIN, 1);
}
void map_task(void)
{
    
}
void map_run()
{
}
void map_stop()
{
    
}

// Trigger IRQ when enabled
void map_trigger_irq(void)
{
    
}


