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
#include "api/api.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "littlefs/lfs_util.h"
#include "mia.pio.h"

//MUST be updated if PIO is changed
#define MAP_TUNE_INSTR 5
//TODO: Add assert that we're modding the right instruction

void map_init(void)
{
    char status[20];
    sprintf(status, "RAM at 0x%08x",(unsigned int)xram);
    ssd_write_text(0,5,false,status);
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

void map_api_tune(void)
{
    uint8_t delay = API_A & 0x1F;   //Range 0-31
    printf("MAP tune %d\n",delay);
    MIA_MAP_PIO->instr_mem[MAP_TUNE_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    api_sync_xstack();              //For safety only
    return api_return_ax(delay);
}
