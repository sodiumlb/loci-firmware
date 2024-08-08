/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "oric/map.h"
#include "sys/cfg.h"
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

void map_api_tune_tmap(void)
{
    uint8_t delay = API_A;          //Range 0-31 for setting, other return current
    api_sync_xstack();              //For safety only
    cfg_set_map_delay(delay);        
    return api_return_ax(cfg_get_map_delay());
}
void map_api_tune_tior(void)
{
    uint8_t delay = API_A;          //Range 0-31 for setting, other return current
    api_sync_xstack();              //For safety only
    cfg_set_io_read_delay(delay);        
    return api_return_ax(cfg_get_io_read_delay());
}
void map_api_tune_tiow(void)
{
    uint8_t delay = API_A;          //Range 0-31 for setting, other return current
    api_sync_xstack();              //For safety only
    cfg_set_io_write_delay(delay);        
    return api_return_ax(cfg_get_io_write_delay());
}
void map_api_tune_tiod(void)
{
    uint8_t delay = API_A;          //Range 0-7 for setting, other return current
    api_sync_xstack();              //For safety only
    cfg_set_io_data_delay(delay);        
    return api_return_ax(cfg_get_io_data_delay());
}
void map_api_tune_tadr(void)
{
    uint8_t delay = API_A;          //Range 0-7 for setting, other return current
    api_sync_xstack();              //For safety only
    cfg_set_read_addr_delay(delay);        
    return api_return_ax(cfg_get_read_addr_delay());
}
