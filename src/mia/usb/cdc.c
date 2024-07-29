/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "api/api.h"
#include "sys/cfg.h"
#include "usb/cdc.h"
#include "usb/usb.h"
#include "pico/stdio/driver.h"

void cdc_stdio_out_chars(const char *buf, int length);
void cdc_stdio_out_flush(void);
static int cdc_stdio_in_chars(char *buf, int length);

//Currently only use first CDC device found as a STDIO terminal
//static uint8_t cdc_device_id = 0xFF;

static stdio_driver_t cdc_stdio_app = {
    .out_chars = cdc_stdio_out_chars,
    .out_flush = cdc_stdio_out_flush,
    .in_chars = cdc_stdio_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF
#endif
};


void cdc_stdio_out_chars(const char *buf, int length)
{

    for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        if(tuh_cdc_mounted(i)){
            //tuh_cdc_write(i, buf, length);
            
            int sent = 0;
            do {
                sent += tuh_cdc_write(i,(const char *)(buf+sent),length-sent);
                if(sent < length)
                    tuh_task();     //TODO This is brute force. Any nicer options?
            } while(sent < length);
            
        }
    }
}

void cdc_stdio_out_flush(void)
{
    for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        if(tuh_cdc_mounted(i)){
            tuh_cdc_write_flush(i);
        }
    }
}

static int cdc_stdio_in_chars(char *buf, int length)
{
    int ret = 0;
    for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        if(tuh_cdc_mounted(i) && tuh_cdc_read_available(i)){
            ret = tuh_cdc_read(i,buf,length);
            break;
        }
    }
    return ret;
}


void cdc_init(void)
{
    stdio_set_driver_enabled(&cdc_stdio_app, true);
}

void cdc_task(void)
{
    cdc_stdio_out_flush();
}

void tuh_cdc_mount_cb(uint8_t idx)
{
    tuh_itf_info_t itf_info = {0};
    tuh_cdc_itf_get_info(idx, &itf_info);
    usb_set_status(itf_info.daddr,"CDC device mounted");
    printf("CDC (%d) mounted\n",itf_info.daddr);
}

void tuh_cdc_umount_cb(uint8_t idx)
{
    //if(cdc_device_id == idx){
    //    cdc_device_id = 0xFF;
        usb_set_status(idx,"CDC device unmounted");
    //}
}