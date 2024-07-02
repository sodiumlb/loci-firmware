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
    //TODO put in proper flow control when cable arrives
        /*
    if(cdc_device_id != 0xFF){
        bool xoff = false;
        //char info[32];
        //int slen = sprintf(info,"<%d>",length);
        //tuh_cdc_write(cdc_device_id,info,slen);
        for(int i=0; i < length; i++){
            do{
                uint8_t ch = 0;
                tuh_cdc_peek(cdc_device_id,&ch);
                if(ch == 0x11){
                    xoff = false;
                    tuh_cdc_read(cdc_device_id,&ch,1);
                }
                if(ch == 0x13){
                    xoff = true;
                    tuh_cdc_read(cdc_device_id,&ch,1);
                }
            }while(xoff);
            //while(tuh_cdc_write_available(cdc_device_id)==0){
                //tuh_cdc_write_flush(cdc_device_id);
            //}
            tuh_cdc_write(cdc_device_id,(unsigned char*)(buf+i),1);   
        }
        */
       /* 
        int sent = 0;
        while(sent < length){
            sent += tuh_cdc_write(cdc_device_id,(const char *)(buf+sent),length-sent);
            tuh_cdc_write_flush(cdc_device_id);
        }
        */
    //}
    

    for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        if(tuh_cdc_mounted(i)){
            tuh_cdc_write(i,buf,length);
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

void tuh_mount_cb(uint8_t idx)
{
    uint16_t vid, pid;
    tuh_vid_pid_get(idx,&vid,&pid);
    printf("USB %04X:%04X (%d) mounted\n", vid, pid, idx);
    for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        if(tuh_cdc_mounted(i)){
            //if(cdc_device_id == 0xFF)
            //    cdc_device_id = i;
            printf("CDC %d mounted\n", i);
            usb_set_status(idx,"CDC device mounted");
        }
    }
}

void tuh_cdc_umount_cb(uint8_t idx)
{
    //if(cdc_device_id == idx){
    //    cdc_device_id = 0xFF;
        usb_set_status(idx,"CDC device unmounted");
    //}
}