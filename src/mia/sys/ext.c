/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "sys/ext.h"
#include "pico/stdlib.h"
//#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mon/rom.h"
#include "sys/com.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "sys/ssd.h"
#include <stdio.h>
#include "oric/tap.h"
#include "oric/dsk.h"
#include "fatfs/ff.h"
#include "usb/mou.h"

#ifdef HW_REV_1_1
//PCA9557
#define I2C_IOEXP_ADDR 0x18
#define I2C_IOEXP_REG_IN 0x00
#define I2C_IOEXP_REG_OUT 0x01
#define I2C_IOEXP_REG_INV 0x02
#define I2C_IOEXP_REG_DIR 0x03
//LED,BTN_D,BTN_C,BTN_B,BTN_A,IRQ,ROMDIS_3V3,nRESET
#define I2C_IOEXP_DIR 0b01111000
#define I2C_IOEXP_INV 0b01111000
#define I2C_INIT_ODATA (EXT_nRESET|EXT_LED)
#endif
#ifdef HW_REV_1_2
//PI4IOE5V6408ZTAEX 
#define I2C_IOEXP_ADDR 0x43
#define I2C_IOEXP_REG_IN 0x0F
#define I2C_IOEXP_REG_OUT 0x05
#define I2C_IOEXP_REG_DIR 0x03
#define I2C_IOEXP_REG_HIZ 0x07
//LED,BTN_D,BTN_C,BTN_B,BTN_A,IRQ,ROMDIS_3V3,nRESET
//nOE,ROMDIS,IRQ,RESET,LED,BTN_A,SWEXT,SWVBUS
#define I2C_IOEXP_OUTPUT_IS_ONE
#define I2C_IOEXP_DIR 0b11111001
#define I2C_IOEXP_INV 0b00000100
#define I2C_INIT_ODATA (EXT_LED|EXT_SWVBUS)
#endif
static uint8_t ext_port_odata = I2C_INIT_ODATA;
static uint8_t ext_port_idata = 0;
static uint8_t ext_port_dir = I2C_IOEXP_DIR;


static enum {
    EXT_IDLE,
    EXT_LOADING_DEVROM,
    EXT_LOADING_BIOS,
} ext_state;


void ext_update(void);
bool ext_get_cached(uint8_t pin);

void ext_init(void)
{

    //i2c_init(EXT_I2C, 400*1000);
    //i2c_init(EXT_I2C, 1200*1000);
    i2c_init(EXT_I2C, 1200*1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
//    bi_dec(bi_2pins_with_func(SDA_PIN,SCL_PIN,GPIO_FUNC_I2C));
    {
        uint8_t txdata[] = { I2C_IOEXP_REG_DIR, ext_port_dir };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }
#ifdef I2C_IOEXP_REG_INV
    {
        uint8_t txdata[] = { I2C_IOEXP_REG_INV, I2C_IOEXP_INV };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }
#endif
    {
        uint8_t txdata[] = { I2C_IOEXP_REG_OUT, ext_port_odata };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }
#ifdef I2C_IOEXP_REG_HIZ
    {
        uint8_t txdata[] = { I2C_IOEXP_REG_HIZ, 0x00 };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }
#endif
    ext_state = EXT_IDLE;
}

#define TEST_PRG_ADDR (0xFFE8)
const uint8_t __in_flash() test_prg[] = {
    //FFE8: 
    0xA9, 0x1A, 0x8D, 0x11, 0x03, 0x8D, 0x80, 0xBB,
    //FFF0: 
    //0x59, 0xF8, 0xA9, 0x01, 0x50, 0xF4, 0x00, 0x00,
    0xD0, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //FFF8: 
    0x00, 0x00, 0xF2, 0xFF, 0xE8, 0xFF, 0xE8, 0xFF
};

FIL tap_file;
FIL dsk_file;

#define CLOAD_PATCH_11_ADDR (0xE6C9)
const uint8_t __in_flash() cload_patch_11[] = {
    0xA9, 0x01, 0x8D, 0x15, 0x03, 0xAD, 0x15, 0x03,
    0xD0, 0xFB, 0xAD, 0x17, 0x03, 0x85, 0x2F, 0x60
};

#define EXT_I2C_SAMPLE_DELAY 0x4000;
uint16_t ext_refresh_cnt = EXT_I2C_SAMPLE_DELAY;

void ext_task(void)
{
    switch(ext_state){
        case EXT_LOADING_DEVROM:
            if(!rom_active()){
                if(!rom_load("BASIC11",7)){
                //if(!rom_load("test108k",8)){
                    ssd_write_text(0,2,true,"!rom_load failed");
                }else{
                    ssd_write_text(0,1,false,"BIOS loaded ok");
                }
                ext_state = EXT_LOADING_BIOS;
            }
            break;
        case EXT_LOADING_BIOS:
            if(!rom_active()){
                ext_state = EXT_IDLE;
                main_run();
                //Patch for TAP loading CLOAD
                //xram[0xE4AC+0] = 0xEA;  //NOP
                //xram[0xE4AC+1] = 0xEA;  //NOP
                //xram[0xE4AC+2] = 0xEA;  //NOP
                //for(uint16_t i=0; i<sizeof(cload_patch_11); i++){
                //    xram[CLOAD_PATCH_11_ADDR+i] = cload_patch_11[i];
                //}    
              }
            break;
    }

    if(ext_refresh_cnt--)
        return;
    ext_refresh_cnt = EXT_I2C_SAMPLE_DELAY;

    /*
    static uint8_t scan_addr = 1;
    {
        uint8_t idata;
        uint8_t txdata[] = { scan_addr };

        i2c_write_blocking(EXT_I2C, I2C_IOEXP_ADDR, txdata, 1, true);
        if(i2c_read_blocking(EXT_I2C, I2C_IOEXP_ADDR, &idata, 1, false) < 0){
            printf(".");
        }else{
            printf("%02x%02x|",scan_addr,idata);
        }
    }
    if(scan_addr++ > 0x13)
        scan_addr = 1;
    return;
    */

    ext_update();
    if(!main_active()){
        if(ext_get_cached(EXT_BTN_A)){
            rom_mon_load("loci_rom.rp6502", 15);    //First ROM priority: USB storage
            if(!rom_active()){
                rom_load("LOCI_ROM",8);             //Second ROM priority: new name in flash
            }
            if(!rom_active()){
                rom_load("CUMINIROM",9);            //Third ROM priority: old name in flash
            }
            ext_state = EXT_LOADING_BIOS;

            /**
            //mia_set_rom_read_enable(true);
            //ext_put(EXT_nROMDIS,false);
            //ext_set_dir(EXT_nROMDIS,true);
            if(!rom_load("MICRODISC",9)){
                ssd_write_text(0,2,true,"!rom_load failed");
            }else{
                ssd_write_text(0,2,false,"DEV ROM loaded ok");
            }
            ext_state = EXT_LOADING_DEVROM;

            f_open(&tap_file,"im10.tap",FA_READ);
            tap_mount_fat(&tap_file);
            //f_open(&dsk_file,"sedoric3.dsk",FA_READ);
            //f_open(&dsk_file,"B7en-1_3.dsk",FA_READ | FA_WRITE);
            f_open(&dsk_file,"PushingTheEnvelope.dsk",FA_READ);
            if(dsk_mount_fat(0,&dsk_file)){
                ssd_write_text(0,0,false,"dsk loaded ok");
            }
            mou_xreg(0x4000);
            main_run();
            */
        }
    }else{
        if(ext_get_cached(EXT_BTN_A)){
            main_break();
        }
    }
    /*
    //char status[32];
    if(ext_get_cached(EXT_BTN_D)){
        //sprintf((char *)mbuf,"E6C9: 0x%02x%02x%02x%02x%02x%02x",
        //    xram[0xE6C9],xram[0xE6CA],xram[0xE6CB],xram[0xE6CC],xram[0xE6CD],xram[0xE6CE]);
        //sprintf((char *)mbuf,"FFFA: 0x%02x%02x%02x%02x%02x%02x",
        //    xram[0xFFFA],xram[0xFFFB],xram[0xFFFC],xram[0xFFFD],xram[0xFFFE],xram[0xFFFF]);
        sprintf((char *)mbuf,"E4A8: 0x%02x%02x%02x%02x%02x%02x%02x%02x ",
            xram[0xE4A8],xram[0xE4A9],xram[0xE4AA],xram[0xE4AB],xram[0xE4AC],xram[0xE4AD],xram[0xE4AE],xram[0xE4AF]);
        ssd_write_text(0,4,false,(char *)mbuf);
        //sprintf((char *)mbuf,"AR: 0x%08x", ssd_action_rword);
        //ssd_write_text(0,1,!ssd_action_is_wr,(char *)mbuf);
        //sprintf((char *)mbuf,"AW: 0x%08x", ssd_action_word);
        //            xram[0x04FA],xram[0x04FB],xram[0x04FC],xram[0x04FD],xram[0x04FE],xram[0x04FF]);
        //ssd_write_text(0,2,ssd_action_is_wr,(char *)mbuf);
        //sprintf((char *)mbuf,"DSK %02x %02x %02x %02x %02x %02x",
        //    dsk_reg_cmd, dsk_reg_status, dsk_reg_irq, dsk_state, IOREGS(DSK_IO_DRQ), IOREGS(DSK_IO_CMD));
        //ssd_write_text(0,3,false,(char *)mbuf);
    }
    */
/*
    uint8_t rxdata;
    
    int bytes_written;
    int bytes_read;
    {
        uint8_t txdata[] = { I2C_IOEXP_REG_IN };
        bytes_written = i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 1, true);
    }
    bytes_read = i2c_read_blocking(EXT_I2C,I2C_IOEXP_ADDR, &rxdata, 1, false);
    printf("%d %d %02x\n",bytes_written, bytes_read, rxdata);
    

   printf(".\n");
    for(uint8_t i=0x08; i < 0x80; i++){
        int ret = -1;
        ret = i2c_read_blocking(EXT_I2C, i, &rxdata, 1, false);
        if(ret>=0)
        { 
            printf("%02x found\n",i);
        }
    }
*/

}

void ext_put(uint8_t pin, bool value)
{
    if(value){
        ext_port_odata |= pin;
    }else{
        ext_port_odata &= ~pin;
    }
    {
        uint8_t txdata[] = {I2C_IOEXP_REG_OUT, ext_port_odata};
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }
}

void ext_pulse(uint8_t pin)
{
    uint8_t txdata[] = {I2C_IOEXP_REG_OUT, ext_port_odata ^ pin};
    i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, true);
    txdata[1] = ext_port_odata;
    i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
}

void ext_set_dir(uint8_t pin, bool output)
{
#ifdef I2C_IOEXP_OUTPUT_IS_ONE
    if(!output){
        ext_port_dir &= ~pin;
    }else{
        ext_port_dir |= pin;
    }
#else
    if(output){
        ext_port_dir &= ~pin;
    }else{
        ext_port_dir |= pin;
    }
#endif
    {
        uint8_t txdata[] = {I2C_IOEXP_REG_DIR, ext_port_dir};
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 2, false);
    }

}

void ext_update(void){
        uint8_t txdata[] = { I2C_IOEXP_REG_IN };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 1, true);
        i2c_read_blocking(EXT_I2C,I2C_IOEXP_ADDR, &ext_port_idata, 1, false);    
}

bool ext_get_cached(uint8_t pin){
#ifdef I2C_IOEXP_REG_INV
        return((ext_port_idata & pin) != 0x00);
#else
        return(((ext_port_idata ^ I2C_IOEXP_INV) & pin) != 0x00);
#endif
}
 
bool ext_get(uint8_t pin)
{   
    //Read inputs from device
#ifdef I2C_IOEXP_OUTPUT_IS_ONE
    if(!(ext_port_dir & pin)){
#else
    if(exp_port_dir & pin){
#endif
        uint8_t txdata[] = { I2C_IOEXP_REG_IN };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 1, true);
        i2c_read_blocking(EXT_I2C,I2C_IOEXP_ADDR, &ext_port_idata, 1, false);
#ifdef I2C_IOEXP_REG_INV
        return((ext_port_idata & pin) != 0x00);
#else
        return(((ext_port_idata ^ I2C_IOEXP_INV) & pin) != 0x00);
#endif
    //Read outputs from local copy
    }else{
        return((ext_port_odata & pin) != 0x00);
    }
}