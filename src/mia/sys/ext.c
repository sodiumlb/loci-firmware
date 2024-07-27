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
#include "locifw_version.h"
#include "loci_roms.h"

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
static uint8_t ext_port_idata_prev = 0;
static uint8_t ext_port_dir = I2C_IOEXP_DIR;

#define EXT_BTN_LONGPRESS 2000
static absolute_time_t ext_btn_holdtimer; 


static enum {
    EXT_IDLE,
    EXT_LOADING_DEVROM,
    EXT_LOADING_BIOS,
} ext_state;


void ext_update(void);
bool ext_get_cached(uint8_t pin);
bool ext_btn_pressed(uint8_t pin);
bool ext_btn_released(uint8_t pin);

void ext_embedded_rom(const uint8_t* data, const uint32_t len, uint16_t xram_addr){
    memcpy((void*)(&xram[xram_addr]), (void*)data, len);
}

void ext_patch_version(void){
    //TODO safer decision for patching in version number
    if(xram[0xFFF7]==0xF0 && xram[0xFFF8]==0xF1 && xram[0xFFF9]==0xF2){
        #ifdef LOCIFW_VERSION
        {
            xram[0xFFF7] = LOCIFW_VERSION_PATCH;
            xram[0xFFF8] = LOCIFW_VERSION_MINOR;
            xram[0xFFF9] = LOCIFW_VERSION_MAJOR;
        }
        #else
        {
            //TODO Set date instead?
            xram[0xFFF7] = 0;
            xram[0xFFF8] = 0;
            xram[0xFFF9] = 0;
        }
        #endif
    }
}

void ext_init(void)
{

    i2c_init(EXT_I2C, 1200*1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

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
    ext_update();
    ext_port_idata_prev = ext_port_idata;
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

#define EXT_I2C_SAMPLE_DELAY 0x1000;
uint16_t ext_refresh_cnt = EXT_I2C_SAMPLE_DELAY;

void ext_task(void)
{
    switch(ext_state){
        case EXT_LOADING_DEVROM:
            if(!rom_active()){
                if(!rom_load("BASIC11",7)){
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
                ext_patch_version();
                main_run();
              }
            break;
    }

    if(ext_refresh_cnt--)
        return;
    ext_refresh_cnt = EXT_I2C_SAMPLE_DELAY;

    ext_update();

    if(ext_btn_pressed(EXT_BTN_A)){
        ext_btn_holdtimer = delayed_by_ms(get_absolute_time(), EXT_BTN_LONGPRESS);
    }

    if(!main_active()){
        if(ext_btn_released(EXT_BTN_A)){
            if(absolute_time_diff_us(ext_btn_holdtimer, get_absolute_time()) < 0){
                rom_mon_load("locirom.rp6502", 14);    //First ROM priority: USB storage
                if(!rom_active()){
                    rom_load("LOCIROM",7);             //Second ROM priority: new name in flash
                }
                if(rom_active()){
                    ext_state = EXT_LOADING_BIOS;
                }
#ifdef EMBEDDED_LOCIROM
                else{
                    ext_embedded_rom(locirom, locirom_size, 0xC000);
                    ext_patch_version();
                    main_run();
                }
#endif
            }
        }
    }else{
        if(ext_btn_released(EXT_BTN_A)){
            main_break();
        }
    }
    if(ext_get_cached(EXT_BTN_A) && (absolute_time_diff_us(ext_btn_holdtimer, get_absolute_time()) > 0)){
        //Longpress function here
    }
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
        ext_port_idata_prev = ext_port_idata;
        uint8_t txdata[] = { I2C_IOEXP_REG_IN };
        i2c_write_blocking(EXT_I2C,I2C_IOEXP_ADDR, txdata, 1, true);
        i2c_read_blocking(EXT_I2C,I2C_IOEXP_ADDR, &ext_port_idata, 1, false);
#ifndef I2C_IOEXP_REG_INV
        ext_port_idata ^= I2C_IOEXP_INV;    
#endif
}

bool ext_get_cached(uint8_t pin){
        return((ext_port_idata & pin) != 0x00);
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
#ifndef I2C_IOEXP_REG_INV
        ext_port_idata ^= I2C_IOEXP_INV;
#endif
        return((ext_port_idata & pin) != 0x00);
    //Read outputs from local copy
    }else{
        return((ext_port_odata & pin) != 0x00);
    }
}

bool ext_btn_pressed(uint8_t pin){
    return !!((ext_port_idata & ~ext_port_idata_prev) & pin);
}
bool ext_btn_released(uint8_t pin){
    return !!((~ext_port_idata & ext_port_idata_prev) & pin);
}