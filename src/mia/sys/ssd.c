/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "sys/ssd.h"
#include "sys/ext.h"
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include <stdio.h>
#include <stddef.h>
//#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "sys/font_ssd_rot.h"

static uint8_t ssd_i2c_addr;
static uint8_t ssd_ssd1306_addrs[] = { 0x3C, 0x3D };

//On screen width. Assumin 8 wide on file for now
#define SSD_FONT_WIDTH (6)
#define SSD_FB_WIDTH 128
#define SSD_FB_HEIGHT 64
#define SSD_FB_COLS (SSD_FB_WIDTH/SSD_FONT_WIDTH)
#define SSD_FB_ROWS (SSD_FB_HEIGHT/8)

static uint8_t ssd_framebuffer_tx[1025];
//extern volatile uint8_t ssd_framebuffer_tx[1025];
//asm(".equ ssd_framebuffer_tx, 0x20001000");

static bool ssd_is_present = false;
static volatile uint8_t *ssd_framebuffer = &ssd_framebuffer_tx[1];
static bool ssd_need_refresh = false;
volatile bool ssd_got_action_word = false;
volatile uint32_t ssd_action_word;
volatile uint32_t ssd_action_rword;
volatile bool ssd_action_is_wr = false;


void ssd_tx_framebuffer(void){
    if(ssd_need_refresh && ssd_is_present){
        ssd_framebuffer_tx[0] = 0x40;
        i2c_write_blocking(EXT_I2C, ssd_i2c_addr, ssd_framebuffer_tx, 1025, false);
        ssd_need_refresh = false;
    }
    /*
    uint8_t *p = ssd_framebuffer;
    uint8_t txdata[17];
    txdata[0] = 0x40;
    for(int i=0; i < 64; i++){
        for(int j=0; j < 16; j++){
            txdata[j+1] = p[j];
        }
        //i2c_write_raw_blocking(EXT_I2C, txdata, 1);
        //i2c_write_blocking(EXT_I2C, SSD1306_ADDR, txdata, 1, true);
        i2c_write_blocking(EXT_I2C, SSD1306_ADDR, txdata, 17, false);
        p += 16;
    }
    */
}

void ssd_clear(void){
    for(uint16_t i=0; i < 1024; i++){
        ssd_framebuffer[i] = 0x00;
    }
}
//x and y are character columnt and row coordinates, not pixel coordinates
void ssd_draw_char(uint16_t x, uint16_t y, bool invert, char c){
    if((x * y) > ((SSD_FB_ROWS * SSD_FB_COLS))){
        return;
    }
    if(c < 32){
        return;
    }
    uint16_t glyph_offs = ((uint16_t)c - 32) * 8;
    for(uint16_t i=0; i<SSD_FONT_WIDTH; i++){
        uint8_t stripe = font_8x8[glyph_offs + i];
        if(invert){
            stripe = ~stripe;
        }
        ssd_framebuffer[(y*SSD_FB_WIDTH) + (x * SSD_FONT_WIDTH) + i] = stripe;
    }
    ssd_need_refresh = true;
}

void ssd_write_text(uint8_t x, uint8_t y, bool invert, char *text){
    for(uint16_t i=0; (i+x) < (SSD_FB_COLS); i++){
        if(text[i]==0){ return; }
        ssd_draw_char(i+x, y, invert, text[i]);
    }
}

static uint16_t ssd_cur_x = 0;
static uint16_t ssd_cur_y = 0;

int ssd_putc(char ch){
    if(ch == 0) return 0;
    if(ch == '\n'){
        ssd_cur_x = 0;
        ssd_cur_y++;
    }else{
        ssd_draw_char(ssd_cur_x, ssd_cur_y, false, ch);
        ssd_cur_x++;
    }
    if(ssd_cur_x >= SSD_FB_COLS){
        ssd_cur_x = 0;
        ssd_cur_y++;
    }
    if(ssd_cur_y >= SSD_FB_ROWS){
        ssd_cur_y = 0;
    }
    return 0;
}

static void ssd_stdio_out_chars(const char *buf, int len)
{
    for(uint16_t i=0; i<len; i++)
    {
        ssd_putc(buf[i]);
    }
    ssd_tx_framebuffer();
}

static stdio_driver_t ssd_stdio_app = {
    .out_chars = ssd_stdio_out_chars,
    .in_chars = 0,
#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
    .set_chars_available_callback = 0,
#endif
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF
#endif
};

const uint8_t __in_flash() ssd_init_commands[] = {
        0x80, 0x8d, 0x80, 0x14,             //Charge pump regulator
        0x80, 0xaf,                         //Display on
        0x80, 0x20, 0x80, 0x00,             //Addressing mode Horizontal
        0x80, 0x21, 0x80, 0x00, 0x80, 0x7F, //Column address
        0x80, 0x22, 0x80, 0x00, 0x80, 0x07, //Page address
        0x80, 0xC8,                         //COM direction swap
        0x80, 0xA1                          //Segment remap
    };

void ssd_init(void)
{

    ssd_clear();
/*
    for(uint8_t i=0; i < sizeof(init_commands); i++){
        uint8_t txdata[] = { 0x80, init_commands[i] };
        i2c_write_blocking(EXT_I2C, SSD1306_ADDR, txdata, 2, false);
    }
*/
    uint8_t rxdata;
    ssd_is_present = false;
    for(unsigned int i=0; i < sizeof(ssd_ssd1306_addrs); i++){
        if(i2c_read_blocking(EXT_I2C, ssd_ssd1306_addrs[i], &rxdata, 1, false) > 0){
            ssd_is_present = true;
            ssd_i2c_addr = ssd_ssd1306_addrs[i];
            break;    
        }
    }
    if(ssd_is_present){
        i2c_write_blocking(EXT_I2C, ssd_i2c_addr, ssd_init_commands, 26, false);
        ssd_write_text(0,0,true, RP6502_NAME " " RP6502_VERSION);
        //ssd_write_text(0,1,false, "ABCDEFGHIJKLMNOPQRSTUV");
        //ssd_write_text(0,2,false, "WXYZ01234567890abcdefg");
        //ssd_write_text(0,3,false, "hijklmnopqrstuvwxyz");
        //ssd_write_text(0,4,false, "ijklmnopqrstuvw");
        //ssd_write_text(0,6,false, "The quick brown fox");
        //ssd_write_text(0,7,false, "jumps over the lazy dog");
        //ssd_write_text(10,4,false, "abcdefghijklmnopqrst");
        //ssd_write_text(0,5,true, "<zxcæøåqwertyuiopå");
        ssd_tx_framebuffer();
        stdio_set_driver_enabled(&ssd_stdio_app, true);
    }
}

void ssd_task(void)
{
    if(ssd_got_action_word){
        printf("##%08lx##",ssd_action_word);
        ssd_got_action_word = false;
    }
    ssd_tx_framebuffer();
/*
    
    int bytes_written;
    int bytes_read;
    {
        uint8_t txdata[] = { PCA9557_REG_IN };
        bytes_written = i2c_write_blocking(EXT_I2C,PCA9557_ADDR, txdata, 1, true);
    }
    bytes_read = i2c_read_blocking(EXT_I2C,PCA9557_ADDR, &rxdata, 1, false);
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
