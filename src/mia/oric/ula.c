/*
 * Copyright (c) 2025 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Using the ULA (Oric screen) as output device for the monitor */

#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "main.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "oric/ula.h"
#include "oric/font_abus.h"

static uint8_t ula_cur_x;
static uint8_t ula_cur_y;

static bool ula_active = false;

//For simplicity, only use 1kB of FB to fit in mbuf
#define ULA_FB_COLS (40)
#define ULA_FB_ROWS (28)
#define ULA_FB_ADDR (0xBB80)
#define ULA_CH_ADDR (0xB500)

static uint8_t ula_line_buf_y = 0;
static bool ula_line_buf_dirty = false;
extern char ula_line_buf[0x100];
asm(".equ ula_line_buf, 0x20040100");

enum ULA_STATE { 
    IDLE, 
    FONT,
    CLR
};
static enum ULA_STATE ula_state;

typedef enum
{
    ansi_state_C0,
    ansi_state_Fe,
    ansi_state_SS2,
    ansi_state_SS3,
    ansi_state_CSI
} ansi_state_t;

#define ULA_CSI_PARAM_MAX_LEN 4
static ansi_state_t ula_ansi_state;
static uint16_t ula_csi_param[ULA_CSI_PARAM_MAX_LEN];
static uint8_t ula_csi_param_count;

static void ula_stdio_out_chars(const char *buf, int len);
static void ula_stdio_out_flush(void);
static void ula_wait_mia(void){
    do{
        mia_main_task();
    }while(mia_active());
}

void ula_clr_line(uint8_t line){
    char *line_buf = &ula_line_buf[line * ULA_FB_COLS];
    line_buf[0] = 0x02; //ULA Green Ink
    for(uint16_t i=1; i < (ULA_FB_COLS-1); i++){
        line_buf[i] = ' '; //ASCII space
    }
    line_buf[ULA_FB_COLS - 1] = 0x1a; //50Hz Textmode
}

void ula_draw_char(uint8_t x, uint8_t y, bool invert, char ch){

    if(y != ula_line_buf_y){
        ula_stdio_out_flush();
    }
    ula_line_buf_dirty = true;
    ula_line_buf[x] = (invert ? 0x80 | ch : ch);
}

//TODO This is a very poor interpretation of the ANSI escape code, adopted from the com.c module. Good enough for debug?
static void ula_line_state_C0(char ch){
    if (ch == '\r'){
        ula_cur_x = 1;
        ula_cur_y++;
        ula_stdio_out_flush();
    }
    else if (ch == '\33')
        ula_ansi_state = ansi_state_Fe;
    else if (ch == '\b' || ch == 127){ 
        ula_cur_x--;
        ula_draw_char(ula_cur_x, ula_cur_y, false, ' ');
    }
    else if (ch == 1) // ctrl-a
        ula_cur_x = 1;
    else if (ch == 2){ // ctrl-b
        ula_cur_x--;
        ula_draw_char(ula_cur_x, ula_cur_y, false, ' ');
    }    
    else if (ch == 5) // ctrl-e
        ula_cur_x = ULA_FB_COLS-1;
    else if (ch == 6) // ctrl-f
        ula_cur_x++;
    else{
        if(ch >= 32){
            ula_draw_char(ula_cur_x, ula_cur_y, false, ch);
            ula_cur_x++;
        }
    }
}

static void ula_line_state_Fe(char ch)
{
    if (ch == '[')
    {
        ula_ansi_state = ansi_state_CSI;
        ula_csi_param_count = 0;
        ula_csi_param[0] = 0;
    }
    else if (ch == 'b' || ch == 2)
    {
        ula_ansi_state = ansi_state_C0;
        //NOP Don't have a buffer view
        //com_line_backward_word();
    }
    else if (ch == 'f' || ch == 6)
    {
        ula_ansi_state = ansi_state_C0;
        //NOP Don't have a buffer view
        //com_line_forward_word();
    }
    else if (ch == 'N')
        ula_ansi_state = ansi_state_SS2;
    else if (ch == 'O')
        ula_ansi_state = ansi_state_SS3;
    else
    {
        ula_ansi_state = ansi_state_C0;
        if (ch == 127){
            ula_cur_x--;
            ula_draw_char(ula_cur_x, ula_cur_y, false, ' ');
        }        
    }
}

static void ula_line_state_SS2(char ch)
{
    (void)ch;
    ula_ansi_state = ansi_state_C0;
}

static void ula_line_state_SS3(char ch)
{
    ula_ansi_state = ansi_state_C0;
    if (ch == 'F')
        ula_cur_x = ULA_FB_COLS-1;
    else if (ch == 'H')
        ula_cur_x = 1;
}

static void ula_line_state_CSI(char ch)
{
    // Silently discard overflow parameters but still count to + 1.
    if (ch >= '0' && ch <= '9')
    {
        if (ula_csi_param_count < ULA_CSI_PARAM_MAX_LEN)
        {
            ula_csi_param[ula_csi_param_count] *= 10;
            ula_csi_param[ula_csi_param_count] += ch - '0';
        }
        return;
    }
    if (ch == ';' || ch == ':')
    {
        if (++ula_csi_param_count < ULA_CSI_PARAM_MAX_LEN)
            ula_csi_param[ula_csi_param_count] = 0;
        else
            ula_csi_param_count = ULA_CSI_PARAM_MAX_LEN;
        return;
    }
    ula_ansi_state = ansi_state_C0;
    if (++ula_csi_param_count > ULA_CSI_PARAM_MAX_LEN)
        ula_csi_param_count = ULA_CSI_PARAM_MAX_LEN;
    if (ch == 'C')
        ula_cur_x++;
    else if (ch == 'D')
        ula_cur_x--;
    else if (ch == 'F')
        ula_cur_x = ULA_FB_COLS-1;
    else if (ch == 'H')
        ula_cur_x = 0;
    else if (ch == 'b' || ch == 2)
        {}
        //NOP Don't have a buffer view
        //com_line_backward_word();
    else if (ch == 'f' || ch == 6)
        {}
        //NOP Don't have a buffer view
        //com_line_forward_word();
    else if (ch == '~')
        switch (ula_csi_param[0])
        {
        case 1:
        case 7:
            ula_cur_x = 1;
            break;
        case 4:
        case 8:
            ula_cur_x = ULA_FB_COLS-1;
            break;
        case 3:
            ula_cur_x--;
            break;
        }
}

static void ula_scroll(void){
    //Using 135 byte chunks (0x87)
    for(uint16_t addr = ULA_FB_ADDR; addr < (ULA_FB_ADDR +(ULA_FB_COLS*(ULA_FB_ROWS-1))); addr += 0x87){
        mia_read_ula(addr+ULA_FB_COLS,0x87);
        ula_wait_mia();
        mia_write_ula(addr,0x87);
        ula_wait_mia();
    }
    ula_clr_line(0);
    mia_write_ula(ULA_FB_ADDR+(ULA_FB_COLS*(ULA_FB_ROWS-1)), ULA_FB_COLS);
    ula_wait_mia();
}

int ula_putc(char ch){
    if(ch == 0) return 0;
    if(ch == '\30')
        ula_ansi_state = ansi_state_C0;
    else
        switch( ula_ansi_state ){
            case ansi_state_C0:
                ula_line_state_C0(ch);
                break;
            case ansi_state_Fe:
                ula_line_state_Fe(ch);
                break;
            case ansi_state_SS2:
                ula_line_state_SS2(ch);
                break;
            case ansi_state_SS3:
                ula_line_state_SS3(ch);
                break;
            case ansi_state_CSI:
                ula_line_state_CSI(ch);
                break;
        }
    if(ula_cur_x >= ULA_FB_COLS){
        ula_cur_x = 1;
        ula_cur_y++;
        ula_stdio_out_flush();
    }
    if(ula_cur_y >= ULA_FB_ROWS){
        ula_scroll();
        ula_cur_y = ULA_FB_ROWS-1;
    }
    return 0;
}

static void ula_stdio_out_chars(const char *buf, int len)
{
    ula_active = true;
    for(uint16_t i=0; i<len; i++)
    {
        ula_putc(buf[i]);
    }

}

static void ula_stdio_out_flush(void)
{
    mia_write_ula(ULA_FB_ADDR + (ULA_FB_COLS * ula_line_buf_y),ULA_FB_COLS);
    ula_wait_mia();
    ula_line_buf_dirty = false;
    if(ula_cur_y != ula_line_buf_y){
        ula_clr_line(0);
        ula_line_buf_y = ula_cur_y;
    }
}

static stdio_driver_t ula_stdio_app = {
    .out_chars = ula_stdio_out_chars,
    .out_flush = ula_stdio_out_flush,
    .in_chars = 0,
#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
    .set_chars_available_callback = 0,
#endif
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF
#endif
};


void ula_reset(void){
    ula_active = true;
    //Init Oric text mode screen
    assert(sizeof(font_6x8) <= MBUF_SIZE);
    memcpy(mbuf, font_6x8, sizeof(font_6x8));
    mbuf_len = sizeof(font_6x8);
    mia_write_buf(ULA_CH_ADDR);
    ula_state = FONT;
}

void ula_run(void){
    if(ula_active){
        stdio_set_driver_enabled(&ula_stdio_app, false);
        //puts("ula off\n");
        ula_active = false;
    }
}

void ula_task(void){
    if(mia_active()){
        return;
    }
    switch(ula_state){
        case(IDLE):
            //Timely flush dirty buffers to screen
            if(ula_line_buf_dirty){
                ula_stdio_out_flush();
            }
            break;
        case(FONT):
            ula_cur_x = 1;
            ula_cur_y = 0;
            ula_clr_line(0);
            //puts("ula on\n");
            mia_write_ula(ULA_FB_ADDR,ULA_FB_COLS);
            ula_state = CLR;
            break;
        case(CLR):
            if(!mia_active()){
                if(++ula_cur_y < ULA_FB_ROWS){
                    mia_write_ula(ULA_FB_ADDR+(ULA_FB_COLS*ula_cur_y), ULA_FB_COLS);
                }else{
                    ula_cur_y = 0;
                    stdio_set_driver_enabled(&ula_stdio_app, true);
                    ula_putc(']');  //Lost prompt work-around
                    ula_state = IDLE;
                }
            }
            break;
        default:
            ula_state = IDLE;
    }
}