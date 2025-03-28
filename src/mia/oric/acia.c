/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Emulating 6551 ACIA for modem connections */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "tusb.h"
#include "sys/ext.h"
#include "sys/mem.h"
#include "usb/cdc.h"
#include "oric/acia.h"

#define ACIA_CMD_DTR 0b00000001
#define ACIA_CMD_IRQ 0b00000010
#define ACIA_CMD_TXC 0b00001100
#define ACIA_CMD_ECH 0b00010000
#define ACIA_CMD_PEN 0b00100000
#define ACIA_CMD_PAR 0b11000000

#define ACIA_TXC_OFF 0b00000000
#define ACIA_TXC_TXI 0b00000100
#define ACIA_TXC_TXO 0b00001000
#define ACIA_TXC_BRK 0b00001100

#define ACIA_PAR_ODD 0b00000000
#define ACIA_PAR_EVN 0b01000000
#define ACIA_PAR_MRK 0b10000000
#define ACIA_PAR_SPC 0b11000000

#define ACIA_CTRL_BAUD 0b00001111
#define ACIA_CTRL_CLK  0b00010000
#define ACIA_CTRL_DATA 0b01100000
#define ACIA_CTRL_STOP 0b10000000

#define ACIA_STAT_PAR_ERR 0b00000001
#define ACIA_STAT_FRM_ERR 0b00000010
#define ACIA_STAT_OVR_ERR 0b00000100
#define ACIA_STAT_RX_FULL 0b00001000
#define ACIA_STAT_TX_EMPTY 0b00010000
#define ACIA_STAT_NOT_DCD  0b00100000
#define ACIA_STAT_NOT_DSR  0b01000000
#define ACIA_STAT_IRQ      0b10000000

volatile acia_io_t* acia_io;
volatile uint8_t acia_stat_base;
volatile uint8_t acia_stat_rx;
volatile uint8_t acia_stat_tx;
volatile uint8_t acia_stat_irq;
#define ACIA_UPDATE_STAT  acia_io->stat = acia_stat_base | acia_stat_tx | acia_stat_rx | acia_stat_irq

static int acia_dev;
volatile uint8_t acia_tx_data;
volatile bool acia_cmd_todo;
volatile bool acia_ctrl_todo;

static bool acia_tx_irq_enable;
static bool acia_rx_irq_enable;
static bool acia_tx_brk_enable;
static bool acia_tx_echo_enable;
static uint8_t acia_data_bits;
static uint32_t acia_baud_rate;
static cdc_line_coding_parity_t acia_parity;
static cdc_line_coding_stopbits_t acia_stop_bits;
static cdc_control_line_state_t acia_line_state_dtr;
static cdc_control_line_state_t acia_line_state_rts;

bool acia_do_cmd(uint8_t cmd);
void acia_do_ctrl(uint8_t ctrl);

#define ACIA_RX_BUFFER_SIZE 32
#define ACIA_RX_BUFFER_IDX_MASK 0x1F
static uint8_t acia_rx_buffer[ACIA_RX_BUFFER_SIZE];
volatile uint8_t acia_rx_buffer_head;
volatile uint8_t acia_rx_buffer_tail;

void acia_set_base_status(uint8_t bit_mask, bool set){
    if(set){
        acia_stat_base |= bit_mask;
    }else{
        acia_stat_base &= ~bit_mask;
    }
}

bool acia_get_base_status(uint8_t bit){
    return(!!(acia_stat_base & bit));
}

/* Kernel events
 */

void acia_init(void){
    acia_io = (acia_io_t*)(&IOREGS(0x380));
    acia_reset(true);
    acia_dev = -1;
    acia_rx_buffer_head = 0;
    acia_rx_buffer_tail = 0;
}

void acia_task(void){
    static uint8_t cnt = 0;
    bool fire_irq = false;
    if(acia_dev < 0){
        for(uint8_t i=0; i < CFG_TUH_CDC; i++){
        //Only hooking up to CDC devices with AT commands aka modems
        //TODO make configurable what CDC device to "mount"
            if(tuh_cdc_mounted(i) && cdc_is_modem(i)){
                acia_dev = i;
                //printf("ACIA on %d\n", acia_dev);
                acia_set_base_status(ACIA_STAT_NOT_DSR,false);
                acia_set_base_status(ACIA_STAT_NOT_DCD,false);
                if(acia_line_state_dtr != 0){
                    fire_irq = true;
                }
                break;
            }
        }
        return;
    }

    if(!tuh_cdc_mounted(acia_dev)){
        printf("ACIA lost\n");
        acia_dev = -1;
        acia_rx_buffer_head = 0;
        acia_rx_buffer_tail = 0;
        acia_set_base_status((ACIA_STAT_NOT_DSR | ACIA_STAT_NOT_DCD) ,true);
        if(acia_line_state_dtr != 0){
            fire_irq = true;
        }
        return;
    }
    if(acia_line_state_dtr != 0){
        if(!acia_stat_tx){
            if(tuh_cdc_write_available(acia_dev)){
                if(tuh_cdc_write(acia_dev, (void*)&acia_tx_data, 1)){
                    tuh_cdc_write_flush(acia_dev);
                    __dmb();
                    acia_stat_tx = ACIA_STAT_TX_EMPTY;
                    if(acia_tx_echo_enable){
                        acia_io->data = acia_tx_data;
                        __dmb();
                        acia_stat_rx = ACIA_STAT_RX_FULL;
                        if(acia_rx_irq_enable){
                            fire_irq = true;
                        }
                    }else if(acia_tx_irq_enable){
                        fire_irq = true;
                    }
                }
            }
        }
        uint32_t acia_rx_available = tuh_cdc_read_available(acia_dev);
        if(acia_rx_available > 0){
            uint8_t len = acia_rx_buffer_tail - acia_rx_buffer_head - 2; //Only overlap when empty (keep 1 extra distance)
            len &= ACIA_RX_BUFFER_IDX_MASK; //Truncate len
            if(len > (ACIA_RX_BUFFER_SIZE - acia_rx_buffer_head))       //Truncate wrapping loads
                len = ACIA_RX_BUFFER_SIZE - acia_rx_buffer_head;
            acia_rx_buffer_head += tuh_cdc_read(acia_dev, &acia_rx_buffer[acia_rx_buffer_head], len);
            acia_rx_buffer_head &= ACIA_RX_BUFFER_IDX_MASK; //Wrap index
        }
        if(!acia_stat_rx){
            if(acia_rx_buffer_head != acia_rx_buffer_tail){
                acia_io->data = acia_rx_buffer[acia_rx_buffer_tail];
                acia_rx_buffer_tail = (acia_rx_buffer_tail + 1) & ACIA_RX_BUFFER_IDX_MASK;
                __dmb();
                acia_stat_rx = ACIA_STAT_RX_FULL;
                if(acia_rx_irq_enable){
                    fire_irq = true;
                }
                cnt = 255;
            }
        }else{
            if(0 == --cnt && acia_rx_irq_enable){
                fire_irq = true;
            }
        }
    }
    if(acia_cmd_todo){
        //printf("ACIA cmd %02x\n", acia_io->cmd);
        bool cmd_fire_irq = acia_do_cmd(acia_io->cmd);
        fire_irq = fire_irq || cmd_fire_irq;
        acia_cmd_todo = false;
    }
    if(acia_ctrl_todo){
        //printf("ACIA ctrl %02x\n", acia_io->ctrl);
        acia_do_ctrl(acia_io->ctrl);
        acia_ctrl_todo = false;
    }

    __dmb();
    if(fire_irq){
        acia_stat_irq = ACIA_STAT_IRQ;
        ext_put(EXT_IRQ,true);
        acia_io->stat = acia_stat_base | acia_stat_tx | acia_stat_rx | ACIA_STAT_IRQ;   //Race free irq assign
        __dmb();
        ext_put(EXT_IRQ,false);
    }else{
        ACIA_UPDATE_STAT;
    }
}


void acia_stop(){
    //printf("acia_stop()\n");
    acia_dev = -1;
    acia_rx_buffer_head = 0;
    acia_rx_buffer_tail = 0;
    acia_reset(true);
}

bool acia_do_cmd(uint8_t cmd){
    bool fire_irq = false;
    if(!!(cmd & ACIA_CMD_DTR)){
        acia_line_state_dtr = CDC_CONTROL_LINE_STATE_DTR;
        if(acia_get_base_status(ACIA_STAT_NOT_DCD))
            fire_irq = true;
        acia_set_base_status(ACIA_STAT_NOT_DCD,false);
    }else{
        acia_line_state_dtr = 0;
        acia_set_base_status(ACIA_STAT_NOT_DCD,true);
    }
    acia_rx_irq_enable = !(cmd & ACIA_CMD_IRQ);
    
    if(!!(cmd & ACIA_CMD_TXC))
        acia_line_state_rts = CDC_CONTROL_LINE_STATE_RTS;
    else
        acia_line_state_rts = 0;

    acia_tx_irq_enable = (cmd & ACIA_CMD_TXC) == 0x04;
    acia_tx_brk_enable = (cmd & ACIA_CMD_TXC) == 0x0C;
    acia_tx_echo_enable = !!(cmd & ACIA_CMD_ECH);
    
    if(!!(cmd & ACIA_CMD_PEN)){
        switch(cmd & ACIA_CMD_PAR){
            case(ACIA_PAR_ODD):
                acia_parity = CDC_LINE_CODING_PARITY_ODD;
                break;
            case(ACIA_PAR_EVN):
                acia_parity = CDC_LINE_CODING_PARITY_EVEN;
                break;
            case(ACIA_PAR_MRK):
                acia_parity = CDC_LINE_CODING_PARITY_MARK;
                break;
            case(ACIA_PAR_SPC):
                acia_parity = CDC_LINE_CODING_PARITY_SPACE;
                break;
            default:
                break;
        }
    }else{
        acia_parity = CDC_LINE_CODING_PARITY_NONE;
    }
    tuh_cdc_set_control_line_state(acia_dev, acia_line_state_dtr | acia_line_state_rts, NULL, 0);
    tuh_cdc_set_data_format(acia_dev, acia_stop_bits, acia_parity, acia_data_bits, NULL, 0);
    return fire_irq;
}

void acia_do_ctrl(uint8_t ctrl){
    switch(ctrl & ACIA_CTRL_BAUD){
        case(0b0000):
            acia_baud_rate = 115200;  //w65c51 style
            break;
        case(0b0001):
            acia_baud_rate = 50;
            break;
        case(0b0010):
            acia_baud_rate = 75;
            break;
        case(0b0011):
            acia_baud_rate = 110; //109.92
            break;
        case(0b0100):
            acia_baud_rate = 135; //134.58
            break;
        case(0b0101):
            acia_baud_rate = 150;
            break;
        case(0b0110):
            acia_baud_rate = 300;
            break;
        case(0b0111):
            acia_baud_rate = 600;
            break;
        case(0b1000):
            acia_baud_rate = 1200;
            break;
        case(0b1001):
            acia_baud_rate = 1800;
            break;
        case(0b1010):
            acia_baud_rate = 2400;
            break;
        case(0b1011):
            acia_baud_rate = 3600;
            break;
        case(0b1100):
            acia_baud_rate = 4800;
            break;
        case(0b1101):
            acia_baud_rate = 7200;
            break;
        case(0b1110):
            acia_baud_rate = 9600;
            break;
        case(0b1111):
            acia_baud_rate = 19200;
            break;
        default:
            break;
    }
    //Ignoring ACIA_CTRL_CLK for now, assume baud is 0 when ext clk is used
    acia_data_bits = 8 - ((ctrl & ACIA_CTRL_DATA)>>5);
    if(!!(ctrl & ACIA_CTRL_STOP)){
        if(acia_data_bits == 5 && acia_parity != CDC_LINE_CODING_PARITY_NONE){
            acia_stop_bits = CDC_LINE_CODING_STOP_BITS_1_5;
        }else if(acia_data_bits == 8 && acia_parity != CDC_LINE_CODING_PARITY_NONE){
            acia_stop_bits = CDC_LINE_CODING_STOP_BITS_1;
        }else{
            acia_stop_bits = CDC_LINE_CODING_STOP_BITS_2;
        }

    }else{
        acia_stop_bits = CDC_LINE_CODING_STOP_BITS_1;
    }
    tuh_cdc_set_baudrate(acia_dev, acia_baud_rate, NULL, 0);
    tuh_cdc_set_data_format(acia_dev, acia_stop_bits, acia_parity, acia_data_bits, NULL, 0);
}


/* Action loop interface functions. Time critical. Runs on other core */

void __not_in_flash() acia_reset(bool hw_reset){
    if(hw_reset){
        // IOREGS(ACIA_IO_DATA) = 0;
        // IOREGS(ACIA_IO_STAT) = ACIA_STAT_TX_EMPTY | ACIA_STAT_NOT_DSR | ACIA_STAT_NOT_DCD;
        // IOREGS(ACIA_IO_CMD)  = ACIA_CMD_IRQ;
        // IOREGS(ACIA_IO_CTRL) = 0;
        acia_io->regs = (ACIA_CMD_IRQ << 16);
        acia_stat_irq = 0x00;
        acia_stat_rx = 0x00;
        acia_stat_tx = ACIA_STAT_TX_EMPTY;
        acia_stat_base = ACIA_STAT_NOT_DSR | ACIA_STAT_NOT_DCD;
    }else{
        acia_stat_base &= ~ACIA_STAT_OVR_ERR;
        acia_io->data = 0;
        acia_io->cmd  = (acia_io->cmd & 0xe0) | ACIA_CMD_IRQ;
    }
    ACIA_UPDATE_STAT;
}

void __not_in_flash() acia_read(void){
    //acia_io->stat &= ~ACIA_STAT_RX_FULL;
    __dmb();
    acia_stat_rx = 0x00;
    ACIA_UPDATE_STAT;
}

void __not_in_flash() acia_clr_irq(void){
    //acia_io->stat &= ~ACIA_STAT_IRQ;
    __dmb();
    acia_stat_irq = 0x00;
    ACIA_UPDATE_STAT;
}

void __not_in_flash() acia_write(uint8_t data){
    acia_tx_data = data;
    __dmb();
    acia_stat_tx = 0x00;
    ACIA_UPDATE_STAT;
}

void __not_in_flash() acia_cmd(uint8_t data){
    acia_io->cmd = data;
    __dmb();
    acia_cmd_todo = true;
}
void __not_in_flash() acia_ctrl(uint8_t data){
    acia_io->ctrl = data;
    __dmb();
    acia_ctrl_todo = true;
}
