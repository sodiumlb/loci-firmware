#include "main.h"
#include "str.h"
#include <stdio.h>
#include "api/api.h"
#include "sys/cfg.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "pico/stdlib.h"
#include "mia.pio.h"

//MUST be updated if PIO is changed
#define ADJ_MAP_INSTR  (5)
#define ADJ_ACTR_INSTR (2)
#define ADJ_ACTW_INSTR (11)
#define ADJ_ADDR_INSTR (8)
#define ADJ_IO_INSTR   (3)


uint8_t adj_map_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("MAP tune %d\n",delay);
    MIA_MAP_PIO->instr_mem[mia_get_map_prg_offset() + ADJ_MAP_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_io_read_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("IO read tune %d\n",delay);
    MIA_ACT_PIO->instr_mem[mia_get_act_prg_offset() + ADJ_ACTR_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_io_write_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("IO write tune %d\n",delay);
    MIA_ACT_PIO->instr_mem[mia_get_act_prg_offset() + ADJ_ACTW_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_read_addr_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("Addr tune %d\n",delay);
    MIA_READ_PIO->instr_mem[mia_get_read_addr_prg_offset() + ADJ_ADDR_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_io_data_delay(uint8_t delay)
{
    delay &= 0x07;
    printf("IO data tune %d\n",delay);
    MIA_IO_READ_PIO->instr_mem[mia_get_io_read_prg_offset() + ADJ_IO_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

void adj_init(void){
    //Set delays from cfg
    adj_map_delay(cfg_get_map_delay());
    adj_io_write_delay(cfg_get_io_write_delay());
    adj_io_read_delay(cfg_get_io_read_delay());
    adj_io_data_delay(cfg_get_io_data_delay());
    adj_read_addr_delay(cfg_get_read_addr_delay());
}

static enum {
    ADJ_IDLE,
    ADJ_SCAN,
    ADJ_CLEANUP,
} adj_state;

bool adj_active(void){
    return adj_state != ADJ_IDLE;
}
static bool adj_scan_requested = false;

void adj_scan(void){
    adj_scan_requested = true;
}

#define ADJ_SCAN_TIME_US 5000
void adj_task(void){
    static uint8_t tior; 
    static absolute_time_t adj_timer;
    switch(adj_state){
        case(ADJ_IDLE):
            if(adj_scan_requested){
                adj_scan_requested = false;
                adj_state = ADJ_SCAN;
                adj_timer = delayed_by_us(get_absolute_time(), ADJ_SCAN_TIME_US*20);
                tior = 0;
                xram[0xFFF0] = 0x80 | tior;     //Flag scanning by setting MSB
                api_return_released();
                adj_io_read_delay(tior);
            }
            break;
        case(ADJ_SCAN):
            if(absolute_time_diff_us(get_absolute_time(), adj_timer) < 0){
                if(++tior > 31){
                    adj_state = ADJ_CLEANUP;
                }else{
                    adj_timer = delayed_by_us(get_absolute_time(), ADJ_SCAN_TIME_US);
                    adj_io_read_delay(tior);
                    xram[0xFFF0] = 0x80 | tior;
                }
            }
            break;
        case(ADJ_CLEANUP):
            tior = cfg_get_io_read_delay();
            adj_io_read_delay(tior);
            xram[0xFFF0] = tior;
            adj_state = ADJ_IDLE;
            api_zxstack();
            break;
        default:
            break;
    }
}

