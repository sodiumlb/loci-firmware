#include "main.h"
#include "str.h"
#include <stdio.h>
#include "sys/cfg.h"
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
    uint8_t delay2=0;
    if(delay>14)
        delay = 14;
    if(delay>7)
        delay2 = delay - 7;
    printf("Addr tune %d\n",delay);
    MIA_READ_PIO->instr_mem[mia_get_read_addr_prg_offset() + ADJ_ADDR_INSTR + 0] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay & 0x07));
    MIA_READ_PIO->instr_mem[mia_get_read_addr_prg_offset() + ADJ_ADDR_INSTR + 1] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay2));
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