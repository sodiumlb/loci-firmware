#include "main.h"
#include "str.h"
#include "pico/stdlib.h"
#include "mia.pio.h"

//MUST be updated if PIO is changed
#define ADJ_MAP_INSTR  (5)
#define ADJ_ACTR_INSTR (mia_map_program.length + 2)
#define ADJ_ACTW_INSTR (mia_map_program.length + 10)
#define ADJ_ADDR_INSTR (mia_read_data_program.length + 0)
#define ADJ_IO_INSTR   (mia_read_data_program.length + mia_read_addr_program.length + 3)


uint8_t adj_map_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("MAP tune %d\n",delay);
    MIA_MAP_PIO->instr_mem[ADJ_MAP_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_actr_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("ACT read tune %d\n",delay);
    MIA_ACT_PIO->instr_mem[ADJ_ACTR_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_actw_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("ACT write tune %d\n",delay);
    MIA_ACT_PIO->instr_mem[ADJ_ACTW_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_addr_delay(uint8_t delay)
{
    delay &= 0x1f;
    printf("Addr tune %d\n",delay);
    MIA_READ_PIO->instr_mem[ADJ_ADDR_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

uint8_t adj_io_read_delay(uint8_t delay)
{
    delay &= 0x07;
    printf("IO read tune %d\n",delay);
    MIA_READ_PIO->instr_mem[ADJ_IO_INSTR] = (uint16_t)(pio_encode_nop() | pio_encode_delay(delay));
    return delay;
}

//Monitor command functions for tmap tior tiow tiod taddr

void adj_mon_map(const char *args, size_t len){
    uint32_t delay;
    if (len)
    {
        if (!parse_uint32(&args, &len, &delay) ||
            !parse_end(args, len))
        {
            printf("?invalid argument\n");
            return;
        }
        adj_map_delay(delay);
    }
}
void adj_mon_act_read(const char *args, size_t len){
    uint32_t delay;
    if (len)
    {
        if (!parse_uint32(&args, &len, &delay) ||
            !parse_end(args, len))
        {
            printf("?invalid argument\n");
            return;
        }
        adj_actr_delay(delay);
    }
}
void adj_mon_act_write(const char *args, size_t len){
    uint32_t delay;
    if (len)
    {
        if (!parse_uint32(&args, &len, &delay) ||
            !parse_end(args, len))
        {
            printf("?invalid argument\n");
            return;
        }
        adj_actw_delay(delay);
    }
}
void adj_mon_io_read(const char *args, size_t len){
    uint32_t delay;
    if (len)
    {
        if (!parse_uint32(&args, &len, &delay) ||
            !parse_end(args, len))
        {
            printf("?invalid argument\n");
            return;
        }
        adj_io_read_delay(delay);
    }
}
void adj_mon_read_addr(const char *args, size_t len){
    uint32_t delay;
    if (len)
    {
        if (!parse_uint32(&args, &len, &delay) ||
            !parse_end(args, len))
        {
            printf("?invalid argument\n");
            return;
        }
        adj_addr_delay(delay);
    }
}