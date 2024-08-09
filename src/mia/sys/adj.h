#include "main.h"
#include "pico/stdlib.h"

uint8_t adj_map_delay(uint8_t delay);
uint8_t adj_io_read_delay(uint8_t delay);
uint8_t adj_io_write_delay(uint8_t delay);
uint8_t adj_io_data_delay(uint8_t delay);
uint8_t adj_read_addr_delay(uint8_t delay);

void adj_init(void);
void adj_scan(void);
void adj_task(void);