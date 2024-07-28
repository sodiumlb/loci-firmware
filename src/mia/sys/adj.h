#include "main.h"
#include "pico/stdlib.h"

uint8_t adj_map_delay(uint8_t delay);
uint8_t adj_actr_delay(uint8_t delay);
uint8_t adj_actw_delay(uint8_t delay);
uint8_t adj_addr_delay(uint8_t delay);
uint8_t adj_io_read_delay(uint8_t delay);

void adj_mon_map(const char *args, size_t len);
void adj_mon_act_read(const char *args, size_t len);
void adj_mon_act_write(const char *args, size_t len);
void adj_mon_io_read(const char *args, size_t len);
void adj_mon_read_addr(const char *args, size_t len);
