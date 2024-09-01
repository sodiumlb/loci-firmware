/*
 * Copyright (c) 2023 Rumbledethumps
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "api/api.h"
#include "mon/rom.h"
#include "sys/com.h"
#include "sys/cpu.h"
#include "sys/ext.h"
#include "sys/led.h"
//#include "sys/pix.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "sys/ssd.h"
#include "mia.pio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "littlefs/lfs_util.h"
#include "oric/acia.h"
#include "oric/dsk.h"
#include "oric/tap.h"

#define MIA_WATCHDOG_MS 250

//volatile uint8_t mia_iopage_enable_map[64] __attribute__((aligned(64)));
static uint64_t mia_iopage_enable_map;

static enum state {
    action_state_idle = 0,
    action_state_read,
    action_state_write,
    action_state_verify,
} volatile action_state = action_state_idle;
static absolute_time_t action_watchdog_timer;
static volatile int32_t action_result = -1;
static int32_t saved_reset_vec = -1;
static uint16_t rw_addr;
static volatile int32_t rw_pos;
static volatile int32_t rw_end;
static volatile bool irq_enabled;
static volatile bool reset_requested;

#define MIA_BOOTSET_FDC 0x01
#define MIA_BOOTSET_TAP 0x02
#define MIA_BOOTSET_B11 0x04
#define MIA_BOOTSET_TAP_BIT 0x08
#define MIA_BOOTSET_FAST 0x80

static uint8_t mia_boot_settings;

static volatile uint32_t mia_io_errors;

/*
void mia_trigger_irq(void)
{
    if (irq_enabled & 0x01)
        ext_put(EXT_IRQ, true);
}
*/
uint32_t mia_buf_crc32(void)
{
    // use littlefs library
    return ~lfs_crc(~0, mbuf, mbuf_len);
}

// The PIO will notify the action loop of all register writes.
// Only every fourth register (0, 4, 8, ...) is watched for
// read access. This additional read address to be 
// is varied based on the state of the RIA.
static void mia_set_watch_address(uint32_t addr)
{
    //pio_sm_put(RIA_ACT_PIO, RIA_ACT_SM, addr & 0x1F);
}
// The PIO will notify the action loop of all register writes.
// Only every fourth register (0, 4, 8, ...) is watched for
// read access. This additional read address to be watched
// is varied based on the state of the RIA.
void mia_set_rom_read_enable(bool enable)
{
    pio_sm_set_enabled(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, enable);
    //pio_sm_set_enabled(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, enable);
    /*
    if(enable)
        pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0xFFFFFFFF);
    else
        pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0x00000000);
    */
}

static enum {
    MIA_IDLE,
    MIA_LOADING_DEVROM,
    MIA_LOADING_BIOS,
} mia_state;


void mia_run(void)
{

    ext_put(EXT_ROMDIS,true);
    ext_put(EXT_nOE,false);
    //printf("**mia_run()**");
    //ext_set_dir(EXT_ROMDIS,true);
    //mia_set_rom_read_enable(true);
    //mia_set_rom_ram_enable(true,false);
    mia_set_rom_ram_enable(!!(mia_boot_settings & MIA_BOOTSET_FDC),!(mia_boot_settings & MIA_BOOTSET_FDC));
    //ext_put(EXT_OE,true);
    //ext_put(EXT_nRESET,true);
  
    mia_io_errors = 0;
    reset_requested = false;

    //mia_set_watch_address(0xFFE2);
    //Normal emulated boot
    if (action_state == action_state_idle)
        return;

    //Special LOCI access to Oric RAM actions
    
    action_result = -1;
    saved_reset_vec = XRAMW(0xFFFC);
    XRAMW(0xFFFC) = 0x03B0;
    action_watchdog_timer = delayed_by_us(get_absolute_time(),
                                          cpu_get_reset_us() +
                                              MIA_WATCHDOG_MS * 1000);
    switch (action_state)
    {
    case action_state_write:
        // Self-modifying fast load
        // FFF0  A9 00     LDA #$00
        // FFF2  8D 00 00  STA $0000
        // FFF5  80 F9     BRA $FFF0
        // FFF7  80 FE     BRA $FFF7
        //Oric mod: CLV + BVC to get BRA 
        // 03B0  B8        CLV
        // 03B1  A9 00     LDA #$00
        // 03B3  8D 00 00  STA $0000
        // 03B6  50 F9     BVC $03B1
        // 03B8  50 FE     BVC $03B8
        //mia_set_watch_address(0xFFF6);
        IOREGS(0x03B0) = 0xB8;
        IOREGS(0x03B1) = 0xA9;
        IOREGS(0x03B2) = mbuf[0];
        IOREGS(0x03B3) = 0x8D;
        IOREGS(0x03B4) = rw_addr & 0xFF;
        IOREGS(0x03B5) = rw_addr >> 8;
        IOREGS(0x03B6) = 0x50;
        IOREGS(0x03B7) = 0xF9;
        IOREGS(0x03B8) = 0x50;
        IOREGS(0x03B9) = 0xFE;
        break;
    case action_state_read:
    case action_state_verify:
        // Self-modifying fast load
        // FFF0  AD 00 00  LDA $0000
        // FFF3  8D FC FF  STA $FFFC/$FFFD
        // FFF6  80 F8     BRA $FFF0
        //Oric mod: CLV + BVC to get BRA 
        // 03B0  B8        CLV
        // 03B1  AD 00 00  LDA $0000
        // 03B4  8D BC 03  STA $03BC/$03BD
        // 03B7  50 F8     BVC $03B1
        IOREGS(0x03B0) = 0xB8;
        IOREGS(0x03B1) = 0xAD;
        IOREGS(0x03B2) = rw_addr & 0xFF;
        IOREGS(0x03B3) = rw_addr >> 8;
        IOREGS(0x03B4) = 0x8D;
        IOREGS(0x03B5) = (action_state == action_state_verify) ? 0xBC : 0xBD;
        IOREGS(0x03B6) = 0x03;
        IOREGS(0x03B7) = 0x50;
        IOREGS(0x03B8) = 0xF8;
        break;
    default:
        break;
    }
}

void mia_stop(void)
{
    irq_enabled = false;
    ext_put(EXT_IRQ, false);
    ext_put(EXT_ROMDIS,false);
    //ext_set_dir(EXT_ROMDIS, false);
    ext_put(EXT_nOE,true);
    mia_set_rom_ram_enable(false,false);
    //mia_set_rom_read_enable(false);
    mia_boot_settings = 0x00;
    action_state = action_state_idle;
    if (saved_reset_vec >= 0)
    {
        XRAMW(0xFFFC) = saved_reset_vec;
        saved_reset_vec = -1;
    }
}

bool mia_active(void)
{
    return action_state != action_state_idle;
}

bool mia_boot_active(void)
{
    return mia_state != MIA_IDLE;
}

/* Basic 1.1 CLOAD read byte patch
A9 01                LDA #$01
8D 15 03             STA $0315
AD 15 03   WAIT:     LDA $0315
D0 FB                BNE WAIT
AD 17 03             LDA $0317
85 2F                STA $2F
60                   RTS
*/
#define CLOAD_PATCH_11_ADDR (0xE6C9)
const uint8_t __in_flash() mia_cload_patch_11[] = {
    0xA9, 0x01, 0x8D, 0x15, 0x03, 0xAD, 0x15, 0x03,
    0xD0, 0xFB, 0xAD, 0x17, 0x03, 0x85, 0x2F, 0x60
};

/* Basic 1.1 CLOAD synch patch
4C 4D E7             JMP $E74D
*/
#define SYNCH_PATCH_11_ADDR (0xE735)
const uint8_t __in_flash() mia_synch_patch_11[] = {
    0x4C, 0x4D, 0xE7 
};

/* Basic 1.1 CLOAD read bit patch
48                   PHA
A9 04                LDA #$04
8D 15 03             STA $0315
AD 15 03   WAIT:     LDA $0315
D0 FB                BNE WAIT
AD 17 03             LDA $0317
6A                   ROR A
68                   PHA
60                   RTS
*/
#define READ_BIT_PATCH_11_ADDR (0xE71C)
const uint8_t __in_flash() mia_read_bit_patch_11[] = {
    0x48, 0xA9, 0x04, 0x8D, 0x15, 0x03, 0xAD, 0x15, 
    0x03, 0xD0, 0xFB, 0xAD, 0x17, 0x03, 0x6A, 0x68, 
    0x60
};

void mia_task(void)
{
    static uint32_t prev_io_errors = 0;
    // check on watchdog unless we explicitly ended or errored
    if (mia_active() && action_result == -1)
    {
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(now, action_watchdog_timer) < 0)
        {
            printf("****TIMEOUT****\n");
            action_result = -3;
            main_stop();
        }
    }
    IOREGS(0x031B) = oric_bank1[IOREGS(0x31A)];
    switch(mia_state){
        case MIA_LOADING_DEVROM:
            if(!rom_active()){
                mia_state = MIA_LOADING_BIOS;
                if(mia_boot_settings & MIA_BOOTSET_B11){
                    if(!rom_load("BASIC11",7)){
                    //if(!rom_load("test108k",8)){
                        ssd_write_text(0,2,true,"!rom_load 11 failed");
                    }else{
                        ssd_write_text(0,1,false,"BIOS loaded ok");
                    }
                }else{
                    if(!rom_load("BASIC10",7)){
                    //if(!rom_load("test108k",8)){
                        ssd_write_text(0,2,true,"!rom_load 10 failed");
                    }else{
                        ssd_write_text(0,1,false,"BIOS loaded ok");
                    }
                }
            }
            break;
        case MIA_LOADING_BIOS:
            if(!rom_active()){
                ssd_write_text(0,1,false,"BIOS loaded done");
                mia_state = MIA_IDLE;
                //Patch for TAP loading CLOAD
                if(mia_boot_settings & MIA_BOOTSET_TAP){
                    if(mia_boot_settings & MIA_BOOTSET_B11){
                        for(uint16_t i=0; i<sizeof(mia_synch_patch_11); i++){
                            xram[SYNCH_PATCH_11_ADDR+i] = mia_synch_patch_11[i];
                        }
                        for(uint16_t i=0; i<sizeof(mia_read_bit_patch_11); i++){
                            xram[READ_BIT_PATCH_11_ADDR+i] = mia_read_bit_patch_11[i];
                        }
                        //Disable byte patch if bit mode is enabledß
                        if(!(mia_boot_settings & MIA_BOOTSET_TAP_BIT)){     
                            for(uint16_t i=0; i<sizeof(mia_cload_patch_11); i++){
                                xram[CLOAD_PATCH_11_ADDR+i] = mia_cload_patch_11[i];
                            }
                        }
                     }else{
                        printf("CLOAD patch for BASIC10 not implemented\n");
                    }
                }
                if(!!(mia_boot_settings & MIA_BOOTSET_FAST)){
                    ssd_write_text(0,1,false,"**Return Boot**");
                    api_return_boot();
                }else{
                    main_run();
                }
            }
            break;
        case MIA_IDLE:
            break;
    }
    if(mia_io_errors != prev_io_errors){
        if(mia_io_errors > 0){
            printf("!IO error %ld\n", mia_io_errors);
        }
        prev_io_errors = mia_io_errors;
    }
    if(reset_requested){
        ext_put(EXT_RESET,true);
        reset_requested = false;
    }
}

bool mia_print_error_message(void)
{
    switch (action_result)
    {
    case -1: // OK, default at start
    case -2: // OK, explicitly ended
        return false;
        break;
    case -3:
        printf("?watchdog timeout\n");
        break;
    default:
        printf("?verify failed at $%04lX\n", action_result);
        break;
    }
    return true;
}

void mia_read_buf(uint16_t addr)
{
    assert(!cpu_active());
    // avoid forbidden areas
    uint16_t len = mbuf_len;
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF)
            mbuf[len] = REGS(addr + len);
        else
            mbuf[len] = 0;
    while (len && (addr + len > 0xFF00))
        if (addr + --len <= 0xFFFF)
            mbuf[len] = 0;
    if (!len)
        return;
    rw_addr = addr;
    rw_end = len;
    rw_pos = 0;
    action_state = action_state_read;
    main_run();
}

void mia_verify_buf(uint16_t addr)
{
    assert(!cpu_active());
    // avoid forbidden areas
    action_result = -1;
    uint16_t len = mbuf_len;
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF && mbuf[len] != REGS(addr + len))
            action_result = addr + len;
    while (len && (addr + len > 0xFF00))
        --len;
    if (!len || action_result != -1)
        return;
    rw_addr = addr;
    rw_end = len;
    rw_pos = 0;
    action_state = action_state_verify;
    main_run();
}

void mia_write_buf(uint16_t addr)
{
    assert(!cpu_active());
    // avoid forbidden area
    uint16_t len = mbuf_len;
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF)
            REGS(addr + len) = mbuf[len];
    while (len && (addr + len > 0xFF00))
        len--;
    if (!len)
        return;
    rw_addr = addr;
    rw_end = len;
    rw_pos = 0;
    action_state = action_state_write;
    main_run();
}


void mia_enable_overlay_ram(bool low_enable, bool high_enable){
    MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & ~(1u << MIA_MAP_SM1)) | (bool_to_bit(low_enable) << MIA_MAP_SM1);
    MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & ~(1u << MIA_MAP_SM2)) | (bool_to_bit(high_enable) << MIA_MAP_SM2);
//    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM1, low_enable);
//    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM2, high_enable);
}

/*
void mia_enable_rom(bool device_enable, bool bios_enable){
    MIA_READ_PIO->ctrl = (MIA_READ_PIO->ctrl & ~(1u << MIA_ROM_READ_SM)) | (bool_to_bit(device_enable || bios_enable) << MIA_ROM_READ_SM);
//    MIA_READ_PIO->ctrl = (MIA_READ_PIO->ctrl & ~(1u << MIA_ROM_READ_SM1)) | (bool_to_bit(bios_enable) << MIA_ROM_READ_SM1);
//    MIA_READ_PIO->ctrl = (MIA_READ_PIO->ctrl & ~(1u << MIA_ROM_READ_SM2)) | (bool_to_bit(device_enable || bios_enable) << MIA_ROM_READ_SM2);
//    pio_sm_set_enabled(MIA_READ_PIO, MIA_ROM_READ_SM1, bios_enable);
//    pio_sm_set_enabled(MIA_READ_PIO, MIA_ROM_READ_SM2, device_enable || bios_enable);
}
*/

/* TODO Evaluate how risky runtime change of ROM address is */

void mia_set_rom_addr(uintptr_t addr){
    //pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, addr >> 14);
    MIA_READ_PIO->txf[MIA_READ_ADDR_SM] = addr >> 14;
    /*
    uint16_t pull_instr = pio_encode_pull(false, true);
    uint16_t mov_instr = pio_encode_mov(pio_y, pio_osr);
    uint16_t out_instr = pio_encode_out(pio_null, 32);
    while(!(MIA_READ_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MIA_READ_SM)))){
    //    //tight waiting loop till FIFO is empty
    }
    MIA_READ_PIO->ctrl = (MIA_READ_PIO->ctrl & ~(1u << MIA_READ_SM)) | (bool_to_bit(false) << MIA_READ_SM);
    pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_SM, addr >> 14);
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pull_instr);
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, mov_instr);
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, out_instr);
    MIA_READ_PIO->ctrl = (MIA_READ_PIO->ctrl & ~(1u << MIA_READ_SM)) | (bool_to_bit(true) << MIA_READ_SM);
    */
}

//bool mia_overlay_ram_enabled = false;
//bool mia_device_rom_enabled = false;
/*
void mia_set_rom_ram_enable(bool device_rom, bool overlay_ram){
    //if(device_rom != mia_device_rom_enabled)
    mia_set_rom_addr(device_rom ? (uintptr_t)oric_bank2 : (uintptr_t)oric_bank3);

    //bios rom enabled only when overlay ram is disabled
    //mia_enable_rom(device_rom, !overlay_ram); 
    mia_set_rom_read_enable(device_rom || !overlay_ram); 
    //mia_device_rom_enabled = device_rom;

    //high bank overlay ram only enabled when device_rom is disabled
    mia_enable_overlay_ram(overlay_ram, !device_rom && overlay_ram);
    //mia_overlay_ram_enabled = overlay_ram;
}
*/
//Called by action loop, needs to be fast so using bare PIO accesses
inline void mia_set_rom_ram_enable(bool device_rom, bool basic_rom){
    //mia_set_rom_read_enable(device_rom || basic_rom); 
    MIA_ROM_READ_PIO->ctrl = (MIA_ROM_READ_PIO->ctrl & ~(1u << MIA_ROM_READ_SM)) | (bool_to_bit(device_rom || basic_rom) << MIA_ROM_READ_SM);
    //device rom loaded in bank2, basic rom loaded in bank3
    //mia_set_rom_addr(basic_rom ? (uintptr_t)oric_bank3 : (uintptr_t)oric_bank2);
    if(device_rom || basic_rom)
        //MIA_READ_PIO->txf[MIA_READ_ADDR_SM] = (basic_rom ? (uintptr_t)oric_bank3 : (uintptr_t)oric_bank2) >> 14;
        MIA_READ_PIO->txf[MIA_READ_ADDR_SM] = (basic_rom ? (uintptr_t)(0x2000C000 >> 14) : (uintptr_t)(0x20008000 >> 14));
    
    //high bank overlay ram only enabled when device_rom is disabled
    //mia_enable_overlay_ram(overlay_ram, !device_rom && overlay_ram);
    bool overlay_ram = !basic_rom;
    MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & ~(1u << MIA_MAP_SM1)) | (bool_to_bit(overlay_ram) << MIA_MAP_SM1);
    MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & ~(1u << MIA_MAP_SM2)) | (bool_to_bit(!device_rom && overlay_ram) << MIA_MAP_SM2);
}

int mia_read_dma_channel;

//#define CASE_READ(addr) (addr & 0x1F)
//#define CASE_WRITE(addr) (0x20 | (addr & 0x1F))
#define CASE_READ(addr) (addr & 0x0000FFFF)
#define CASE_WRITE(addr) (0x01000000 | (addr & 0x0000FFFF))
#define MIA_RW0 IOREGS(0x03A4)
#define MIA_STEP0 *(int8_t *)&IOREGS(0x03A5)
#define MIA_ADDR0 IOREGSW(0x03A6)
#define MIA_RW1 IOREGS(0x03A8)
#define MIA_STEP1 *(int8_t *)&IOREGS(0x03A9)
#define MIA_ADDR1 IOREGSW(0x03AA)
static __attribute__((optimize("O1"))) __not_in_flash() void act_loop(void)
{
    printf("act_loop started\n");
    
    // In here we bypass the usual SDK calls as needed for performance.
    while (true)
    {
        if (!(MIA_ACT_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MIA_ACT_SM))))
        {
            uint32_t rw_data_addr = MIA_ACT_PIO->rxf[MIA_ACT_SM];
            (&dma_hw->ch[mia_read_dma_channel])->al3_read_addr_trig = (uintptr_t)((uint32_t)&iopage | (rw_data_addr & 0xFF));
        
            //Track errors and stop processing if address is wrong (0x03xx)
            if((rw_data_addr & 0x0000FF00) != 0x00000300){
                mia_io_errors++;
                continue;
            }
            if(!(rw_data_addr & 0x01000000)){  //Handle io page reads. Save PIO cycles
                if((mia_iopage_enable_map >> ((rw_data_addr & 0x000000FC) >> 2)) & 0x1ULL)
                    MIA_IO_READ_PIO->irq = 1u << 5;
            }else{  //Write - Saves having dedicated write PIO program
                IOREGS(rw_data_addr & 0xFF) = (rw_data_addr >> 16) & 0xFF;
            }
          
            if (true)//((1u << CPU_RESB_PIN) & sio_hw->gpio_in))
            {
                uint32_t data = (rw_data_addr >> 16) & 0xFF;
                switch (rw_data_addr & 0x0100FFFF)
                {
                case CASE_WRITE(0x300):
                    tap_act(data);              //Motor sense
                    break;
                case CASE_READ(TAP_IO_STAT):
                case CASE_WRITE(TAP_IO_CMD):
                case CASE_READ(TAP_IO_DATA):
                case CASE_WRITE(TAP_IO_DATA):
                    //printf(" %08x   \n", rw_data_addr);
                    //tap_act();
                    break;
                //RW_DATA segment at xram 0x4000 aka oric_bank1
                case CASE_WRITE(0x31A): //ADDR
                    //IOREGS(0x031B) = 0x08;
                    //break;     
                   __attribute__((fallthrough));
                case CASE_READ(0x031B): //RW_DATA
                    IOREGS(0x031B) = oric_bank1[IOREGS(0x31A)]; 
                    break;
                case CASE_WRITE(0x031B): //RW_DATA
                    oric_bank1[IOREGS(0x31A)] = data;
                    break;
                //ACIA Device Registers 0x380-0x383
                case CASE_READ(ACIA_IO_DATA):
                    acia_read();
                    break;
                case CASE_WRITE(ACIA_IO_DATA):
                    acia_write(data);
                    break;
                case CASE_READ(ACIA_IO_STAT):
                    acia_clr_irq();
                    break;
                case CASE_WRITE(ACIA_IO_STAT):
                    acia_reset(false);
                    break;
                case CASE_WRITE(ACIA_IO_CMD):
                    acia_cmd(data);
                    break;
                case CASE_WRITE(ACIA_IO_CTRL):
                    acia_ctrl(data);
                    break;
                //Microdisc Device Register
                case CASE_READ(DSK_IO_CMD):
                    dsk_reg_irq = 0x80;         //Clear IRQ on read (active low)
                    IOREGS(DSK_IO_CTRL) = dsk_reg_irq;
                    break;                    
                case CASE_WRITE(DSK_IO_CMD):    //CMD and STAT are overlayed R/W
                    dsk_reg_irq = 0x80;         //Clear IRQ on write (active low)
                    IOREGS(DSK_IO_CTRL) = dsk_reg_irq;
                    dsk_act(data);              //Process command
                    break;
                case CASE_READ(DSK_IO_DATA):
                    dsk_reg_status &= 0b11111101;
                    IOREGS(DSK_IO_CMD) = dsk_reg_status;
                    IOREGS(DSK_IO_DRQ) = 0x80;
                    dsk_rw(false,0x00);
                    break;
                case CASE_WRITE(DSK_IO_DATA):
                    dsk_reg_status &= 0b11111101;
                    IOREGS(DSK_IO_CMD) = dsk_reg_status;
                    IOREGS(DSK_IO_DRQ) = 0x80;
                    dsk_rw(true, data);
                    //ssd_got_action_word = true;
                    //ssd_action_word = rw_data_addr;
                    break;
                case CASE_WRITE(DSK_IO_CTRL):   //CTRL and IRQ are overlayed
                    //Bits 7:EPROM 6-5:drv_sel 4:side_sel 3:DDEN 2:Read CLK/2 1:ROM/RAM 0:IRQ_EN
                    //[7] 0:device rom enabled
                    //[1] 0:basic rom disabled
                    mia_set_rom_ram_enable(!(data & 0x80), !!(data & 0x02)); //device_rom,basic_rom
                    dsk_set_ctrl(data); //Handling of DSK related bits
                    IOREGS(DSK_IO_CTRL) = dsk_reg_irq; // 
                    //ssd_got_action_word = true;
                    //ssd_action_word = rw_data_addr;
                    break;
                case CASE_READ(DSK_IO_DRQ):
                    //ssd_got_action_word = true;
                    //ssd_action_word = rw_data_addr;
                    break;

                //RP6502-like interface
                case CASE_READ(0x03B7): // action write
                    if (rw_pos < rw_end)
                    {
                        if (rw_pos > 0)
                        {
                            IOREGS(0x03B2) = mbuf[rw_pos];
                            IOREGSW(0x03B4) += 1;
                        }
                        if (++rw_pos == rw_end)
                            IOREGS(0x03B7) = 0x00;
                    }
                    else if(action_state == action_state_write)
                    {
                        //TODO proper handling on Oric
                        //gpio_put(CPU_RESB_PIN, false);
                        reset_requested = true;
                        action_result = -2;
                        main_stop();
                    }
                    break;
                case CASE_WRITE(0x03BD): // action read
                    if (rw_pos < rw_end)
                    {
                        IOREGSW(0x03B2) += 1;
                        mbuf[rw_pos] = data;
                        if (++rw_pos == rw_end)
                        {
                            //TODO proper handling on Oric
                            //gpio_put(CPU_RESB_PIN, false);
                            reset_requested = true;
                            action_result = -2;
                            main_stop();
                        }
                    }
                    break;
                case CASE_WRITE(0x03BC): // action verify
                    if (rw_pos < rw_end)
                    {
                        IOREGSW(0x03B2) += 1;
                        if (mbuf[rw_pos] != data && action_result < 0)
                            action_result = IOREGSW(0x0382) - 1;
                        if (++rw_pos == rw_end)
                        {
                            //TODO proper handling on Oric
                            //gpio_put(CPU_RESB_PIN, false);
                            reset_requested = true;
                            action_result = -2;
                            main_stop();
                        }
                    }
                    break;
                case CASE_WRITE(0x03B0): // IRQ Enable
                    irq_enabled = data;
                    __attribute__((fallthrough));
                case CASE_READ(0x03B0): // IRQ ACK
                    //gpio_put(CPU_IRQB_PIN, true);
                    break;
                case CASE_WRITE(0x03AF): // OS function call
                    api_return_blocked();
                    if (data == 0x00) // zxstack()
                    {
                        API_STACK = 0;
                        xstack_ptr = XSTACK_SIZE;
                        api_return_ax(0);
                    }
                    else if (data == 0xFF) // exit()
                    {
                        //TODO proper handling on Oric
                        //gpio_put(CPU_RESB_PIN, false);
                        reset_requested = true;
                        main_stop();
                    }
                    break;
                case CASE_WRITE(0x03AC): // xstack
                    if (xstack_ptr)
                        xstack[--xstack_ptr] = data;
                    API_STACK = xstack[xstack_ptr];
                    break;
                case CASE_READ(0x03AC): // xstack
                    if (xstack_ptr < XSTACK_SIZE)
                        ++xstack_ptr;
                    API_STACK = xstack[xstack_ptr];
                    break;
                case CASE_WRITE(0x03AB): // Set XRAM >ADDR1
                    IOREGS(0x03AB) = data;
                    MIA_RW1 = xram[MIA_ADDR1];
                    break;
                case CASE_WRITE(0x03AA): // Set XRAM <ADDR1
                    IOREGS(0x03AA) = data;
                    MIA_RW1 = xram[MIA_ADDR1];
                    break;
                case CASE_WRITE(0x03A8): // W XRAM1
                    xram[MIA_ADDR1] = data;
                    //PIX_SEND_XRAM(RIA_ADDR1, data);
                    MIA_RW0 = xram[MIA_ADDR0];
                    __attribute__((fallthrough));
                case CASE_READ(0x03A8): // R XRAM1
                    MIA_ADDR1 += MIA_STEP1;
                    MIA_RW1 = xram[MIA_ADDR1];
                    break;
                case CASE_WRITE(0x03A7): // Set XRAM >ADDR0
                    IOREGS(0x03A7) = data;
                    MIA_RW0 = xram[MIA_ADDR0];
                    break;
                case CASE_WRITE(0x03A6): // Set XRAM <ADDR0
                    IOREGS(0x03A6) = data;
                    MIA_RW0 = xram[MIA_ADDR0];
                    break;
                case CASE_WRITE(0x03A4): // W XRAM0
                    xram[MIA_ADDR0] = data;
                    //PIX_SEND_XRAM(RIA_ADDR0, data);
                    MIA_RW1 = xram[MIA_ADDR1];
                    __attribute__((fallthrough));
                case CASE_READ(0x03A4): // R XRAM0
                    MIA_ADDR0 += MIA_STEP0;
                    MIA_RW0 = xram[MIA_ADDR0];
                    break;
                case CASE_READ(0x03A2): // UART Rx
                {
                    int ch = cpu_rx_char;
                    if (ch >= 0)
                    {
                        IOREGS(0x03A2) = ch;
                        IOREGS(0x03A0) |= 0b01000000;
                        cpu_rx_char = -1;
                    }
                    else
                    {
                        IOREGS(0x03AE0) &= ~0b01000000;
                        IOREGS(0x03AE2) = 0;
                    }
                    break;
                }
                case CASE_WRITE(0x03A1): // UART Tx
                    if (com_tx_writable())
                        com_tx_write(data);
                    if (com_tx_writable())
                        IOREGS(0x03A0) |= 0b10000000;
                    else
                        IOREGS(0x03A0) &= ~0b10000000;
                    break;
                case CASE_READ(0x03A0): // UART Tx/Rx flow control
                {
                    int ch = cpu_rx_char;
                    if (!(IOREGS(0x03A0) & 0b01000000) && ch >= 0)
                    {
                        IOREGS(0x03A2) = ch;
                        IOREGS(0x03A0) |= 0b01000000;
                        cpu_rx_char = -1;
                    }
                    if (com_tx_writable())
                        IOREGS(0x03A0) |= 0b10000000;
                    else
                        IOREGS(0x03A0) &= ~0b10000000;
                    break;
                }
    
                }
            /*
                if((rw_data_addr & 0x0000FFF0) == 0x000003B0){
                    //ssd_got_action_word = true;
                    if(rw_data_addr >> 24){
                        ssd_action_word = rw_data_addr;
                        ssd_action_is_wr = true;
                    }else{
                        //static uint8_t cnt = 0;
                        //ssd_action_rword = rw_data_addr | (cnt++ <<16);
                        ssd_action_rword = (rw_data_addr & 0xFF00FFFF) | (IOREGS(rw_data_addr & 0xFFFF) << 16);
                        ssd_action_is_wr = false;
                    }
                }
            */
            }
        }
            
    }
}

static void mia_write_pio_init(void)
{
    
    // PIO to manage 6502 writes
    uint offset = pio_add_program(MIA_WRITE_PIO, &mia_write_program);
    pio_sm_config config = mia_write_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A_PIN_BASE);
    sm_config_set_in_shift(&config, false, false, 0);
    //sm_config_set_out_pins(&config, D_PIN_BASE, 8);
    sm_config_set_jmp_pin(&config, RnW_PIN);
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, D_PIN_BASE, 8, false); //Start in read mode
    //sm_config_set_sideset_pins(&config, CPU_PHI2_PIN);
    //pio_gpio_init(RIA_WRITE_PIO, CPU_PHI2_PIN);
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, CPU_PHI2_PIN, 1, true);
    pio_sm_init(MIA_WRITE_PIO, MIA_WRITE_SM, offset, &config);
    //mia_set_rom_read_enable(false);
    pio_sm_put(MIA_WRITE_PIO, MIA_WRITE_SM, ((uintptr_t)xram >> 14));
    pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_mov(pio_x, pio_osr));
    pio_sm_set_enabled(MIA_WRITE_PIO, MIA_WRITE_SM, true);

    // Need both channels now to configure chain ping-pong
    int addr_chan = dma_claim_unused_channel(true);
    int data_chan = dma_claim_unused_channel(true);

    // DMA move the requested memory data to PIO for output
    dma_channel_config data_dma = dma_channel_get_default_config(data_chan);
    channel_config_set_high_priority(&data_dma, true);
    channel_config_set_dreq(&data_dma, pio_get_dreq(MIA_WRITE_PIO, MIA_WRITE_SM, false));
    channel_config_set_read_increment(&data_dma, false);
    channel_config_set_transfer_data_size(&data_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&data_dma, addr_chan);
    dma_channel_configure(
        data_chan,
        &data_dma,
        xram,                              // dst
        &MIA_WRITE_PIO->rxf[MIA_WRITE_SM], // src
        1,
        false);

    // DMA move address from PIO into the data DMA config
    dma_channel_config addr_dma = dma_channel_get_default_config(addr_chan);
    channel_config_set_high_priority(&addr_dma, true);
    channel_config_set_dreq(&addr_dma, pio_get_dreq(MIA_WRITE_PIO, MIA_WRITE_SM, false));
    channel_config_set_read_increment(&addr_dma, false);
    channel_config_set_chain_to(&addr_dma, data_chan);
    dma_channel_configure(
        addr_chan,
        &addr_dma,
        &dma_channel_hw_addr(data_chan)->write_addr, // dst
        &MIA_WRITE_PIO->rxf[MIA_WRITE_SM],           // src
        1,
        true);
    
}

/*
static void mia_read_pio_init(void)
{ 
    // PIO for 6502 reads
    uint offset = pio_add_program(MIA_READ_PIO, &mia_read_program);
    pio_sm_config config = mia_read_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A_PIN_BASE);
    sm_config_set_in_shift(&config, false, true, 14);
    sm_config_set_out_pins(&config, D_PIN_BASE, 8);
    sm_config_set_out_shift(&config, true, true, 8);
    sm_config_set_jmp_pin(&config, A15_PIN);
        //sm_config_set_out_special(&config, true, false, 0); //sticky output
    //for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
    //    pio_gpio_init(MIA_READ_PIO, i);
    //pio_gpio_init(MIA_READ_PIO, DIR_PIN);
    pio_sm_set_consecutive_pindirs(MIA_READ_PIO, MIA_READ_SM, D_PIN_BASE, 8, true); 
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, DIR_PIN, 1, true);
    pio_sm_init(MIA_READ_PIO, MIA_READ_SM, offset, &config);
    pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_SM, (uintptr_t)oric_bank3 >> 14);
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pio_encode_mov(pio_y, pio_osr));
    //WORKAROUND Odd extra OUT instruction needed, else the next pull does not work 
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pio_encode_out(pio_null, 32));
    //printf(" Y \n");
    pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_SM, (uintptr_t)oric_bank0 >> 14);
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_SM, pio_encode_mov(pio_x, pio_osr));
    //printf(" X \n");
    //if(!pio_sm_is_tx_fifo_empty(MIA_READ_PIO, MIA_READ_SM))
    //    printf("READ FIFO NOT EMPTY!\n");
    //else
    //    printf("TX fifo empty\n");
    pio_sm_set_enabled(MIA_READ_PIO, MIA_READ_SM, true);
    
    // Need both channels now to configure chain ping-pong
    int addr_chan = dma_claim_unused_channel(true);
    int data_chan = dma_claim_unused_channel(true);

    // DMA move the requested memory data to PIO for output
    dma_channel_config data_dma = dma_channel_get_default_config(data_chan);
    channel_config_set_high_priority(&data_dma, true);
    channel_config_set_dreq(&data_dma, pio_get_dreq(MIA_READ_PIO, MIA_READ_SM, true));
    channel_config_set_transfer_data_size(&data_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&data_dma, addr_chan);
    dma_channel_configure(
        data_chan,
        &data_dma,
        &MIA_READ_PIO->txf[MIA_READ_SM], // dst
        xram,                            // src
        1,
        false);

    // DMA move address from PIO into the data DMA config
    dma_channel_config addr_dma = dma_channel_get_default_config(addr_chan);
    channel_config_set_high_priority(&addr_dma, true);
    channel_config_set_dreq(&addr_dma, pio_get_dreq(MIA_READ_PIO, MIA_READ_SM, false));
    channel_config_set_read_increment(&addr_dma, false);
    channel_config_set_chain_to(&addr_dma, data_chan);
    dma_channel_configure(
        addr_chan,
        &addr_dma,
        &dma_channel_hw_addr(data_chan)->read_addr, // dst
        &MIA_READ_PIO->rxf[MIA_READ_SM],            // src
        1,
        true);
}
*/
static uint mia_read_addr_prg_offset;
uint mia_get_read_addr_prg_offset(void){
    return mia_read_addr_prg_offset;
}

static void mia_read_pio_init(void)
{ 
    // PIO for 6502 reads
    uint offset_d = pio_add_program(MIA_READ_PIO, &mia_read_data_program);
    uint offset_a = pio_add_program(MIA_READ_PIO, &mia_read_addr_program);
    mia_read_addr_prg_offset = offset_a;
    pio_sm_config config_d = mia_read_data_program_get_default_config(offset_d);
    pio_sm_config config_a = mia_read_addr_program_get_default_config(offset_a);
    sm_config_set_out_pins(&config_d, D_PIN_BASE, 8);
    sm_config_set_out_shift(&config_d, true, true, 8);
    
    sm_config_set_in_pins(&config_a, A_PIN_BASE);
    sm_config_set_in_shift(&config_a, false, true, 14);
    sm_config_set_jmp_pin(&config_a, A15_PIN);
    sm_config_set_set_pins(&config_a, DIR_PIN, 1);
    
        //sm_config_set_out_special(&config, true, false, 0); //sticky output
    //for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
    //    pio_gpio_init(MIA_READ_PIO, i);
    //pio_gpio_init(MIA_READ_PIO, DIR_PIN);
    pio_sm_set_consecutive_pindirs(MIA_READ_PIO, MIA_READ_DATA_SM, D_PIN_BASE, 8, true); 
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, DIR_PIN, 1, true);
    pio_sm_init(MIA_READ_PIO, MIA_READ_DATA_SM, offset_d, &config_d);
    pio_sm_init(MIA_READ_PIO, MIA_READ_ADDR_SM, offset_a, &config_a);
    pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, (uintptr_t)oric_bank0 >> 14);
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, pio_encode_mov(pio_y, pio_osr));
    //WORKAROUND Odd extra OUT instruction needed, else the next pull does not work 
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, pio_encode_out(pio_null, 32));
    //printf(" Y \n");
    //pio_sm_put_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, (uintptr_t)oric_bank3 >> 14);
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_READ_ADDR_SM, pio_encode_mov(pio_x, pio_osr));
    //printf(" X \n");
    //if(!pio_sm_is_tx_fifo_empty(MIA_READ_PIO, MIA_READ_SM))
    //    printf("READ FIFO NOT EMPTY!\n");
    //else
    //    printf("TX fifo empty\n");
    pio_sm_set_enabled(MIA_READ_PIO, MIA_READ_DATA_SM, true);
    pio_sm_set_enabled(MIA_READ_PIO, MIA_READ_ADDR_SM, true);
    
    // Need both channels now to configure chain ping-pong
    int addr_chan = dma_claim_unused_channel(true);
    int data_chan = dma_claim_unused_channel(true);
    mia_read_dma_channel = data_chan;

    // DMA move the requested memory data to PIO for output
    dma_channel_config data_dma = dma_channel_get_default_config(data_chan);
    channel_config_set_high_priority(&data_dma, true);
    channel_config_set_dreq(&data_dma, pio_get_dreq(MIA_READ_PIO, MIA_READ_DATA_SM, true));
    channel_config_set_transfer_data_size(&data_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&data_dma, addr_chan);
    dma_channel_configure(
        data_chan,
        &data_dma,
        &MIA_READ_PIO->txf[MIA_READ_DATA_SM], // dst
        xram,                            // src
        1,
        false);

    // DMA move address from PIO into the data DMA config
    dma_channel_config addr_dma = dma_channel_get_default_config(addr_chan);
    channel_config_set_high_priority(&addr_dma, true);
    channel_config_set_dreq(&addr_dma, pio_get_dreq(MIA_READ_PIO, MIA_READ_ADDR_SM, false));
    channel_config_set_read_increment(&addr_dma, false);
    channel_config_set_chain_to(&addr_dma, data_chan);
    dma_channel_configure(
        addr_chan,
        &addr_dma,
        &dma_channel_hw_addr(data_chan)->read_addr, // dst
        &MIA_READ_PIO->rxf[MIA_READ_ADDR_SM],            // src
        1,
        true);
}

static uint mia_act_prg_offset;
uint mia_get_act_prg_offset(void){
    return mia_act_prg_offset;
}
static void mia_act_pio_init(void)
{
    // PIO to supply action loop with events
    uint offset = pio_add_program(MIA_ACT_PIO, &mia_action_program);
    mia_act_prg_offset = offset;
    pio_sm_config config = mia_action_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A_PIN_BASE);
    sm_config_set_in_shift(&config, false, true, 25);
    sm_config_set_jmp_pin(&config, RnW_PIN);
    pio_sm_init(MIA_ACT_PIO, MIA_ACT_SM, offset, &config);
    pio_sm_put(MIA_ACT_PIO, MIA_ACT_SM, 0x0300 >> 8);
    pio_sm_exec_wait_blocking(MIA_ACT_PIO, MIA_ACT_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_ACT_PIO, MIA_ACT_SM, pio_encode_mov(pio_y, pio_osr));
    pio_sm_set_enabled(MIA_ACT_PIO, MIA_ACT_SM, true);
    multicore_launch_core1(act_loop);
}

static uint mia_io_read_prg_offset;
uint mia_get_io_read_prg_offset(void){
    return mia_io_read_prg_offset;
}
static void mia_io_read_pio_init(void)
{
    
    // PIO to manage 6502 reads to Oric IO page 0x0300
    uint offset = pio_add_program(MIA_IO_READ_PIO, &mia_io_read_program);
    mia_io_read_prg_offset = offset;
    pio_sm_config config = mia_io_read_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A2_PIN);
    sm_config_set_in_shift(&config, false, true, 6);
    sm_config_set_out_pins(&config, D_PIN_BASE, 8);
    sm_config_set_out_shift(&config, true, true, 9);
    sm_config_set_set_pins(&config, DIR_PIN, 1);
    //sm_config_set_out_special(&config, true, false, 0); //sticky output
    sm_config_set_jmp_pin(&config, RnW_PIN);
    //pio_gpio_init(RIA_WRITE_PIO, CPU_PHI2_PIN);
    //for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
    //    pio_gpio_init(MIA_IO_READ_PIO, i);
    //pio_gpio_init(MIA_READ_PIO, DIR_PIN);
    pio_sm_set_consecutive_pindirs(MIA_IO_READ_PIO, MIA_IO_READ_SM, DIR_PIN, 1, true);
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, CPU_PHI2_PIN, 1, true);
    pio_sm_init(MIA_IO_READ_PIO, MIA_IO_READ_SM, offset, &config);
    pio_sm_put(MIA_IO_READ_PIO, MIA_IO_READ_SM, (uintptr_t)mia_iopage_enable_map >> 6);
    pio_sm_exec_wait_blocking(MIA_IO_READ_PIO, MIA_IO_READ_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_IO_READ_PIO, MIA_IO_READ_SM, pio_encode_mov(pio_x, pio_osr));
    pio_sm_set_enabled(MIA_IO_READ_PIO, MIA_IO_READ_SM, true);

    // Need both channels now to configure chain ping-pong
    int addr_chan = dma_claim_unused_channel(true);
    int data_chan = dma_claim_unused_channel(true);

    dma_channel_config data_dma = dma_channel_get_default_config(data_chan);
    channel_config_set_high_priority(&data_dma, true);
    channel_config_set_dreq(&data_dma, pio_get_dreq(MIA_IO_READ_PIO, MIA_IO_READ_SM, true));
    channel_config_set_transfer_data_size(&data_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&data_dma, addr_chan);
    dma_channel_configure(
        data_chan,
        &data_dma,
        &MIA_IO_READ_PIO->txf[MIA_IO_READ_SM], // dst
        xram,                            // src
        1,
        false);

    // DMA move address from PIO into the data DMA config
    dma_channel_config addr_dma = dma_channel_get_default_config(addr_chan);
    channel_config_set_high_priority(&addr_dma, true);
    channel_config_set_dreq(&addr_dma, pio_get_dreq(MIA_IO_READ_PIO, MIA_IO_READ_SM, false));
    channel_config_set_read_increment(&addr_dma, false);
    channel_config_set_chain_to(&addr_dma, data_chan);
    dma_channel_configure(
        addr_chan,
        &addr_dma,
        &dma_channel_hw_addr(data_chan)->read_addr, // dst
        &MIA_IO_READ_PIO->rxf[MIA_IO_READ_SM],            // src
        1,
        true);
}

static void mia_rom_read_pio_init(void)
{
    
    // PIO to manage 6502 reads to Oric ROM area 0xC000-0xFFFF
    uint offset = pio_add_program(MIA_ROM_READ_PIO, &mia_rom_read_program);
    pio_sm_config config = mia_rom_read_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A8_PIN);
    sm_config_set_in_shift(&config, false, false, 0);
    sm_config_set_out_pins(&config, D_PIN_BASE, 8);
    sm_config_set_set_pins(&config, DIR_PIN, 1);
    //sm_config_set_sideset_pins(&config, DIR_PIN);
    //sm_config_set_out_special(&config, true, false, 0); //sticky output
    sm_config_set_jmp_pin(&config, RnW_PIN);
   //sm_config_set_sideset_pins(&config, CPU_PHI2_PIN);
    //pio_gpio_init(RIA_WR_PIO, CPU_PHI2_PIN);
    //for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
    //    pio_gpio_init(MIA_ROM_READ_PIO, i);
    pio_sm_init(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, offset, &config);
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, ((0xC000 >> 14) << 4) | 0b0110 );    //ROM 16kB at 0xC000 + MAP,DIR,Phi2,RW match
    pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0x03 );    //Constant for multiple matching
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_init(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, offset, &config);
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, 0xE000 >> 13);    //ROM 8kB at 0xE000
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM2, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM2, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_set_enabled(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, true);
    mia_set_rom_read_enable(false);

}

static uint mia_map_prg_offset;
uint mia_get_map_prg_offset(void){
    return mia_map_prg_offset;
}

static void mia_map_pio_init(void)
{
    
    // PIO to manage MAP signal timing
    uint offset = pio_add_program(MIA_MAP_PIO, &mia_map_program);
    mia_map_prg_offset = offset;
    pio_sm_config config = mia_map_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A13_PIN);
    sm_config_set_in_shift(&config, false, false, 0);
    sm_config_set_set_pins(&config, MAP_PIN, 1);
    //pio_gpio_init(MIA_MAP_PIO, MAP_PIN);
    pio_sm_init(MIA_MAP_PIO, MIA_MAP_SM1, offset, &config);
    pio_sm_set_consecutive_pindirs(MIA_MAP_PIO, MIA_MAP_SM1, MAP_PIN, 1, true);
    pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM1, 0xC000 >> 13);    //Overlay RAM 8kB at 0xC000
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM1, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM1, pio_encode_mov(pio_x, pio_osr));
    pio_sm_init(MIA_MAP_PIO, MIA_MAP_SM2, offset, &config);
    pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM2, 0xE000 >> 13);    //Overlay RAM 8kB at 0xE000
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM2, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM2, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0x0310 >> 4);
    //pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_mov(pio_y, pio_osr));
    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM1, false);
    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM2, false);   
}


void mia_init(void)
{
    
    // drive irq pin
    //gpio_init(nIRQ_PIN);
    gpio_init(RnW_PIN);
    ext_put(EXT_IRQ, false);
    //ext_put(EXT_nRESET,false);
    //ext_set_dir(EXT_IRQ, true);
    //gpio_set_pulls(nIRQ_PIN,false,false);
    //Don´t Enable levelshifters yet
    //ext_put(EXT_OE,false);
    //gpio_init(DIR_PIN);

    // drive nROMDIS
    ext_put(EXT_ROMDIS,false);
    //ext_set_dir(EXT_ROMDIS,false);
    //ext_put(EXT_nROMDIS,false);

    // safety check for compiler alignment
    assert(!((uintptr_t)xram & 0xFFFF));

    // Adjustments for GPIO performance. Important!
    for (int i = A_PIN_BASE; i <= MAP_PIN; i++)
    {
        //gpio_set_pulls(i, true, true);
        gpio_set_pulls(i, false, false);
        //gpio_set_input_hysteresis_enabled(i, false);
        //hw_set_bits(&pio0->input_sync_bypass, 1u << i);
        //hw_set_bits(&pio1->input_sync_bypass, 1u << i);
    }
    for (int i = D_PIN_BASE; i <= D_PIN_BASE+8; i++)
    {
        //gpio_set_pulls(i, true, true);
        gpio_set_pulls(i, false, false);
        //gpio_set_input_hysteresis_enabled(i, false);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_2MA);
    }
    //gpio_set_drive_strength(nIRQ_PIN, GPIO_DRIVE_STRENGTH_2MA);
    for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
        pio_gpio_init(MIA_READ_PIO, i);

    pio_gpio_init(MIA_READ_PIO, DIR_PIN);
    pio_gpio_init(MIA_MAP_PIO, MAP_PIN);
    
    //Invert RnW signal to nRW to save some PIO cycles
    gpio_set_inover(RnW_PIN, GPIO_OVERRIDE_INVERT);
    
    // Lower CPU0 on crossbar by raising others
    bus_ctrl_hw->priority |=
        BUSCTRL_BUS_PRIORITY_DMA_R_BITS |
        BUSCTRL_BUS_PRIORITY_DMA_W_BITS |
        BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    //Register space clears
    for(int i=0x0310; i<=0x031B; i++){
        IOREGS(i) = 0x00;
    }
    //Register space clears
    for(int i=0x03A0; i<=0x03BF; i++){
        IOREGS(i) = 0x00;
    }

    /*
    for(int i=0; i<64; i++){
        mia_iopage_enable_map[i] = 0x00;
    }
    //Enable response on IO registers 0x310-0x31B
    for(int i=(0x10 >> 2); i<=(0x1B >> 2); i++){
        mia_iopage_enable_map[i] = 0xff;
    }
    */
   mia_iopage_enable_map = 0x00;
    //Enable response on IO registers 0x310-0x31B (DSK/TAP)
    for(int i=(0x10 >> 2); i<=(0x1B >> 2); i++){
        mia_iopage_enable_map |= (0x1ULL << i);
    }
    //Enable response on IO registers 0x380-0x383 (ACIA)
    for(int i=(0x80 >> 2); i<=(0x83 >> 2); i++){
        mia_iopage_enable_map |= (0x1ULL << i);
    }
    //Enable response on IO registers 0x3A0-0x3BF (LOCI)
    for(int i=(0xA0 >> 2); i<=(0xBF >> 2); i++){
        mia_iopage_enable_map |= (0x1ULL << i);
    }
   
    mia_boot_settings = 0;

    // the inits
    // Same PIO order matters for timing tuning
    mia_map_pio_init(); //Must be first for MAP tuning
    mia_act_pio_init();
    //mia_write_pio_init();
    
    //Same PIO order matters timing tuning
    mia_read_pio_init();    //read_data, read_addr
    mia_io_read_pio_init();
    mia_rom_read_pio_init();
}

void mia_reclock(uint16_t clkdiv_int, uint8_t clkdiv_frac)
{
    pio_sm_set_clkdiv_int_frac(MIA_WRITE_PIO, MIA_WRITE_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_READ_PIO, MIA_READ_DATA_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_READ_PIO, MIA_READ_ADDR_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_IO_READ_PIO, MIA_IO_READ_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, clkdiv_int, clkdiv_frac);
    //pio_sm_set_clkdiv_int_frac(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_ACT_PIO, MIA_ACT_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_MAP_PIO, MIA_MAP_SM1, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_MAP_PIO, MIA_MAP_SM2, clkdiv_int, clkdiv_frac);
}

void mia_api_boot(void){
    mia_boot_settings = API_A;
    if(!!(mia_boot_settings & MIA_BOOTSET_FDC)){
        if(!rom_load("MICRODISC",9)){
            ssd_write_text(0,2,true,"!rom_load failed");
            api_return_errno(API_EMFILE);
        }else{
            ssd_write_text(0,2,false,"DEV ROM loaded ok");
            if(mia_boot_settings & MIA_BOOTSET_FAST){
                mia_set_rom_ram_enable(true,false);
                //Returns control with api_return_boot() when ROM has loaded
                ssd_write_text(0,2,false,"Fast boot ON");
            }else{
                api_return_ax(0);
                main_stop();
            }
            mia_state = MIA_LOADING_DEVROM;
        }
    }else{
        if(!rom_load("BASIC11",7)){
            ssd_write_text(0,2,true,"!rom_load failed");
            api_return_errno(API_EMFILE);
        }else{
            ssd_write_text(0,2,false,"BASIC11 ROM loaded ok");
            if(!!(mia_boot_settings & MIA_BOOTSET_FAST)){
                mia_set_rom_ram_enable(false,true);
                //Returns control with api_return_boot() when ROM has loaded
                ssd_write_text(0,2,false,"Fast boot ON");
            }else{
                api_return_ax(0);
                main_stop();
            }
            mia_state = MIA_LOADING_BIOS;
        }

    }
}