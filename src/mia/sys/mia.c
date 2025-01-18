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
#include "oric/rom.h"
#include "oric/tap.h"

#define MIA_WATCHDOG_MS 250

//volatile uint8_t mia_iopage_enable_map[64] __attribute__((aligned(64)));
static uint32_t mia_iopage_enable_map[2];

static enum state {
    action_state_idle = 0,
    action_state_read,
    action_state_write,
    action_state_verify,
} volatile action_state = action_state_idle;
static absolute_time_t action_watchdog_timer;
static volatile int32_t action_result = -1;
static int32_t saved_reset_vec = -1;
static int32_t saved_brk_vec = -1;
static uint16_t rw_addr;
static volatile int32_t rw_pos;
static volatile int32_t rw_end;
static volatile bool irq_enabled;
static volatile bool reset_requested;
static volatile bool snoop_flag;
volatile bool map_flag_basic, map_flag_device;
static bool saved_map_flag_basic, saved_map_flag_device;
static uint32_t ula_trig_addr;

void mia_clear_snoop_flag(void){
    snoop_flag = false;
}
bool mia_get_snoop_flag(void){
    return snoop_flag;
}

void mia_save_map_flags(void){
    saved_map_flag_basic = map_flag_basic;
    saved_map_flag_device = map_flag_device;
}



static uint8_t mia_boot_settings;

uint8_t mia_get_boot_settings(void){
    return mia_boot_settings;
}

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

//Using injection of PIO operations to enable and disable funtionality
//PIO_OP_OFF = set x, 31
#define PIO_OP_OFF 0xe03f
//PIO_OP_ON_3 = set x, 3
#define PIO_OP_ON_3 0xe023
//PIO_OP_ON_C = set x, C000>>13
#define PIO_OP_ON_C 0xe026
//PIO_OP_ON_E = set x, E000>>13
#define PIO_OP_ON_E 0xe027
void mia_set_rom_read_enable(bool enable)
{
    if(enable)
        MIA_ROM_READ_PIO->sm[MIA_ROM_READ_SM].instr = PIO_OP_ON_3;
    else
        MIA_ROM_READ_PIO->sm[MIA_ROM_READ_SM].instr = PIO_OP_OFF;
}

static enum {
    MIA_IDLE,
    MIA_LOADING_DEVROM,
    MIA_LOADING_BIOS,
} mia_boot_state;


void mia_run(void)
{

    ext_put(EXT_ROMDIS,true);
    ext_put(EXT_nOE,false);
    //printf("**mia_run()**");
    //ext_set_dir(EXT_ROMDIS,true);
    //mia_set_rom_read_enable(true);
    //mia_set_rom_ram_enable(true,false);
    mia_set_rom_read_enable(true);
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
    mia_set_rom_ram_enable(false,false);    //Turn on overlay RAM during read/write actions
    MIA_MAP_PIO->instr_mem[mia_get_map_prg_offset() + 3] = (uint16_t)(pio_encode_out(pio_y,14));

    action_result = -1;
    saved_reset_vec = XRAMW(0xFFFC);
    saved_brk_vec   = XRAMW(0xFFFE);
    XRAMW(0xFFFC) = 0x03B0;
    XRAMW(0xFFFE) = 0x03B0;
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
        // 03B0  78        SEI
        // 03B1  A9 00     LDA #$00
        // 03B3  8D 00 00  STA $0000
        // 03B6  B8        CLV
        // 03B7  50 F8     BVC $03B0
        //mia_set_watch_address(0xFFF6);
        IOREGS(0x03B0) = 0x78;
        IOREGS(0x03B1) = 0xA9;
        IOREGS(0x03B2) = mbuf[0];
        IOREGS(0x03B3) = 0x8D;
        IOREGS(0x03B4) = rw_addr & 0xFF;
        IOREGS(0x03B5) = rw_addr >> 8;
        IOREGS(0x03B6) = 0xB8;
        IOREGS(0x03B7) = 0x50;
        IOREGS(0x03B8) = 0xF8;
        break;
    case action_state_read:
    case action_state_verify:
        // Self-modifying fast load
        // FFF0  AD 00 00  LDA $0000
        // FFF3  8D FC FF  STA $FFFC/$FFFD
        // FFF6  80 F8     BRA $FFF0
        //Oric mod: CLV + BVC to get BRA 
        // 03B0  78        SEI
        // 03B1  AD 00 00  LDA $0000
        // 03B4  8D BC 03  STA $03BC/$03BD
        // 03B7  B8        CLV
        // 03B8  50 F7     BVC $03B0
        IOREGS(0x03B0) = 0x78;
        IOREGS(0x03B1) = 0xAD;
        IOREGS(0x03B2) = rw_addr & 0xFF;
        IOREGS(0x03B3) = rw_addr >> 8;
        IOREGS(0x03B4) = 0x8D;
        IOREGS(0x03B5) = (action_state == action_state_verify) ? 0xBC : 0xBD;
        IOREGS(0x03B6) = 0x03;
        IOREGS(0x03B7) = 0xB8;
        IOREGS(0x03B8) = 0x50;
        IOREGS(0x03B9) = 0xF7;
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
    mia_set_rom_read_enable(false);
    mia_set_rom_ram_enable(false,false);
    MIA_MAP_PIO->instr_mem[mia_get_map_prg_offset() + 3] = (uint16_t)(pio_encode_mov_not(pio_y,pio_null));
    //mia_set_rom_read_enable(false);
    mia_boot_settings = 0x00;
    action_state = action_state_idle;
    mia_boot_state = MIA_IDLE;
    if (saved_reset_vec >= 0)
    {
        XRAMW(0xFFFC) = saved_reset_vec;
        saved_reset_vec = -1;
    }
    if (saved_brk_vec >= 0)
    {
        XRAMW(0xFFFE) = saved_brk_vec;
        saved_brk_vec = -1;
    }
}

bool mia_active(void)
{
    return action_state != action_state_idle;
}

bool mia_boot_active(void)
{
    return mia_boot_state != MIA_IDLE;
}

/* Basic CLOAD read byte patch
A9 01                LDA #$01
8D 15 03             STA $0315
AD 15 03   WAIT:     LDA $0315
D0 FB                BNE WAIT
AD 17 03             LDA $0317
85 2F                STA $2F
18                   CLC
60                   RTS
*/
#define CLOAD_PATCH_10_ADDR (0xE630)
#define CLOAD_PATCH_11_ADDR (0xE6C9)
const uint8_t __in_flash() mia_cload_patch[] = {
    0xA9, 0x01, 0x8D, 0x15, 0x03, 0xAD, 0x15, 0x03,
    0xD0, 0xFB, 0xAD, 0x17, 0x03, 0x85, 0x2F, 0x18,
    0x60
};

/* Basic 1.1 CLOAD synch patch
4C 4D E7             JMP $E74D
*/
#define SYNCH_PATCH_11_ADDR (0xE735)
const uint8_t __in_flash() mia_synch_patch_11[] = {
    0x4C, 0x4D, 0xE7 
};
/* Basic 1.0 CLOAD synch patch
4C AD E6             JMP $E6AD
*/
#define SYNCH_PATCH_10_ADDR (0xE696)
const uint8_t __in_flash() mia_synch_patch_10[] = {
    0x4C, 0xAD, 0xE6 
};

/* Basic CLOAD read bit patch
48                   PHA
A9 04                LDA #$04
8D 15 03             STA $0315
AD 15 03   WAIT:     LDA $0315
D0 FB                BNE WAIT
AD 17 03             LDA $0317
6A                   ROR A
68                   PLA
60                   RTS
*/
#define READ_BIT_PATCH_10_ADDR (0xE67D)
#define READ_BIT_PATCH_11_ADDR (0xE71C)
const uint8_t __in_flash() mia_read_bit_patch[] = {
    0x48, 0xA9, 0x04, 0x8D, 0x15, 0x03, 0xAD, 0x15, 
    0x03, 0xD0, 0xFB, 0xAD, 0x17, 0x03, 0x6A, 0x68, 
    0x60
};

/* Basic CSAVE write byte patch
48                   PHA
8D 17 03             STA $0317
A9 02                LDA #$02
8D 15 03             STA $0315
AD 15 03   WAIT:     LDA $0315
D0 FB                BNE WAIT
68                   PLA
60                   RTS
*/
#define WRITE_BYTE_PATCH_10_ADDR (0xE5C6)
#define WRITE_BYTE_PATCH_11_ADDR (0xE65E)
const uint8_t __in_flash() mia_write_byte_patch[] = {
    0x48, 0x8D, 0x17, 0x03, 0xA9, 0x02, 0x8D, 0x15, 
    0x03, 0xAD, 0x15, 0x03, 0xD0, 0xFB, 0x68, 0x60
};

/* Basic autoload setup patch
A9 22               LDA #$22    ;Put '"\0' in buffer 
85 35               STA $35
A9 00               LDA #$00
85 36               STA $36
85 EA               STA $EA
A9 34               LDA #$34    ;Point to buffer
85 E9               STA $E9
A9 B6               LDA #$B6    ;Setup CLOAD token
38                  SEC
4C 0F C9            JMP $C90F   ;Jump to execute    Basic 1.1
4C F8 C8            JMP $C8F8   ;Jump to execute    Basic 1.0
*/
//Uses freed space in sync function
#define AUTOLOAD_SETUP_PATCH_11_ADDR (0xE738)
const uint8_t __in_flash() mia_autoload_setup_patch_11[] = {
    0xA9, 0x22, 0x85, 0x35, 0xA9, 0x00, 0x85, 0x36, 
    0x85, 0xEA, 0xA9, 0x34, 0x85, 0xE9, 0xA9, 0xB6, 
    0x38, 0x4C, 0x0F, 0xC9
};
#define AUTOLOAD_SETUP_PATCH_10_ADDR (0xE699)
const uint8_t __in_flash() mia_autoload_setup_patch_10[] = {
    0xA9, 0x22, 0x85, 0x35, 0xA9, 0x00, 0x85, 0x36, 
    0x85, 0xEA, 0xA9, 0x34, 0x85, 0xE9, 0xA9, 0xB6, 
    0x38, 0x4C, 0xF8, 0xC8
};
/* Basic autoload jump patch
4C 38 E7            JMP $E738   ;Basic 1.1
4C 3E EB            JMP $E699   ;Basic 1.0
*/
#define AUTOLOAD_JUMP_PATCH_11_ADDR (0xED83)
const uint8_t __in_flash() mia_autoload_jump_patch_11[] = {
    0x4C, 0x38, 0xE7
};
#define AUTOLOAD_JUMP_PATCH_10_ADDR (0xEB3E)
const uint8_t __in_flash() mia_autoload_jump_patch_10[] = {
    0x4C, 0x99, 0xE6
};

void mia_task(void)
{
    bool rom_is_loading;
    switch(mia_boot_state){
        case MIA_LOADING_DEVROM:
            if(!rom_active()){
                mia_boot_state = MIA_LOADING_BIOS;
                rom_is_loading = rom_load_mounted();
                if(!rom_is_loading){
                    if(mia_boot_settings & MIA_BOOTSET_B11){
                        rom_is_loading = rom_load("BASIC11",7);
                        if(!rom_is_loading)
                            rom_is_loading = rom_load_raw("basic11.rom",0xC000);
                        if(!rom_is_loading)
                            rom_is_loading = rom_load_raw("basic11b.rom",0xC000);

                        //if(!rom_load("test108k",8)){
                        if(!rom_is_loading){
                            printf("!rom_load 11 failed\n");
                        }else{
                            printf("BIOS loaded ok\n");
                        }
                    }else{
                        rom_is_loading = rom_load("BASIC10",7);
                        if(!rom_is_loading)
                            rom_is_loading = rom_load_raw("basic10.rom",0xC000);
                        if(!rom_is_loading){
                            printf("!rom_load 10 failed\n");
                        }else{
                            printf("BIOS loaded ok\n");
                        }
                    }
                }
            }
            break;
        case MIA_LOADING_BIOS:
            if(!rom_active()){
                printf("BIOS loaded done\n");
                mia_boot_state = MIA_IDLE;
                //Patch for TAP loading CLOAD and CSAVE
                if(!rom_is_mounted() && (mia_boot_settings & MIA_BOOTSET_TAP)){
                    if(mia_boot_settings & MIA_BOOTSET_B11){
                        for(uint16_t i=0; i<sizeof(mia_synch_patch_11); i++){
                            xram[SYNCH_PATCH_11_ADDR+i] = mia_synch_patch_11[i];
                        }
                        for(uint16_t i=0; i<sizeof(mia_read_bit_patch); i++){
                            xram[READ_BIT_PATCH_11_ADDR+i] = mia_read_bit_patch[i];
                        }
                        //Disable byte patch if bit mode is enabled
                        if(!(mia_boot_settings & MIA_BOOTSET_TAP_BIT)){     
                            for(uint16_t i=0; i<sizeof(mia_cload_patch); i++){
                                xram[CLOAD_PATCH_11_ADDR+i] = mia_cload_patch[i];
                            }
                        }
                        for(uint16_t i=0; i<sizeof(mia_write_byte_patch); i++){
                            xram[WRITE_BYTE_PATCH_11_ADDR+i] = mia_write_byte_patch[i];
                        }
                        if(!!(mia_boot_settings & MIA_BOOTSET_TAP_ALD)){
                            for(uint16_t i=0; i<sizeof(mia_autoload_setup_patch_11); i++){
                                xram[AUTOLOAD_SETUP_PATCH_11_ADDR+i] = mia_autoload_setup_patch_11[i];
                            }
                            for(uint16_t i=0; i<sizeof(mia_autoload_jump_patch_11); i++){
                                xram[AUTOLOAD_JUMP_PATCH_11_ADDR+i] = mia_autoload_jump_patch_11[i];
                            }
                        }
                     }else{
                        for(uint16_t i=0; i<sizeof(mia_synch_patch_10); i++){
                            xram[SYNCH_PATCH_10_ADDR+i] = mia_synch_patch_10[i];
                        }
                        for(uint16_t i=0; i<sizeof(mia_read_bit_patch); i++){
                            xram[READ_BIT_PATCH_10_ADDR+i] = mia_read_bit_patch[i];
                        }
                        //Disable byte patch if bit mode is enabled
                        if(!(mia_boot_settings & MIA_BOOTSET_TAP_BIT)){     
                            for(uint16_t i=0; i<sizeof(mia_cload_patch); i++){
                                xram[CLOAD_PATCH_10_ADDR+i] = mia_cload_patch[i];
                            }
                        }
                        for(uint16_t i=0; i<sizeof(mia_write_byte_patch); i++){
                            xram[WRITE_BYTE_PATCH_10_ADDR+i] = mia_write_byte_patch[i];
                        }
                        if(!!(mia_boot_settings & MIA_BOOTSET_TAP_ALD)){
                            for(uint16_t i=0; i<sizeof(mia_autoload_setup_patch_10); i++){
                                xram[AUTOLOAD_SETUP_PATCH_10_ADDR+i] = mia_autoload_setup_patch_10[i];
                            }
                            for(uint16_t i=0; i<sizeof(mia_autoload_jump_patch_10); i++){
                                xram[AUTOLOAD_JUMP_PATCH_10_ADDR+i] = mia_autoload_jump_patch_10[i];
                            }
                        }
                    }
                }
                if(!!(mia_boot_settings & MIA_BOOTSET_FAST)){
                    if(!!(mia_boot_settings & MIA_BOOTSET_RESUME)){
                        mia_set_rom_ram_enable(saved_map_flag_device,saved_map_flag_basic);
                        printf("Resuming\n");
                        __dsb();
                        api_return_resume();
                    }else{
                        if(mia_boot_settings & MIA_BOOTSET_FDC)
                            mia_set_rom_ram_enable(true,false);
                        else
                            mia_set_rom_ram_enable(false,true);
                        printf("Booting\n");
                        __dsb();
                        api_return_boot();
                    }
                }else{
                    main_run();
                }
            }
            break;
        case MIA_IDLE:
            break;
    }
}

void mia_main_task(){
    static uint32_t prev_io_errors = 0;

    if(reset_requested){
        ext_put(EXT_RESET,true);
        reset_requested = false;
    }
    // check on watchdog unless we explicitly ended or errored
    if (mia_active() && action_result == -1)
    {
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(now, action_watchdog_timer) < 0)
        {
            printf("****TIMEOUT****\n");
            printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
            IOREGS(0x03B0),IOREGS(0x03B1),IOREGS(0x03B2),IOREGS(0x03B3),
            IOREGS(0x03B4),IOREGS(0x03B5),IOREGS(0x03B6),IOREGS(0x03B7),
            IOREGS(0x03B8));
            printf("%d-%d %s\n", rw_pos, rw_end, action_state == action_state_read ? "R" : "N");
            action_result = -3;
            main_stop();
        }
    }
    if(mia_io_errors != prev_io_errors){
        if(mia_io_errors > 0){
            printf("!IO error %ld\n", mia_io_errors);
        }
        prev_io_errors = mia_io_errors;
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
    /*
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF)
            mbuf[len] = REGS(addr + len);
        else
            mbuf[len] = 0;
    while (len && (addr + len > 0xFF00))
        if (addr + --len <= 0xFFFF)
            mbuf[len] = 0;
    */
    if(addr + len > 0xFFFF)
        len = (0x10000 - addr);
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
    /*
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF && mbuf[len] != REGS(addr + len))
            action_result = addr + len;
    while (len && (addr + len > 0xFF00))
        --len;
    */
    if(addr + len > 0xFFFF)
        len = (0x10000 - addr);
    if (!len || action_result != -1)
        return;
    rw_addr = addr;
    rw_end = len;
    rw_pos = 0;
    action_state = action_state_verify;
    main_run();
}
static uint8_t prev_ctrl;
void mia_write_buf(uint16_t addr)
{
    assert(!cpu_active());
    // avoid forbidden area
    uint16_t len = mbuf_len;
    /*
    while (len && (addr + len > 0xFFFA))
        if (addr + --len <= 0xFFFF)
            REGS(addr + len) = mbuf[len];
    while (len && (addr + len > 0xFF00))
        len--;
    */
    if(addr + len > 0xFFFF)
        len = (0x10000 - addr);
    if (!len)
        return;
    rw_addr = addr;
    rw_end = len;
    rw_pos = 0;
    action_state = action_state_write;
    main_run();
}


//Called by action loop, needs to be fast so using bare PIO accesses
inline __attribute__((always_inline)) void mia_set_rom_ram_enable_inline(bool device_rom, bool basic_rom){
    //mia_set_rom_read_enable(device_rom || basic_rom); 
    //MIA_ROM_READ_PIO->ctrl = (MIA_ROM_READ_PIO->ctrl & 0xf & ~(1u << MIA_ROM_READ_SM)) | (bool_to_bit(device_rom || basic_rom) << MIA_ROM_READ_SM);
    //MIA_ROM_READ_PIO->sm[MIA_ROM_READ_SM].instr = (device_rom || basic_rom) ? PIO_OP_ON_3 : PIO_OP_OFF;
    
    //high bank overlay ram only enabled when device_rom is disabled
    //mia_enable_overlay_ram(overlay_ram, !device_rom && overlay_ram);
    bool overlay_ram = !basic_rom;
    //MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & 0xf & ~(1u << MIA_MAP_SM1)) | (bool_to_bit(overlay_ram) << MIA_MAP_SM1);
    //MIA_MAP_PIO->ctrl = (MIA_MAP_PIO->ctrl & 0xf & ~(1u << MIA_MAP_SM2)) | (bool_to_bit(!device_rom && overlay_ram) << MIA_MAP_SM2);
    MIA_MAP_PIO->sm[MIA_MAP_SM1].instr = ( (overlay_ram) ? PIO_OP_ON_C : PIO_OP_OFF );
    MIA_MAP_PIO->sm[MIA_MAP_SM2].instr = ( (!device_rom && overlay_ram) ? PIO_OP_ON_E : PIO_OP_OFF );
    ///MIA_ROM_READ_PIO->sm[MIA_ROM_READ_SM].instr = (device_rom || basic_rom ? PIO_OP_ON_3 : PIO_OP_OFF);
    //device rom loaded in bank2, basic rom loaded in bank3
    //mia_set_rom_addr(basic_rom ? (uintptr_t)oric_bank3 : (uintptr_t)oric_bank2);
        //MIA_READ_PIO->txf[MIA_READ_ADDR_SM] = (basic_rom ? (uintptr_t)oric_bank3 : (uintptr_t)oric_bank2) >> 14;
    MIA_READ_PIO->txf[MIA_READ_ADDR_SM] = (basic_rom ? (uintptr_t)(0x2000C000 >> 14) : (uintptr_t)(0x20008000 >> 14));    

    map_flag_basic = basic_rom;
    map_flag_device = device_rom;
}
void mia_set_rom_ram_enable(bool device_rom, bool basic_rom){
    mia_set_rom_ram_enable_inline(device_rom, basic_rom);
}

static inline __attribute__((always_inline)) uint8_t wait_act_data(void){
    __compiler_memory_barrier();
    while((MIA_ACT_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MIA_ACT_SM)))){}
    return (MIA_ACT_PIO->rxf[MIA_ACT_SM])>>16 & 0xFF;
}

int mia_read_dma_channel;

//#define CASE_READ(addr) (addr & 0x1F)
//#define CASE_WRITE(addr) (0x20 | (addr & 0x1F))
#define CASE_READ(addr) (addr & 0x000000FF)
#define CASE_WRITE(addr) (0x01000000 | (addr & 0x000000FF))
#define MIA_RW0 IOREGS(0x03A4)
#define MIA_STEP0 *(int8_t *)&IOREGS(0x03A5)
#define MIA_ADDR0 IOREGSW(0x03A6)
#define MIA_RW1 IOREGS(0x03A8)
#define MIA_STEP1 *(int8_t *)&IOREGS(0x03A9)
#define MIA_ADDR1 IOREGSW(0x03AA)
static __attribute__((optimize("O1"))) void act_loop(void)
{
    // In here we bypass the usual SDK calls as needed for performance.
    while (true)
    {
        if (!(MIA_ACT_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MIA_ACT_SM))))
        {
            uint32_t rw_data_addr = MIA_ACT_PIO->rxf[MIA_ACT_SM];

            /*
            //Track errors and stop processing if address is wrong (0x03xx)
            if((rw_data_addr & 0x0000FF00) != 0x00000300){
                mia_io_errors++;
                continue;
            }
            */
            if(!(rw_data_addr & 0x01000000)){  //Handle io page reads. Save PIO cycles
                (&dma_hw->ch[mia_read_dma_channel])->al3_read_addr_trig = (uintptr_t)((uint32_t)&iopage | (rw_data_addr & 0xFF));
                if((mia_iopage_enable_map[(rw_data_addr & 0x00000080)>>7]) & (1UL << ((rw_data_addr >> 2) & 0x1F))){
                    MIA_IO_READ_PIO->irq = 1u << 5;
                }
            }
            /*
                Writes are served by a second FIFO word from the PIO program when data is valid.
                For needed performance we wait for that message after we have decoded the address within each write case.
                Any write case that needs both special handling and storing of the data needs to explicitly store the data
                within the case section.
            */
            
            uint8_t data;
            volatile uint8_t sink;
            uint32_t fifo_data;
                switch(rw_data_addr & 0x010000FF){
                    //TAP Motor sense (snooping VIA writes)
                    case CASE_WRITE(0x300):
                        data = wait_act_data();
                        tap_act(data);              
                        break;
                    //Microdisc Device Write Registers
                    case CASE_WRITE(DSK_IO_CMD):    //CMD and STAT are overlayed R/W
                        dsk_reg_irq = 0x80;         //Clear IRQ on write (active low)
                        dsk_reg_status = 0x01;     //Busy
                        fifo_data = 0x80000000 | (IOREGS(DSK_IO_TRACK) << 24) | (IOREGS(DSK_IO_SECT) << 16) | (prev_ctrl << 8);
                        data = wait_act_data();
                        //dsk_act(data);              //Process command
                        sio_hw->fifo_wr = fifo_data | data;
                        break;
                    case CASE_WRITE(DSK_IO_DATA):
                        if(dsk_state == DSK_WRITE){
                            dsk_reg_status &= 0b11111101;
                            IOREGS(DSK_IO_DRQ) = 0x80;          //After data write to synch with dsk_task()
                            data = wait_act_data();
                            dsk_buf[*dsk_active_pos] = data;
                        }else{
                            data = wait_act_data();
                            IOREGS(DSK_IO_DATA) = data;
                        }
                        break;
                    case CASE_WRITE(DSK_IO_CTRL):   //CTRL and IRQ are overlayed
                        data = wait_act_data();
                        //Bits 7:EPROM 6-5:drv_sel 4:side_sel 3:DDEN 2:Read CLK/2 1:ROM/RAM 0:IRQ_EN
                        //[7] 0:device rom enabled
                        //[1] 0:basic rom disabled
                        mia_set_rom_ram_enable_inline(!(data & 0x80), !!(data & 0x02)); //device_rom,basic_rom
                        //dsk_set_ctrl(data); //Handling of DSK related bits
                        if((prev_ctrl ^ data) & 0x01)                  //Only send changed dsk bits and only interrupt enable flag
                            sio_hw->fifo_wr = 0x00000000 | (data << 8);   //Transfer with CMD in dsk_act
                        prev_ctrl = data;    
                        break;
                    //ACIA Device Write Registers 0x380-0x383
                    case CASE_WRITE(ACIA_IO_DATA):
                        data = wait_act_data();
                        acia_write(data);
                        break;
                    case CASE_WRITE(ACIA_IO_STAT):
                        data = wait_act_data();
                        acia_reset(false);
                        break;
                    case CASE_WRITE(ACIA_IO_CMD):
                        data = wait_act_data();
                        acia_cmd(data);
                        break;
                    case CASE_WRITE(ACIA_IO_CTRL):
                        data = wait_act_data();
                        acia_ctrl(data);
                        break;
                    //RP6502-like API interface write registers
                    case CASE_WRITE(0x03AF): // OS function call
                        data = wait_act_data();
                        IOREGS(0x03AF) = data;
                        api_return_blocked();
                        if (data == 0x00) // zxstack()
                        {
                            API_STACK = 0;
                            xstack_ptr = XSTACK_SIZE;
                            api_return_ax(0);
                        }
                        else if (data == 0xFF) // exit()
                        {
                            reset_requested = true;
                            main_stop();
                        }
                        break;
                    case CASE_WRITE(0x03AC): // xstack
                        data = wait_act_data();
                        if (xstack_ptr)
                            xstack[--xstack_ptr] = data;
                        API_STACK = xstack[xstack_ptr];
                        break;
                    case CASE_WRITE(0x03AB): // Set XRAM >ADDR1
                        data = wait_act_data();
                        IOREGS(0x03AB) = data;
                        MIA_RW1 = xram[MIA_ADDR1];
                        break;
                    case CASE_WRITE(0x03AA): // Set XRAM <ADDR1
                        data = wait_act_data();
                        IOREGS(0x03AA) = data;
                        MIA_RW1 = xram[MIA_ADDR1];
                        break;
                    case CASE_WRITE(0x03A8): // W XRAM1
                        data = wait_act_data();
                        xram[MIA_ADDR1] = data;
                        //PIX_SEND_XRAM(RIA_ADDR1, data);
                        MIA_RW0 = xram[MIA_ADDR0];
                        MIA_ADDR1 += MIA_STEP1;
                        MIA_RW1 = xram[MIA_ADDR1];
                        break;
                    case CASE_WRITE(0x03A7): // Set XRAM >ADDR0
                        data = wait_act_data();
                        IOREGS(0x03A7) = data;
                        MIA_RW0 = xram[MIA_ADDR0];
                        break;
                    case CASE_WRITE(0x03A6): // Set XRAM <ADDR0
                        data = wait_act_data();
                        IOREGS(0x03A6) = data;
                        MIA_RW0 = xram[MIA_ADDR0];
                        break;
                    case CASE_WRITE(0x03A4): // W XRAM0
                        data = wait_act_data();
                        xram[MIA_ADDR0] = data;
                        //PIX_SEND_XRAM(RIA_ADDR0, data);
                        MIA_RW1 = xram[MIA_ADDR1];
                        MIA_ADDR0 += MIA_STEP0;
                        MIA_RW0 = xram[MIA_ADDR0];
                        break;
                    case CASE_WRITE(0x03A3): // ULA pattern match
                        data = wait_act_data();
                        MIA_ULA_PIO->txf[MIA_ULA_SM] = 0x83848500 | data;
                        IOREGS(0x03A3) = 0x00;
                        break;
                    case CASE_WRITE(0x03A1): // UART Tx
                        data = wait_act_data();
                        if (com_tx_writable())
                            com_tx_write(data);
                        if (com_tx_writable())
                            IOREGS(0x03A0) |= 0b10000000;
                        else
                            IOREGS(0x03A0) &= ~0b10000000;
                        break;
                    case CASE_WRITE(0x03BD): // action read
                        if (++rw_pos >= rw_end){
                            IOREGS(0x03B9) = 0xFE;
                            reset_requested = true;
                            action_result = -2;
                            main_stop();
                        }else{
                            IOREGSW(0x03B2) = ++rw_addr;
                        }
                        data = wait_act_data();
                        mbuf[rw_pos-1] = data;
                        break;                        
                    case CASE_WRITE(0x03BC): // action verify
                        data = wait_act_data();
                        if (mbuf[rw_pos] != data && action_result < 0)
                            action_result = rw_addr;
                        if (++rw_pos >= rw_end){
                            IOREGS(0x03B9) = 0xFE;
                            reset_requested = true;
                            action_result = -2;
                            main_stop();
                        }else{
                            IOREGSW(0x03B2) = ++rw_addr;
                        }
                        break;
                    case CASE_WRITE(0x319):
                        data = wait_act_data();
                        //Disabled write for LOCI identity marker
                        break;
        
                    //Microdisc Device Read Register
                    case CASE_READ(DSK_IO_CMD):
                        dsk_reg_irq = 0x80;         //Clear IRQ on read (active low)
                        break;                    
                    case CASE_READ(DSK_IO_DATA):
                        dsk_reg_status = (dsk_reg_status & 0b11111100) | dsk_next_busy;
                        IOREGS(DSK_IO_DRQ) = 0x80;
                        //dsk_rw(false,0x00);
                        break;
                    //ACIA Device Read Registers 0x380-0x383
                    case CASE_READ(ACIA_IO_DATA):
                        acia_read();
                        break;
                    case CASE_READ(ACIA_IO_STAT):
                        acia_clr_irq();
                        break;
                    case CASE_READ(0x03AC): // xstack
                        if (xstack_ptr < XSTACK_SIZE)
                            ++xstack_ptr;
                        API_STACK = xstack[xstack_ptr];
                        break;
                    case CASE_READ(0x03A8): // R XRAM1
                        MIA_ADDR1 += MIA_STEP1;
                        MIA_RW1 = xram[MIA_ADDR1];
                        break;
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
                            IOREGS(0x03A0) &= ~0b01000000;
                            IOREGS(0x03A2) = 0;
                        }
                        break;
                    }
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
                    //RP6502-like API interface read registers
                    case CASE_READ(0x03B6): // action write
                        if(action_state == action_state_write){
                            if (++rw_pos >= rw_end){
                                IOREGS(0x03B8) = 0xFE;
                                reset_requested = true;
                                action_result = -2;
                                main_stop();
                            }else{
                                IOREGS(0x03B2) = mbuf[rw_pos];
                                IOREGSW(0x03B4) = ++rw_addr;
                            }
                        }
                        break;
                    case CASE_READ(0x03BB):
                        snoop_flag = true;
                        break;
                    default:
                        //Default register write handling
                        if(rw_data_addr & 0x01000000){
                            data = wait_act_data();
                            IOREGS(rw_data_addr & 0xFFFF) = data;

                        }
                        break;
            }
            sink = data;        //Avoid wait_act_data() calls to be optimised away
                        /*
                        case CASE_WRITE(0x03B0): // IRQ Enable
                            irq_enabled = data;
                            __attribute__((fallthrough));
                        case CASE_READ(0x03B0): // IRQ ACK
                            //gpio_put(CPU_IRQB_PIN, true);
                            break;
                        */

            /*
                if((rw_data_addr & 0x0000FFF0) == 0x000003B0){
                    ssd_got_action_word = true;
                    if(rw_data_addr >> 24){
                        ssd_action_word = rw_data_addr;
                        ssd_action_is_wr = true;
                    }else{
                        static uint8_t cnt = 0;
                        ssd_action_rword = rw_data_addr | (cnt++ <<16);
                        //ssd_action_rword = (rw_data_addr & 0xFF00FFFF) | (IOREGS(rw_data_addr & 0xFFFF) << 16);
                        ssd_action_is_wr = false;
                    }
                }
            */
        } /* if fifo */        
    } /* while loop */
}

/*
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
*/
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
    sm_config_set_out_pins(&config_a, D_PIN_BASE, 8);
    
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
    channel_config_set_read_increment(&data_dma, false);
    channel_config_set_write_increment(&data_dma, false);
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
    channel_config_set_write_increment(&addr_dma, false);
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
    sm_config_set_out_pins(&config, D_PIN_BASE, 8);
    sm_config_set_set_pins(&config, DIR_PIN, 1);
    pio_sm_set_consecutive_pindirs(MIA_IO_READ_PIO, MIA_IO_READ_SM, DIR_PIN, 1, true);
    //pio_sm_set_consecutive_pindirs(MIA_WRITE_PIO, MIA_WRITE_SM, CPU_PHI2_PIN, 1, true);
    pio_sm_init(MIA_IO_READ_PIO, MIA_IO_READ_SM, offset, &config);
    pio_sm_set_enabled(MIA_IO_READ_PIO, MIA_IO_READ_SM, true);

}

static void mia_rom_read_pio_init(void)
{
    
    // PIO to manage 6502 reads to Oric ROM area 0xC000-0xFFFF
    uint offset = pio_add_program(MIA_ROM_READ_PIO, &mia_rom_read_program);
    pio_sm_config config = mia_rom_read_program_get_default_config(offset);
    sm_config_set_in_pins(&config, A14_PIN);
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
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0x03 );    //Constant for multiple matching
    pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0xFF );    //Constant for multiple matching - start as off
    pio_sm_exec_wait_blocking(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_init(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, offset, &config);
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, 0xE000 >> 13);    //ROM 8kB at 0xE000
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM2, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_READ_PIO, MIA_ROM_READ_SM2, pio_encode_mov(pio_x, pio_osr));
    pio_sm_set_enabled(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, true);
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
    sm_config_set_in_pins(&config, D_PIN_BASE);             //Prep to get A15 at MSB
    sm_config_set_in_shift(&config, false, false, 0);
    sm_config_set_out_shift(&config, false, false, 0);  //Shift left, MSB out first
    sm_config_set_set_pins(&config, MAP_PIN, 1);
    //pio_gpio_init(MIA_MAP_PIO, MAP_PIN);
    pio_sm_init(MIA_MAP_PIO, MIA_MAP_SM1, offset, &config);
    pio_sm_set_consecutive_pindirs(MIA_MAP_PIO, MIA_MAP_SM1, MAP_PIN, 1, true);
    //pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM1, 0xC000 >> 13);    //Overlay RAM 8kB at 0xC000
    pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM1, 0xFF);    //Overlay RAM 8kB at 0xC000 - start as off
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM1, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM1, pio_encode_mov(pio_x, pio_osr));
    pio_sm_init(MIA_MAP_PIO, MIA_MAP_SM2, offset, &config);
    //pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM2, 0xE000 >> 13);    //Overlay RAM 8kB at 0xE000
    pio_sm_put(MIA_MAP_PIO, MIA_MAP_SM2, 0xFF);    //Overlay RAM 8kB at 0xE000 - start as off
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM2, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_MAP_PIO, MIA_MAP_SM2, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_put(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, 0x0310 >> 4);
    //pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_pull(false, true));
    //pio_sm_exec_wait_blocking(MIA_WRITE_PIO, MIA_WRITE_SM, pio_encode_mov(pio_y, pio_osr));
    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM1, true);
    pio_sm_set_enabled(MIA_MAP_PIO, MIA_MAP_SM2, true);   
}

static uint mia_ula_prg_offset;
uint mia_get_ula_prg_offset(void){
    return mia_ula_prg_offset;}

static void mia_ula_pio_init(void)
{
    // PIO to manage ULA mode snooping
    uint offset = pio_add_program(MIA_ULA_PIO, &mia_ula_program);
    mia_ula_prg_offset = offset;
    pio_sm_config config = mia_ula_program_get_default_config(offset);
    sm_config_set_in_pins(&config, D_PIN_BASE);
    sm_config_set_in_shift(&config, false, false, 0);
    pio_sm_init(MIA_ULA_PIO, MIA_ULA_SM, offset, &config);
    pio_sm_put(MIA_ULA_PIO, MIA_ULA_SM, 0x83848586);    //Pattern to match - 4 ink changes in a row
    pio_sm_exec_wait_blocking(MIA_ULA_PIO, MIA_ULA_SM, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(MIA_ULA_PIO, MIA_ULA_SM, pio_encode_mov(pio_x, pio_osr));
    //pio_sm_exec_wait_blocking(MIA_ULA_PIO, MIA_ULA_SM, pio_encode_set(pio_x, 0x18 >>3 ));
    pio_sm_set_enabled(MIA_ULA_PIO, MIA_ULA_SM, true);

    int mode_chan = dma_claim_unused_channel(true);
    int trig_chan = dma_claim_unused_channel(true);

    dma_channel_config mode_dma = dma_channel_get_default_config(mode_chan);
    //channel_config_set_high_priority(&mode_dma, true);
    channel_config_set_dreq(&mode_dma, pio_get_dreq(MIA_ULA_PIO, MIA_ULA_SM, false));
    channel_config_set_transfer_data_size(&mode_dma, DMA_SIZE_8);
    channel_config_set_read_increment(&mode_dma, false);
    channel_config_set_write_increment(&mode_dma, false);
    channel_config_set_chain_to(&mode_dma, trig_chan);

    dma_channel_configure(
        mode_chan,
        &mode_dma,
        &IOREGS(0x03A3),                    // dst
        &MIA_ULA_PIO->rxf[MIA_ULA_SM],      // src
        0xFFFFFFFF,                         // max
        true);

    ula_trig_addr = (uint32_t)&IOREGS(0x03A3);
    dma_channel_config trig_dma = dma_channel_get_default_config(trig_chan);
    //channel_config_set_high_priority(&trig_dma, true);
    channel_config_set_read_increment(&trig_dma, false);
    channel_config_set_write_increment(&trig_dma, false);
    channel_config_set_chain_to(&trig_dma, mode_chan);
    dma_channel_configure(
        trig_chan,
        &trig_dma,
        &dma_channel_hw_addr(mode_chan)->write_addr,     // dst
        &ula_trig_addr,                                  // src
        1,
        false);

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
    //Don't Enable levelshifters yet
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
        //gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_2MA);
    }
    //gpio_set_drive_strength(nIRQ_PIN, GPIO_DRIVE_STRENGTH_2MA);
    for (int i = D_PIN_BASE; i < D_PIN_BASE + 8; i++)
        pio_gpio_init(MIA_READ_PIO, i);

    pio_gpio_init(MIA_READ_PIO, DIR_PIN);
    pio_gpio_init(MIA_MAP_PIO, MAP_PIN);
    //gpio_set_drive_strength(DIR_PIN, GPIO_DRIVE_STRENGTH_2MA);
    //gpio_set_drive_strength(MAP_PIN, GPIO_DRIVE_STRENGTH_2MA);
    
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
   mia_iopage_enable_map[0] = 0;
   mia_iopage_enable_map[1] = 0;
    //Enable response on IO registers 0x310-0x31B (DSK/TAP)
    for(int i=(0x10 >> 2); i<=(0x1B >> 2); i++){
        mia_iopage_enable_map[0] |= (0x1UL << (i & 0x1F));
    }
    //Enable response on IO registers 0x380-0x383 (ACIA)
    for(int i=(0x80 >> 2); i<=(0x83 >> 2); i++){
        mia_iopage_enable_map[1] |= (0x1UL << (i & 0x1F));
    }
    //Enable response on IO registers 0x3A0-0x3BF (LOCI)
    for(int i=(0xA0 >> 2); i<=(0xBF >> 2); i++){
        mia_iopage_enable_map[1] |= (0x1UL << (i & 0x1F));
    }
   
   //LOCI identity marker
   IOREGS(0x0319) = 'L';

    mia_boot_settings = 0;
    mia_boot_state = MIA_IDLE;

    // the inits
    // Same PIO order matters for timing tuning
    mia_map_pio_init(); //Must be first for MAP tuning
    mia_act_pio_init();
    //mia_write_pio_init();
    mia_ula_pio_init();
    
    //Same PIO order matters timing tuning
    mia_read_pio_init();    //read_data, read_addr
    mia_io_read_pio_init();
    mia_rom_read_pio_init();
}

void mia_reclock(uint16_t clkdiv_int, uint8_t clkdiv_frac)
{
    //pio_sm_set_clkdiv_int_frac(MIA_WRITE_PIO, MIA_WRITE_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_READ_PIO, MIA_READ_DATA_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_READ_PIO, MIA_READ_ADDR_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_IO_READ_PIO, MIA_IO_READ_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_ROM_READ_PIO, MIA_ROM_READ_SM, clkdiv_int, clkdiv_frac);
    //pio_sm_set_clkdiv_int_frac(MIA_ROM_READ_PIO, MIA_ROM_READ_SM2, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_ACT_PIO, MIA_ACT_SM, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_MAP_PIO, MIA_MAP_SM1, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_MAP_PIO, MIA_MAP_SM2, clkdiv_int, clkdiv_frac);
    pio_sm_set_clkdiv_int_frac(MIA_ULA_PIO, MIA_ULA_SM, clkdiv_int, clkdiv_frac);
}

void mia_api_boot(void){
    bool rom_opened;
    mia_boot_settings = API_A;
    if(!!(mia_boot_settings & MIA_BOOTSET_FDC)){
        rom_opened = rom_load("MICRODISC",9);
        if(!rom_opened)
            rom_opened = rom_load_raw("microdis.rom",0xA000);
        if(!rom_opened){
            printf("!rom_load microdisc failed\n");
            api_return_errno(API_EMFILE);
        }else{
            printf("DEV ROM loaded ok\n");
            if(!(mia_boot_settings & MIA_BOOTSET_RESUME)){
                dsk_init();
            }
            dsk_pause(false);
            if(mia_boot_settings & MIA_BOOTSET_FAST){
                /*
                if((mia_boot_settings & MIA_BOOTSET_RESUME)){    
                    mia_set_rom_ram_enable(saved_map_flag_device,saved_map_flag_basic);         //Setup address for device ROM
                }else{
                    mia_set_rom_ram_enable(true,false);
                }
                */
                //Returns control with api_return_boot() when ROM has loaded
                printf("Fast boot ON\n");
            }else{
                api_return_ax(0);
                main_stop();
            }
            mia_boot_state = MIA_LOADING_DEVROM;
        }
    }else{
        rom_opened = rom_load_mounted();
        if(!rom_opened){
            if(!!(mia_boot_settings & MIA_BOOTSET_B11)){
                rom_opened = rom_load("BASIC11",7);
                if(!rom_opened)
                    rom_opened = rom_load_raw("basic11.rom",0xC000);
                if(!rom_opened)
                    rom_opened = rom_load_raw("basic11b.rom",0xC000);
            }else{
                rom_opened = rom_load("BASIC10",7);
                if(!rom_opened)
                    rom_opened = rom_load_raw("basic10.rom",0xC000);
            }
        }
        if(!rom_opened){
            printf("!rom_load basic failed\n");
            api_return_errno(API_EMFILE);
        }else{
            printf("BASIC ROM loaded ok\n");
            if(!!(mia_boot_settings & MIA_BOOTSET_FAST)){
                //mia_set_rom_ram_enable(false,true);
                //Returns control with api_return_boot() when ROM has loaded
                printf("Fast boot ON\n");
            }else{
                api_return_ax(0);
                main_stop();
            }
            mia_boot_state = MIA_LOADING_BIOS;
        }
    }
}