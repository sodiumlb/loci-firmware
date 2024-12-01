/*
 * Copyright (c) 2023 Rumbledethumps
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** 
 * Emulating Oric TAP cassette image format 
*/

#include "oric/tap.h"
#include "main.h"
#include "sys/lfs.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "sys/led.h"
#include "fatfs/ff.h"
#include "api/api.h"

//Track buffer
//volatile uint8_t tap_buf[1024];

enum TAP_TYPE { EMPTY = 0, LFS, FAT };
typedef struct _tap_drive_t {
    enum TAP_TYPE type;
    lfs_file_t *lfs_file;
    FIL *fat_file;
    uint32_t counter;
    uint32_t size;
    uint16_t encoded_byte;
    uint8_t lead_in;
    uint8_t filename[16];
    uint8_t bit_counter;
    bool dirty;
    bool motor_on;
} tap_drive_t;

typedef struct _tap_header_t {
    uint8_t flag_int;
    uint8_t flag_str;
    uint8_t type;
    uint8_t autorun;
    uint8_t end_addr_hi;
    uint8_t end_addr_lo;
    uint8_t start_addr_hi;
    uint8_t start_addr_lo;
    uint8_t reserved;
    uint8_t filename[16];
} tap_header_t;

#define TAP_LEAD_IN_LEN 8

tap_drive_t tap_drive = {0};


//TODO REMOVE Status is not currently in use by the API and is rottening
#define TAP_STAT_NOT_READY (0b10000000)
#define TAP_STAT_WPROT     (0b01000000)
#define TAP_STAT_BUSY      (0b00000001)

#define TAP_CMD_PLAY        (0x01)
#define TAP_CMD_REC         (0x02)
#define TAP_CMD_REW         (0x03)
#define TAP_CMD_READ_BIT    (0x04)
#define TAP_CMD_FFW         (0x05)

uint8_t tap_parity(uint8_t byte){
    uint8_t parity = 1;
    for(uint8_t i=0; i<8; i++){
        if( (byte>>i) & 0x01 )
            parity++; 
    }
    return parity & 0x01;
}

uint16_t tap_encode_byte(uint8_t byte){
    return 0b11110000000000 | (tap_parity(byte) << 9) | byte << 1;
}

//volatile uint8_t *tap_reg_stat = &IOREGS(TAP_IO_STAT);

void tap_set_status(uint8_t bit, bool val){
    if(val){
        IOREGS(TAP_IO_STAT) |= bit;
    }else{
        IOREGS(TAP_IO_STAT) &= ~bit;
    }
}


//Assumes file has been opened first
bool tap_mount_lfs(lfs_file_t *lfs_file){
    //uint8_t header_buf[20];
    lfs_file_rewind(&lfs_volume, lfs_file);
    //if(lfs_file_read(&lfs_volume, lfs_file, (void *)header_buf, 20) == 20){
    tap_drive.type = LFS;
    tap_drive.lfs_file = lfs_file;
    tap_drive.counter = 0;
    tap_drive.bit_counter = 0;
    tap_drive.size = lfs_file_size(&lfs_volume, lfs_file);
    tap_set_status(TAP_STAT_NOT_READY,false);
    return true;
/*    }else{
        return false;
    }
*/
}
bool tap_mount_fat(FIL* fat_file){
    f_rewind(fat_file);
    tap_drive.type = FAT;
    tap_drive.fat_file = fat_file;
    tap_drive.counter = 0;
    tap_drive.bit_counter = 0;
    tap_drive.size = f_size(fat_file);
    tap_set_status(TAP_STAT_NOT_READY,false);
    return true;
}

bool tap_is_mounted(void){
    return tap_drive.type != EMPTY;
}


void tap_umount(void){
    tap_set_status(TAP_STAT_NOT_READY,true);
    switch(tap_drive.type){
        case LFS:
            lfs_free_file_config(tap_drive.lfs_file);
            lfs_file_close(&lfs_volume, tap_drive.lfs_file);
            break;
        case FAT:
            f_close(tap_drive.fat_file);
            break;
        default:
            break;
    }
    tap_drive.type = EMPTY;
    tap_drive.counter = 0;
    tap_drive.bit_counter = 0;
    tap_drive.lead_in = 0;
    tap_drive.dirty = false;
}

void tap_rewind(void){
    switch(tap_drive.type){
        case LFS:
            lfs_file_rewind(&lfs_volume, tap_drive.lfs_file);
            break;
        case FAT:
            f_rewind(tap_drive.fat_file);
            break;
        default:
            break;
    }
    tap_drive.counter = 0;
    tap_drive.bit_counter = 0;
    tap_drive.lead_in = 0;
}

void tap_ffw(void){
    switch(tap_drive.type){
        case LFS:
            tap_drive.counter = lfs_file_seek(&lfs_volume, tap_drive.lfs_file, 0, LFS_SEEK_END);
            break;
        case FAT:
            f_lseek(tap_drive.fat_file,f_size(tap_drive.fat_file));
            tap_drive.counter = f_tell(tap_drive.fat_file);
            break;
        default:
            break;
    }
}

uint32_t tap_seek(uint32_t pos){
    uint32_t new_pos = pos;
    switch(tap_drive.type){
        case LFS:
            if(lfs_file_seek(&lfs_volume, tap_drive.lfs_file, pos, LFS_SEEK_SET ) < 0)
                new_pos = lfs_file_seek(&lfs_volume, tap_drive.lfs_file, 0, LFS_SEEK_END);
            break;
        case FAT:
            f_lseek(tap_drive.fat_file, pos);
            new_pos = f_tell(tap_drive.fat_file);
            break;
        default:
            new_pos = 0;
            break;
    }
    tap_drive.counter = new_pos;
    if(new_pos == 0)
        tap_drive.lead_in = 0;
    return new_pos;
}

enum TAP_STATE { TAP_IDLE, TAP_READY, TAP_READ_BYTE, TAP_WRITE, TAP_READ_BIT } tap_state = TAP_IDLE;

uint8_t tap_read_byte(void){
    bool fs_ret = false;
    UINT br;
    FRESULT fr;
    uint8_t data;
    if(tap_drive.lead_in < TAP_LEAD_IN_LEN){
        data = 0x16;
        tap_drive.lead_in++;
    }else{
        switch(tap_drive.type){
            case LFS:
                if(lfs_file_read(&lfs_volume, tap_drive.lfs_file, (void *) &data, 1) == 1){
                    tap_drive.counter++;
                    fs_ret = true;
                };
                break;
            case FAT:
                fr = f_read(tap_drive.fat_file, (void *) &data, 1, &br);
                if(fr == FR_OK && br == 1){
                    tap_drive.counter++;
                    fs_ret = true;
                }
                break;
            default:
                break;
                //TODO error
        }
        //Return zero when nothing is read 
        if(!fs_ret){
            data = 0x00;
        }
    }

    return data;
}

void tap_write_byte(uint8_t data){
    bool fs_ret = false;
    UINT bw;
    FRESULT fr;
    switch(tap_drive.type){
        case LFS:
            if(lfs_file_write(&lfs_volume, tap_drive.lfs_file, (void *) &data, 1) == 1){
                tap_drive.counter++;
                fs_ret = true;
            };
            break;
        case FAT:
            fr = f_write(tap_drive.fat_file, (void *) &data, 1, &bw);
            if(fr == FR_OK && bw == 1){
                tap_drive.counter++;
                fs_ret = true;
            }
            break;
        default:
            break;
            //TODO error
    }
    if(fs_ret)
        tap_drive.dirty = true;
}

void tap_init(void){
    IOREGS(TAP_IO_STAT) = TAP_STAT_NOT_READY; 
    IOREGS(TAP_IO_CMD)  = 0x00; 
    IOREGS(TAP_IO_DATA) = 0x00;
    tap_state = TAP_IDLE;
    tap_drive.type = EMPTY;
    tap_drive.counter = 0;
    tap_drive.bit_counter = 0;
    tap_drive.dirty = false;
}

void tap_task(void){
    switch(tap_state){
        case TAP_IDLE:
            if(tap_drive.motor_on){
                tap_state = TAP_READY;
                led_set(true);
            }
            break;
        case TAP_READY:
            if(tap_drive.motor_on){
                switch(IOREGS(TAP_IO_CMD)){
                    case TAP_CMD_PLAY:
                        tap_state = TAP_READ_BYTE;
                        tap_set_status(TAP_STAT_BUSY,true);
                        break;
                    case TAP_CMD_REC:
                    //TODO setup initial write file
                        tap_set_status(TAP_STAT_BUSY,true);
                        tap_state = TAP_WRITE;
                        break;
                    case TAP_CMD_REW:
                        tap_rewind();
                        IOREGS(TAP_IO_CMD) = 0x00;
                        tap_state = TAP_READY;
                    break;
                    case TAP_CMD_READ_BIT:
                        tap_set_status(TAP_STAT_BUSY,true);
                        tap_state = TAP_READ_BIT;
                        break;
                    case TAP_CMD_FFW:
                        tap_ffw();
                        IOREGS(TAP_IO_CMD) = 0x00;
                        tap_state = TAP_READY;
                    default:
                        break;
                }   
            }else{
                led_set(false);
                if(tap_drive.dirty){
                    switch(tap_drive.type){
                        case LFS:
                            lfs_file_sync(&lfs_volume, tap_drive.lfs_file);
                            break;
                        case FAT:
                            f_sync(tap_drive.fat_file);
                            break;
                        default:
                            break;
                    }
                    tap_drive.dirty = false;
                }
                tap_state = TAP_IDLE;
            }
            break;
        case TAP_READ_BYTE:
            if(IOREGS(TAP_IO_STAT) & TAP_STAT_BUSY){
                IOREGS(TAP_IO_DATA) = tap_read_byte();
                tap_set_status(TAP_STAT_BUSY,false);
                IOREGS(TAP_IO_CMD) = 0x00;
                tap_state = TAP_READY;
            }
            break;
        case TAP_READ_BIT:
            if(IOREGS(TAP_IO_STAT) & TAP_STAT_BUSY){
                if(tap_drive.bit_counter == 0){
                    tap_drive.encoded_byte = tap_encode_byte(tap_read_byte());
                }
                IOREGS(TAP_IO_DATA) = (tap_drive.encoded_byte >> tap_drive.bit_counter) & 0x01;
                if(++tap_drive.bit_counter >= 14)
                    tap_drive.bit_counter = 0;
                tap_set_status(TAP_STAT_BUSY,false);
                IOREGS(TAP_IO_CMD) = 0x00;
                tap_state = TAP_READY;
            }
            break;
        case TAP_WRITE:
            if(IOREGS(TAP_IO_STAT) & TAP_STAT_BUSY){
                tap_write_byte(IOREGS(TAP_IO_DATA));
                tap_set_status(TAP_STAT_BUSY,false);
                IOREGS(TAP_IO_CMD) = 0x00;
                tap_state = TAP_READY;
            }
            break;
        default:
            tap_state = TAP_IDLE;
    }
}

//Show tape motor status on LED
void __not_in_flash() tap_act(uint8_t data){
    tap_drive.motor_on = !!(data & 0x40);
}

//Set/Get counter (aka seek).
void tap_api_seek(void){
    uint32_t pos;
    pos = API_AXSREG;
    if(pos >= tap_drive.size)
        pos = tap_drive.size-1;
    if(tap_is_mounted())
        api_return_axsreg(tap_seek(pos));
    else
        api_return_errno(API_ENODEV);
}

//Get counter
void tap_api_tell(void){
        api_return_axsreg(tap_drive.counter);
}

bool tap_seek_next_header(uint32_t start, uint32_t end, uint32_t* pos){
    for(uint32_t i=start; i<end-4; i++){
        //End of sync mark
        if( mbuf[i]   == 0x16 &&
            mbuf[i+1] == 0x16 &&
            mbuf[i+2] == 0x16 &&
            mbuf[i+3] == 0x24
        ){
            *pos = i+4;
            return true;
        }
    }
    return false;
}

bool char_is_txt(uint8_t ch){
    return (ch > 31 && ch < 128);
}


void tap_api_read_header(void){
    bool found = false;
    UINT br = 0;
    FRESULT fr;
    tap_header_t header;

    uint32_t pos = tap_drive.counter;
    uint32_t hpos;
    api_zxstack();
    if(tap_drive.type == EMPTY)
        return api_return_errno(API_ENODEV);

    if(pos >= tap_drive.size - sizeof(tap_header_t))
        return api_return_errno(API_ENOENT);

    do{
        switch(tap_drive.type){
            case LFS:
                br = lfs_file_read(&lfs_volume, tap_drive.lfs_file, mbuf, MBUF_SIZE);
                break;
            case FAT:
                fr = f_read(tap_drive.fat_file, mbuf, MBUF_SIZE, &br);
                if(fr != FR_OK)
                    return api_return_errno(API_EFATFS(fr));
                break;
            default:
                return api_return_errno(API_ENODEV);
                break;
        }
        found = tap_seek_next_header(0, br, &hpos);
        pos += found ? hpos : MBUF_SIZE-4;    //Set to found pos or next block (minus one synch)
        tap_seek(pos);
    }while((br == MBUF_SIZE) && !found);
    
    if(!found)
        return api_return_errno(API_ENOENT);
    //Format return if found
    switch(tap_drive.type){
        case LFS:
            br = lfs_file_read(&lfs_volume, tap_drive.lfs_file, &header, sizeof(tap_header_t));
            break;
        case FAT:
            fr = f_read(tap_drive.fat_file, &header, sizeof(tap_header_t), &br);
            break;
        default:
            break;
    }
    header.filename[15] = '\0'; //safe guard
    //Replace non-text characters in name with '?'
    for(uint32_t i=0; i<strlen((char*)(header.filename)); i++){
        if(!char_is_txt(header.filename[i])){
            header.filename[i] = '?';
        }
    }
    api_set_axsreg((int32_t)(pos-4));  //Point back to synch bytes
    xstack_ptr = XSTACK_SIZE - sizeof(tap_header_t);
    memcpy((void*)&xstack[xstack_ptr], &header, sizeof(tap_header_t));
    api_sync_xstack();
    api_return_released();
}