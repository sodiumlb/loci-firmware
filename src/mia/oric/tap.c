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

//Track buffer
//volatile uint8_t tap_buf[1024];

enum TAP_TYPE { EMPTY = 0, LFS, FAT };
typedef struct _tap_drive_t {
    enum TAP_TYPE type;
    lfs_file_t *lfs_file;
    FIL *fat_file;
    uint32_t counter;
    uint8_t filename[16];
} tap_drive_t;

tap_drive_t tap_drive = {0};

#define TAP_STAT_NOT_READY (0b10000000)
#define TAP_STAT_WPROT     (0b01000000)
#define TAP_STAT_BUSY      (0b00000001)

#define TAP_CMD_PLAY        (0x01)
#define TAP_CMD_REC         (0x02)
#define TAP_CMD_REW         (0x03)

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
    tap_set_status(TAP_STAT_NOT_READY,false);
    return true;
}


void tap_umount(void){
    //TODO Cleanup?
    tap_set_status(TAP_STAT_NOT_READY,true);
    tap_drive.type = EMPTY;
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
}

enum TAP_STATE { TAP_IDLE, TAP_READ, TAP_WRITE, TAP_CLEANUP } tap_state = TAP_IDLE;

bool tap_read_byte(void){
    bool fs_ret = false;
    UINT br;
    FRESULT fr;
    if(tap_drive.counter < 8){
        IOREGS(TAP_IO_DATA) = 0x16;
        tap_drive.counter++;
    }else{
        switch(tap_drive.type){
            case LFS:
                if(lfs_file_read(&lfs_volume, tap_drive.lfs_file, (void *) &IOREGS(TAP_IO_DATA), 1) == 1){
                    tap_drive.counter++;
                    fs_ret = true;
                };
                break;
            case FAT:
                fr = f_read(tap_drive.fat_file, (void *) &IOREGS(TAP_IO_DATA), 1, &br);
                if(fr == FR_OK && br == 1){
                    tap_drive.counter++;
                    fs_ret = true;
                }
                break;
            default:
                break;
                //TODO error
        }
    }
    //Return zero when nothing is read 
    if(!fs_ret){
        IOREGS(TAP_IO_DATA) = 0x00;
    }

    return true;
}

void tap_init(void){
    IOREGS(TAP_IO_STAT) = TAP_STAT_NOT_READY; 
    IOREGS(TAP_IO_CMD)  = 0x00; 
    IOREGS(TAP_IO_DATA) = 0x00;
    tap_state = TAP_IDLE;
}

void tap_task(void){
    switch(tap_state){
        case TAP_IDLE:
            switch(IOREGS(TAP_IO_CMD)){
                case TAP_CMD_PLAY:
                    tap_state = TAP_READ;
                    tap_set_status(TAP_STAT_BUSY,true);
                    break;
                case TAP_CMD_REC:
                //TODO setup initial write file
                    tap_set_status(TAP_STAT_BUSY,true);
                    tap_state = TAP_WRITE;
                    break;
                case TAP_CMD_REW:
                    tap_rewind();
                    tap_state = TAP_CLEANUP;
                   break;
                 default:
                    break;
            }
            break;
        case TAP_READ:
            if(IOREGS(TAP_IO_STAT) & TAP_STAT_BUSY){
                tap_read_byte();
                tap_set_status(TAP_STAT_BUSY,false);
                tap_state = TAP_CLEANUP;
            }
            break;
        case TAP_WRITE:
            break;
        case TAP_CLEANUP:
            IOREGS(TAP_IO_CMD) = 0x00;
            tap_state = TAP_IDLE;
            break;
        default:
            tap_state = TAP_IDLE;
    }
}

void tap_act(void){
    //led_toggle();
    //tap_set_status(TAP_STAT_BUSY,true); //Holds Oric waiting and triggers tap_task() execution
}

