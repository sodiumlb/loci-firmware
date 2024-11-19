/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** 
 * ROM file tracking for "mounting" through the API
*/

#include "oric/rom.h"
#include "main.h"
#include "sys/mem.h"

enum ROM_TYPE { EMPTY = 0, LFS, FAT };
typedef struct _rom_drive_t {
    enum ROM_TYPE type;
    lfs_file_t *lfs_file;
    FIL *fat_file;
    uint32_t size;
} rom_drive_t;

rom_drive_t rom_drive = {0};

bool rom_mount_lfs(lfs_file_t *lfs_file){
    uint32_t file_size = lfs_file_size(&lfs_volume, lfs_file);
    if(file_size > 0x4000)      //Max 16kB ROM files
        return false;
    rom_drive.type = LFS;
    rom_drive.lfs_file = lfs_file;
    rom_drive.size = file_size;
    return true;
}
bool rom_mount_fat(FIL *fat_file){
    uint32_t file_size = f_size(fat_file);
    if(file_size > 0x4000)      //Max 16kB ROM files
        return false;
    f_rewind(fat_file);
    rom_drive.type = FAT;
    rom_drive.fat_file = fat_file;
    rom_drive.size = file_size;
    return true;
}
void rom_umount(void){
    switch(rom_drive.type){
        case LFS:
            lfs_free_file_config(rom_drive.lfs_file);
            lfs_file_close(&lfs_volume, rom_drive.lfs_file);
            break;
        case FAT:
            f_close(rom_drive.fat_file);
            break;
        default:
            break;
    }
    rom_drive.type = EMPTY;
}

bool rom_is_mounted(void){
    return rom_drive.type != EMPTY;
}

//TODO Secure read functions if read is short?
bool rom_load_mounted(void){
    switch(rom_drive.type){
        case LFS:
            lfs_file_rewind(&lfs_volume, rom_drive.lfs_file);
            lfs_file_read(&lfs_volume, rom_drive.lfs_file, (void*)&xram[0x10000-rom_drive.size], rom_drive.size);
            break;
        case FAT:
            f_rewind(rom_drive.fat_file);
            f_read(rom_drive.fat_file, (void*)&xram[0x10000-rom_drive.size], rom_drive.size, NULL);
            break;
        case EMPTY:
            return false;
    }
    return true;
}