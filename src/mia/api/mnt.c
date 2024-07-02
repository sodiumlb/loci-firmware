/*
 * Copyright (c) 2023 Rumbledethumps
 * Copyrighy (c) 2024 Sodiumlightbaby
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "api/api.h"
#include "api/mnt.h"
#include "oric/dsk.h"
#include "oric/tap.h"
#include "fatfs/ff.h"
#include "sys/lfs.h"
#include <string.h>

//4 drives + 1 tape mountable
#define MNT_FD_MAX 5
FIL mnt_fd_fat[MNT_FD_MAX];
lfs_file_t mnt_fd_lfs[MNT_FD_MAX];

//TODO Very costly buffer. Relocate?
uint8_t mnt_lfs_buffer[MNT_FD_MAX][FLASH_PAGE_SIZE];
struct lfs_file_config mnt_lfs_configs[MNT_FD_MAX] = {
    { .buffer = mnt_lfs_buffer[0]},
    { .buffer = mnt_lfs_buffer[1]},
    { .buffer = mnt_lfs_buffer[2]},
    { .buffer = mnt_lfs_buffer[3]},
    { .buffer = mnt_lfs_buffer[4]},
};

/* Kernel events
 */

void mnt_task(void);
void mnt_stop(void);

void mnt_api_mount(void){
    uint8_t drive = API_A;
    char *path = (char*)&xstack[xstack_ptr];
    api_zxstack();
    if(drive == 4){ //Tape
        tap_umount();
    }else{
        dsk_umount(drive);
    }
    if(path[0]=='0'){   //LFS mount for path starting"0:"
        //Todo attribute check RO
        lfs_file_opencfg(&lfs_volume, &mnt_fd_lfs[drive], &path[2], LFS_O_RDWR, & mnt_lfs_configs[drive]);
        if(drive == 4){
            tap_mount_lfs(&mnt_fd_lfs[4]);
        }else{
            dsk_mount_lfs(drive, &mnt_fd_lfs[drive]);
        }
    }else{              //FAT mount
        FRESULT fresult = f_open(&mnt_fd_fat[drive], path, FA_READ | FA_WRITE);
        if (fresult != FR_OK)
            return api_return_errno(API_EFATFS(fresult));
        if(drive == 4){
            tap_mount_fat(&mnt_fd_fat[4]);
        }else{
            dsk_mount_fat(drive, &mnt_fd_fat[drive]);
        }
    }
    printf("{MNT ok %s}",path);
    return api_return_ax(0);
}

void mnt_api_umount(void){
    uint8_t drive = API_A;
    if(drive == 4){ //Tape
        tap_umount();
    }else{
        dsk_umount(drive);
    }
    return api_return_ax(0);
}
