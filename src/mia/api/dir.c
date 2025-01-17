/*
 * Copyright (c) 2023 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "api/api.h"
#include "api/dir.h"
#include "sys/lfs.h"
#include "fatfs/ff.h"
#include "usb/usb.h"
#include "tusb.h"
#include <string.h>

/* Kernel events
 */

void dir_task(void);
void dir_stop(void);

/* The API implementation for directory support.
int closedir(DIR* dirp) 	
Closes the directory stream referred to by dirp. Upon return, dirp may no longer point to an accessible object of the type DIR. If a file descriptor is used to implement type DIR, that file descriptor will be closed. Upon successful completion, closedir() returns 0. Otherwise, -1 is returned and errno is set to indicate the error.
errno Errors: EBADF means dirp does not refer to an open directory stream, EINTR means the function was interrupted by a signal.

DIR* opendir(const char* dirname) 	
Opens a directory stream corresponding to the directory named by dirname. The directory stream is positioned at the first entry. If the type DIR is implemented using a file descriptor, applications will only be able to open up to a total of OPEN_MAX files and directories. Upon successful completion, opendir() returns a pointer to an object of type DIR. Otherwise, a null pointer is returned and errno is set to indicate the error.
errno Errors: EACCES means the search permission is denied for the component of the path prefix of dirname or read permission is denied for dirname. ELOOP means too many symbolic links were encountered in resolving path. ENAMETOOLONG means the length of the dirname argument exceeds PATH_MAX, or a pathname component is longer than NAME_MAX. ENOENT means a component of dirname does not name an existing directory or dirname is an empty string. ENOTDIR means a component of dirname is not a directory. EMFILE means OPEN_MAX file descriptors are currently open in the calling process. ENAMETOOLONG means the pathname resolution of a symbolic link produced an intermediate result whose length exceeds PATH_MAX. ENFILE means there are too many files are currently open in the system.

struct dirent* readdir(DIR* dirp) 	
Returns a pointer to a structure representing the directory entry at the current position in the directory stream specified by the argument dirp, and positions the directory stream at the next entry. It returns a null pointer upon reaching the end of the directory stream. If entries for dot or dot-dot exist, one entry will be returned for dot and one entry will be returned for dot-dot; otherwise they will not be returned. When an error is encountered, a null pointer is returned and errno is set to indicate the error. When the end of the directory is encountered, a null pointer is returned and errno is not changed.
The memory location pointed to by the return value is managed by the library and may change on subsequent calls to readdir. It should not be free'd by the user.
errno Errors: EOVERFLOW means one of the values in the structure to be returned cannot be represented correctly. EBADF means dirp does not refer to an open directory stream. ENOENT means the current position of the directory stream is invalid.

int readdir_r(DIR* dirp, struct dirent* entry, struct dirent** result) 	
Initialises entry to represent the directory entry at the current position in dirp, store a pointer to this structure at the location referenced by result, and positions the directory stream at the next entry. The storage pointed to by entry will be large enough for a dirent with an array of char d_name member containing at least NAME_MAX plus one elements. On successful return, the pointer returned at *result will have the same value as the argument entry. Upon reaching the end of the directory stream, this pointer will have the value NULL.
errno Errors: EBADF means dirp does not refer to an open directory stream. 
 */

#define DIR_LFS_MAX 4
#define DIR_FAT_MAX 4
//FD returned is offset depending on type
#define FD_OFFS_DEV 0
#define FD_OFFS_LFS 32
#define FD_OFFS_FAT 64
int dir_dev = -1;
lfs_dir_t dir_lfs[DIR_LFS_MAX] = {0};
bool dir_lfs_open[DIR_LFS_MAX] = {0};
DIR dir_fat[DIR_FAT_MAX] = {0};

#define DIR_FN_LEN 64
struct dir_dirent {
    int16_t d_fd;
    uint8_t d_name[DIR_FN_LEN];
    uint8_t d_attrib;
    uint8_t reserved;
    uint32_t d_size;
};
static_assert(sizeof(struct dir_dirent)==(2+DIR_FN_LEN+1+1+4),"struct dir_dirent size wrong");

void dir_api_opendir(void){
    uint8_t *path = &xstack[xstack_ptr];
    api_zxstack();
    int fd;
    switch(path[0]){
    case 0x00:  //Internal and USB device list. Only one iterator 
        if(dir_dev >= 0)
            api_return_errno(API_EMFILE);
        dir_dev = 0;
        return api_return_ax(0 + FD_OFFS_DEV);
        break;
    case '0':   //LFS internal storage
        fd = 0;
        for (; fd < DIR_LFS_MAX; fd++)
            if (!dir_lfs_open[fd])
                break;
        if (fd == DIR_LFS_MAX)
            return api_return_errno(API_EMFILE);
        {
            lfs_dir_t *dp = &dir_lfs[fd];
            int lfs_result = lfs_dir_open(&lfs_volume, dp, (const char*)&path[2]);
            if(lfs_result < 0){
                return api_return_errno(API_ELFSFS(lfs_result));
            }
            lfs_dir_seek(&lfs_volume,dp,2);     //Skip . and .. virtual directory entries
        }
        dir_lfs_open[fd] = true;
        return api_return_ax(fd + FD_OFFS_LFS);
        break;
    default:    //FAT USB storage
        fd = 0;
        for (; fd < DIR_FAT_MAX; fd++)
            if (!dir_fat[fd].obj.fs)
                break;
        if (fd == DIR_FAT_MAX)
            return api_return_errno(API_EMFILE);
        {
            DIR *dp = &dir_fat[fd];
            FRESULT fresult = f_opendir(dp, (TCHAR *)path);
            if (fresult != FR_OK)
                return api_return_errno(API_EFATFS(fresult));
        }
        return api_return_ax(fd + FD_OFFS_FAT);
    }
}

void dir_api_closedir(void){
    int fd = API_A;
    api_zxstack();
    if (fd < 0)
        return api_return_errno(API_EINVAL);
    if(fd >= FD_OFFS_FAT){
        DIR *dp = &dir_fat[fd - FD_OFFS_FAT];
        FRESULT fresult = f_closedir(dp);
        if (fresult != FR_OK)
            return api_return_errno(API_EFATFS(fresult));
    }else if(fd >= FD_OFFS_LFS){
        lfs_dir_t *dp = &dir_lfs[fd - FD_OFFS_LFS];
        int lfs_result = lfs_dir_close(&lfs_volume, dp);
        if(lfs_result < 0)
            return api_return_errno(API_ELFSFS(lfs_result));
        dir_lfs_open[fd - FD_OFFS_LFS] = false;
    }else{
        dir_dev = -1;
    }
    return api_return_ax(0);
}

void dir_api_readdir(void){
    int fd = API_A;
    struct dir_dirent dirent;
    if (fd < 0)
        return api_return_errno(API_EINVAL);
    if(fd >= FD_OFFS_FAT){
        DIR *dp = &dir_fat[fd - FD_OFFS_FAT];
        FILINFO fno;
        FRESULT fresult;
        fresult = f_readdir(dp,&fno);
        if (fresult != FR_OK)
            return api_return_errno(API_EFATFS(fresult));
        //printf("[%s]",fno.fname);
        dirent.d_fd = fd;
        dirent.d_attrib = fno.fattrib;
        if(strlen(fno.fname) >= DIR_FN_LEN){
            if(fno.altname[0]){
                strcpy((char*)dirent.d_name, fno.altname);
            }else{  //No altname - all options bad?
                snprintf((char*)dirent.d_name, DIR_FN_LEN, "!Long name %s", fno.fname);
            }
        }else{
            strncpy((char*)dirent.d_name, fno.fname, DIR_FN_LEN);
        } 
        dirent.d_size = (uint32_t)fno.fsize;
    }else if(fd >= FD_OFFS_LFS){
        lfs_dir_t *dp = &dir_lfs[fd - FD_OFFS_LFS];
        struct lfs_info info;
        int lfs_result;
        lfs_result = lfs_dir_read(&lfs_volume, dp, &info);
        if(lfs_result < 0)
            return api_return_errno(API_ELFSFS(lfs_result));
        dirent.d_fd = fd;
        dirent.d_attrib = (info.type == LFS_TYPE_DIR ? AM_DIR : 0x00 ); //Using FAT attributes TODO Read-only flag
        if(strlen(info.name) >= DIR_FN_LEN){
            snprintf((char*)dirent.d_name, DIR_FN_LEN, "!Long name %s", info.name);
        }else{
            strncpy((char*)dirent.d_name, info.name, DIR_FN_LEN);
        } 
        dirent.d_size = info.size;
    }else{ //Device list
        dirent.d_fd = 0;
        dirent.d_attrib = AM_SYS;   //Using system file attribute for devices
        char* dev_message;
        if(dir_dev == 0){
            strcpy((char*)dirent.d_name, "0: Internal storage [15MB]");
        }else{
            while(!tuh_mounted(dir_dev) && dir_dev < CFG_TUH_DEVICE_MAX)
                dir_dev++;
            if(dir_dev >= CFG_TUH_DEVICE_MAX){
                dirent.d_name[0] = '\0';
            }else{
                dev_message = usb_get_status(dir_dev);
                strncpy((char*)dirent.d_name, dev_message, 64);
            }
        }
        dir_dev++;
    }
    api_set_ax(0);
    xstack_ptr = XSTACK_SIZE - sizeof(dirent);
    memcpy((void*)&xstack[xstack_ptr], &dirent, sizeof(dirent));
    api_sync_xstack();
    api_return_released();
}

void dir_api_mkdir(void){
    uint8_t *path = &xstack[xstack_ptr];
    api_zxstack();
    switch(path[0]){
    case 0x00:  //Internal and USB device list. No mkdir possible 
        return api_return_errno(API_ENODEV);
        break;
    case '0':   //LFS internal storage
        {
            int lfs_result = lfs_mkdir(&lfs_volume, (const char*)&path[2]);
            if(lfs_result < 0){
                return api_return_errno(API_ELFSFS(lfs_result));
            }
        }

        break;
    default:    //FAT USB 
        {
            FRESULT fresult = f_mkdir((TCHAR *)path);
            if (fresult != FR_OK)
                return api_return_errno(API_EFATFS(fresult));
        }
    }
    return api_return_ax(0);
}
