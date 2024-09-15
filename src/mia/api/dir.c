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
    uint16_t d_size;
};


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
        dirent.d_size = fno.fsize;
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



/*
#define STD_FIL_MAX 16
FIL std_fil[STD_FIL_MAX];
#define STD_FIL_STDIN 0
#define STD_FIL_STDOUT 1
#define STD_FIL_STDERR 2
#define STD_FIL_OFFS 3
static_assert(STD_FIL_MAX + STD_FIL_OFFS < 128);

static int32_t std_xram_count = -1;
static int32_t std_in_count = -1;

static volatile size_t std_out_tail;
static volatile size_t std_out_head;
static volatile uint8_t std_out_buf[32];
#define STD_OUT_BUF(pos) std_out_buf[(pos) & 0x1F]

static inline bool std_out_buf_writable()
{
    return (((std_out_head + 1) & 0x1F) != (std_out_tail & 0x1F));
}

static inline void std_out_buf_write(char ch)
{
    STD_OUT_BUF(++std_out_head) = ch;
}

bool std_active(void)
{
    // Active until stdout is empty
    return (((std_out_head) & 0x1F) != (std_out_tail & 0x1F));
}

void std_task(void)
{
    // 6502 applications write to std_out_buf which we route
    // through the Pi Pico STDIO driver for CR/LR translation.
    if ((&STD_OUT_BUF(std_out_head) != &STD_OUT_BUF(std_out_tail)) &&
        (&COM_TX_BUF(com_tx_head + 1) != &COM_TX_BUF(com_tx_tail)) &&
        (&COM_TX_BUF(com_tx_head + 2) != &COM_TX_BUF(com_tx_tail)))
        putchar(STD_OUT_BUF(++std_out_tail));
}

void std_api_open(void)
{
    // These match CC65 which is closer to POSIX than FatFs.
    const unsigned char RDWR = 0x03;
    const unsigned char CREAT = 0x10;
    const unsigned char TRUNC = 0x20;
    const unsigned char APPEND = 0x40;
    const unsigned char EXCL = 0x80;

    uint8_t flags = API_A;
    uint8_t mode = flags & RDWR; // RDWR are same bits
    if (flags & CREAT)
    {
        if (flags & EXCL)
            mode |= FA_CREATE_NEW;
        else
        {
            if (flags & TRUNC)
                mode |= FA_CREATE_ALWAYS;
            else if (flags & APPEND)
                mode |= FA_OPEN_APPEND;
            else
                mode |= FA_OPEN_ALWAYS;
        }
    }
    uint8_t *path = &xstack[xstack_ptr];
    api_zxstack();
    int fd = 0;
    for (; fd < STD_FIL_MAX; fd++)
        if (!std_fil[fd].obj.fs)
            break;
    if (fd == STD_FIL_MAX)
        return api_return_errno(API_EMFILE);
    FIL *fp = &std_fil[fd];
    FRESULT fresult = f_open(fp, (TCHAR *)path, mode);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(fd + STD_FIL_OFFS);
}

void std_api_close(void)
{
    int fd = API_A;
    if (fd < STD_FIL_OFFS || fd >= STD_FIL_MAX + STD_FIL_OFFS)
        return api_return_errno(API_EINVAL);
    FIL *fp = &std_fil[fd - STD_FIL_OFFS];
    FRESULT fresult = f_close(fp);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(0);
}

void std_api_read_xstack(void)
{
    uint8_t *buf;
    uint16_t count;
    UINT br;
    if (std_in_count >= 0)
    {
        if (!cpu_stdin_ready())
            return;
        count = std_in_count;
        std_in_count = -1;
        buf = &xstack[XSTACK_SIZE - count];
        br = cpu_stdin_read(buf, count);
    }
    else
    {
        int16_t fd = API_A;
        if (!api_pop_uint16_end(&count) ||
            (fd && fd < STD_FIL_OFFS) ||
            fd >= STD_FIL_MAX + STD_FIL_OFFS ||
            count > 0x100)
            return api_return_errno(API_EINVAL);
        buf = &xstack[XSTACK_SIZE - count];
        if (!fd)
        {
            std_in_count = count;
            cpu_stdin_request();
            return;
        }
        FIL *fp = &std_fil[fd - STD_FIL_OFFS];
        FRESULT fresult = f_read(fp, buf, count, &br);
        if (fresult != FR_OK)
        {
            API_ERRNO = fresult;
            api_set_ax(-1);
            return;
        }
    }
    api_set_ax(br);
    xstack_ptr = XSTACK_SIZE;
    if (br == count)
        xstack_ptr -= count;
    else // relocate short read
        for (UINT i = br; i;)
            xstack[--xstack_ptr] = buf[--i];
    api_sync_xstack();
    api_return_released();
}

void std_api_read_xram(void)
{
    static uint16_t xram_addr;
    if (std_in_count >= 0)
    {
        if (!cpu_stdin_ready())
            return;
        std_xram_count = cpu_stdin_read(&xram[xram_addr], std_in_count);
        std_in_count = -1;
    }
    if (std_xram_count >= 0)
    {
        for (; std_xram_count
         //&& pix_ready()
         ; --std_xram_count, ++xram_addr)
         //   pix_send(PIX_DEVICE_XRAM, 0, xram[xram_addr], xram_addr);
        if (!std_xram_count)
        {
            std_xram_count = -1;
            api_return_released();
        }
        return;
    }
    uint8_t *buf;
    uint16_t count;
    int16_t fd = API_A;
    if (!api_pop_uint16(&count) ||
        !api_pop_uint16_end(&xram_addr) ||
        (fd && fd < STD_FIL_OFFS) ||
        fd >= STD_FIL_MAX + STD_FIL_OFFS)
        return api_return_errno(API_EINVAL);
    if (!fd)
    {
        cpu_stdin_request();
        std_in_count = std_xram_count = count;
        return;
    }
    buf = &xram[xram_addr];
    if (count > 0x7FFF)
        count = 0x7FFF;
    if (buf + count > xram + 0x10000)
        return api_return_errno(API_EINVAL);
    FIL *fp = &std_fil[fd - STD_FIL_OFFS];
    UINT br;
    FRESULT fresult = f_read(fp, buf, count, &br);
    if (fresult == FR_OK)
        api_set_ax(br);
    else
    {
        API_ERRNO = fresult;
        api_set_ax(-1);
    }
    std_xram_count = br;
}

// Non-blocking write
static void std_out_write(char *ptr)
{
    static void *std_out_ptr;
    if (ptr)
        std_out_ptr = ptr;
    for (; std_xram_count && std_out_buf_writable(); --std_xram_count)
        std_out_buf_write(*(uint8_t *)std_out_ptr++);
    if (!std_xram_count)
    {
        std_xram_count = -1;
        api_return_released();
    }
}

void std_api_write_xstack(void)
{
    if (std_xram_count >= 0)
        return std_out_write(NULL);
    uint8_t *buf;
    uint16_t count;
    int fd = API_A;
    if (fd == STD_FIL_STDIN || fd >= STD_FIL_MAX + STD_FIL_OFFS)
        return api_return_errno(API_EINVAL);
    count = XSTACK_SIZE - xstack_ptr;
    buf = &xstack[xstack_ptr];
    api_zxstack();
    if (fd < STD_FIL_OFFS)
    {
        api_set_ax(count);
        std_xram_count = count;
        std_out_write((char *)buf);
        return;
    }
    FIL *fp = &std_fil[fd - STD_FIL_OFFS];
    UINT bw;
    FRESULT fresult = f_write(fp, buf, count, &bw);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(bw);
}

void std_api_write_xram(void)
{
    if (std_xram_count >= 0)
        return std_out_write(NULL);
    uint8_t *buf;
    uint16_t xram_addr;
    uint16_t count;
    int fd = API_A;
    if (fd == STD_FIL_STDIN || fd >= STD_FIL_MAX + STD_FIL_OFFS)
        return api_return_errno(API_EINVAL);
    if (!api_pop_uint16(&count) ||
        !api_pop_uint16_end(&xram_addr))
        return api_return_errno(API_EINVAL);
    buf = &xram[xram_addr];
    if (buf + count > xram + 0x10000)
        return api_return_errno(API_EINVAL);
    if (count > 0x7FFF)
        count = 0x7FFF;
    if (fd < STD_FIL_OFFS)
    {
        api_set_ax(count);
        std_xram_count = count;
        std_out_write((char *)buf);
        return;
    }
    FIL *fp = &std_fil[fd - STD_FIL_OFFS];
    UINT bw;
    FRESULT fresult = f_write(fp, buf, count, &bw);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(bw);
}

void std_api_lseek(void)
{
    int8_t whence;
    int32_t ofs;
    int fd = API_A;
    if (fd < STD_FIL_OFFS ||
        fd >= STD_FIL_MAX + STD_FIL_OFFS ||
        !api_pop_int8(&whence) ||
        !api_pop_int32_end(&ofs))
        return api_return_errno(API_EINVAL);
    FIL *fp = &std_fil[fd - STD_FIL_OFFS];
    switch (whence) // CC65
    {
    case 0: // SEEK_CUR
        ofs += f_tell(fp);
        break;
    case 1: // SEEK_END
        ofs += f_size(fp);
        break;
    case 2: // SEEK_SET
        break;
    default:
        return api_return_errno(API_EINVAL);
    }
    FRESULT fresult = f_lseek(fp, ofs);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    FSIZE_t pos = f_tell(fp);
    // Beyond 2GB is darkness.
    if (pos > 0x7FFFFFFF)
        pos = 0x7FFFFFFF;
    return api_return_axsreg(pos);
}

void std_api_unlink(void)
{
    uint8_t *path = &xstack[xstack_ptr];
    api_zxstack();
    FRESULT fresult = f_unlink((TCHAR *)path);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(0);
}

void std_api_rename(void)
{
    uint8_t *oldname, *newname;
    oldname = newname = &xstack[xstack_ptr];
    api_zxstack();
    while (*oldname)
        oldname++;
    if (oldname == &xstack[XSTACK_SIZE])
        return api_return_errno(API_EINVAL);
    oldname++;
    FRESULT fresult = f_rename((TCHAR *)oldname, (TCHAR *)newname);
    if (fresult != FR_OK)
        return api_return_errno(API_EFATFS(fresult));
    return api_return_ax(0);
}

void std_stop(void)
{
    std_xram_count = -1;
    std_in_count = -1;
    for (int i = 0; i < STD_FIL_MAX; i++)
        if (std_fil[i].obj.fs)
            f_close(&std_fil[i]);
}
*/