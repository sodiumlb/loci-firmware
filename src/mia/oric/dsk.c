/*
 * Copyright (c) 2023 Rumbledethumps
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** 
 * Emulating Oric MFM_DISK DSK floppy image format on a WD 1793 FDC
 * Limitaitons:
 * - Timing not implemented
 * - Assumes READY is tied high
*/

#include "oric/dsk.h"
#include "main.h"
#include "sys/ext.h"
#include "sys/led.h"
#include "sys/mem.h"
#include "sys/mia.h"
#include "sys/ssd.h"
#include "pico/multicore.h"


//Track buffer
volatile uint8_t dsk_buf[6400];

//Oric MFM_DISK signature
uint8_t dsk_signature[8] = "MFM_DISK";

enum DSK_TYPE { EMPTY = 0, LFS, FAT };
typedef struct _dsk_drive_t {
    enum DSK_TYPE type;
    lfs_file_t *lfs_file;
    FIL *fat_file;
    uint32_t sides;
    uint32_t tracks;
    uint32_t geometry;
} dsk_drive_t;

dsk_drive_t dsk_drives[4] = {{0}};

volatile struct {
    dsk_drive_t *drive;
    uint32_t side;
    uint32_t track;
    uint32_t sector;
    uint32_t pos;
    uint32_t data_start;
    uint32_t data_len;
    uint8_t drive_num;
    //uint8_t *data_ptr;
    bool step_dir_out;
    bool index_irq;
    bool sector_dm;
    bool buf_update_needed;
    bool track_writeback;
} dsk_active;

uint8_t dsk_cmd(uint8_t raw_cmd);
bool dsk_flush_track(void);

//Assumes file has been opened first
bool dsk_mount_lfs(uint8_t drive, lfs_file_t *lfs_file){
    uint8_t header_buf[20];
    if(drive > 3) 
        return false;
    lfs_file_rewind(&lfs_volume, lfs_file);
    if(lfs_file_read(&lfs_volume, lfs_file, (void *)header_buf, 20) == 20){
        for(uint8_t i=0; i<8; i++){
            if(header_buf[i] != dsk_signature[i]){
                return false;
            }
        }
        dsk_drives[drive].type = LFS;
        dsk_drives[drive].lfs_file = lfs_file;
        dsk_drives[drive].sides = *(uint32_t *)(header_buf+8);
        dsk_drives[drive].tracks = *(uint32_t *)(header_buf+12);
        dsk_drives[drive].geometry = *(uint32_t *)(header_buf+16);

        dsk_active.buf_update_needed = true;
        return true;
    }else{
        return false;
    }
}

bool dsk_mount_fat(uint8_t drive, FIL *fat_file){
    uint8_t header_buf[20];
    UINT br;
    FRESULT fr;
    if(drive > 3) 
        return false;
    f_rewind(fat_file);
    fr = f_read(fat_file, (void *)header_buf, 20, &br);
    if(fr == FR_OK && br == 20){
        for(uint8_t i=0; i<8; i++){
            if(header_buf[i] != dsk_signature[i]){
                return false;
            }
        }
        dsk_drives[drive].type = FAT;
        dsk_drives[drive].fat_file = fat_file;
        dsk_drives[drive].sides = *(uint32_t *)(header_buf+8);
        dsk_drives[drive].tracks = *(uint32_t *)(header_buf+12);
        dsk_drives[drive].geometry = *(uint32_t *)(header_buf+16);
        printf("dsk tr:%ld/%ld",dsk_drives[drive].tracks,dsk_drives[drive].geometry);
        dsk_active.buf_update_needed = true;
        return true;
    }else{
        return false;
    }
}


void dsk_umount(uint8_t drive){
    if(drive > 3) {
        return;
    }
    if(dsk_active.drive_num == drive && dsk_active.track_writeback){
        dsk_flush_track();
    }
    if(dsk_drives[drive].type == FAT){
        f_close(dsk_drives[drive].fat_file);
    }else if(dsk_drives[drive].type == LFS){
        lfs_free_file_config(dsk_drives[drive].lfs_file);
        lfs_file_close(&lfs_volume, dsk_drives[drive].lfs_file);
    }
    dsk_drives[drive].type = EMPTY;
}

static inline void dsk_set_active_drive(uint8_t drive){
    if(drive != dsk_active.drive_num){
        dsk_active.drive = &dsk_drives[drive];
        dsk_active.drive_num = drive;
        dsk_active.buf_update_needed = true;
    }
}

bool dsk_set_active_track(uint32_t track){
    //printf("<sat>");
    if(dsk_active.drive->type == EMPTY || track > (dsk_active.drive->tracks-1)){
        //printf("##Track error %d##",track);
        return false;
    }
    uint32_t track_off;
    if(dsk_active.drive->geometry == 2){
        track_off = 256 + (6400 * (track + dsk_active.side));
    }else{
        track_off = 256 + (6400 * ((dsk_active.side * dsk_active.drive->tracks) + track));
    }
    //TODO Check returns
    UINT br;
    FRESULT fr;
    switch(dsk_active.drive->type){
        case EMPTY:
            return false;
        case LFS:
            lfs_file_seek(&lfs_volume, dsk_active.drive->lfs_file, (lfs_soff_t)track_off, 0);
            lfs_file_read(&lfs_volume, dsk_active.drive->lfs_file, (void *)dsk_buf, 6400);
            break;
        case FAT:
            f_lseek(dsk_active.drive->fat_file, (FSIZE_t)track_off);
            fr = f_read(dsk_active.drive->fat_file, (void *)dsk_buf, 6400, &br);
            break;
    }
    dsk_active.track = track;
    dsk_active.track_writeback = false;
    char tr_status[10];
    sprintf(tr_status,"[%c:%ld:%02ld]", 'A' + dsk_active.drive_num, dsk_active.side, track);
    ssd_write_text(13,0,true,tr_status);
    return true;
}

bool dsk_flush_track(void){
    uint32_t track_off;
    if(dsk_active.drive->geometry == 2){
        track_off = 256 + (6400 * (dsk_active.track + dsk_active.side));
    }else{
        track_off = 256 + (6400 * ((dsk_active.side * dsk_active.drive->tracks) + dsk_active.track));
    }
    //TODO Check returns
    UINT br;
    FRESULT fr;
    switch(dsk_active.drive->type){
        case EMPTY:
            return false;
        case LFS:
            lfs_file_seek(&lfs_volume, dsk_active.drive->lfs_file, (lfs_soff_t)track_off, 0);
            lfs_file_write(&lfs_volume, dsk_active.drive->lfs_file, (void *)dsk_buf, 6400);
            lfs_file_sync(&lfs_volume, dsk_active.drive->lfs_file);
            break;
        case FAT:
            f_lseek(dsk_active.drive->fat_file, (FSIZE_t)track_off);
            fr = f_write(dsk_active.drive->fat_file, (void *)dsk_buf, 6400, &br);
            f_sync(dsk_active.drive->fat_file);
            break;
    }
    dsk_active.track_writeback = false;
    printf("##WTRACK [%c:%ld:%02ld]##\n",'A' + dsk_active.drive_num, dsk_active.side, dsk_active.track);
    return true;
}

static inline void dsk_set_active_side(uint8_t side){
    if(dsk_active.side != side){
        dsk_active.side = side;
        dsk_active.buf_update_needed = true;
    }
}


//Type I flags
#define DSK_FLAG_IS_SEEK   (0b00010000)
#define DSK_FLAG_UPD_TRACK (0b00010000)
#define DSK_FLAG_LOAD_HEAD (0b00001000)
#define DSK_FLAG_VERIFY    (0b00000100)
//Type II & III flags
#define DSK_FLAG_IS_WRITE  (0b00010000)
#define DSK_FLAG_MULTI     (0b00010000)
#define DSK_FLAG_SIDE      (0b00001000)
#define DSK_FLAG_DELAY     (0b00000100)
#define DSK_FLAG_SIDE_CMP  (0b00000010)
#define DSK_FLAG_DMARK     (0b00000001)
//Type IV flags
#define DSK_FLAG_IS_FINT   (0b00010000)
#define DSK_FLAG_TOREADY   (0b00000001)
#define DSK_FLAG_TONREADY  (0b00000010)
#define DSK_FLAG_INDEX     (0b00000100)
#define DSK_FLAG_IMMEDIATE (0b00001000)

#define DSK_STAT_NOT_READY (0b10000000)
#define DSK_STAT_WPROT     (0b01000000)
#define DSK_STAT_HLOADED   (0b00100000)
#define DSK_STAT_RECTYPE   (0b00100000)
#define DSK_STAT_WFAULT    (0b00100000)
#define DSK_STAT_SEEK_ERR  (0b00010000)
#define DSK_STAT_RNF       (0b00010000)
#define DSK_STAT_CRC_ERR   (0b00001000)
#define DSK_STAT_TRACK_00  (0b00000100)
#define DSK_STAT_LOST_DATA (0b00000100)
#define DSK_STAT_INDEX     (0b00000010)
#define DSK_STAT_DRQ       (0b00000010)
#define DSK_STAT_BUSY      (0b00000001)

#define DSK_REG_IRQ_BIT    (0b10000000)

//Status and CMD register are overlapped and can not be directy mapped
volatile uint8_t        dsk_reg_status, 
                        dsk_reg_cmd,
                        dsk_reg_irq;
volatile uint8_t        dsk_reg_ctrl;
volatile uint8_t        dsk_next_status;
#define dsk_reg_track  IOREGS(DSK_IO_TRACK)
#define dsk_reg_sector IOREGS(DSK_IO_SECT)
#define dsk_reg_data   IOREGS(DSK_IO_DATA)
#define dsk_reg_drq    IOREGS(DSK_IO_DRQ)
/*
extern volatile uint8_t dsk_reg_track,  
                        dsk_reg_sector, 
                        dsk_reg_data,
                        dsk_req_irq;
*/

volatile bool dsk_irq_enable;
void dsk_set_status(uint8_t bit, bool val){
    if(val){
        dsk_reg_status |= bit;
    }else{
        dsk_reg_status &= ~bit;
    }
}

typedef enum _DSK_CMD {  SEEK = 0,        
                STEP,
                STEP_IN,    
                STEP_OUT,   
                RSEC,
                WSEC,
                MISC,        
                TRACK } DSK_CMD;

#define DSK_IS_CMD(cid) ((DSK_CMD)(dsk_reg_cmd>>5) == cid)

/*volatile enum DSK_STATE { 
    DSK_IDLE, 
    DSK_SEEK, 
    DSK_READ_PREP, DSK_READ, 
    DSK_WRITE_PREP, DSK_WRITE, 
    DSK_READ_ADDR,
    DSK_TOGGLE_IRQ,
    DSK_CLEANUP 
}*/

volatile enum DSK_STATE dsk_state = DSK_IDLE;
//volatile enum DSK_STATE dsk_next_state = DSK_IDLE;

void dsk_init(void){
    dsk_state = DSK_IDLE;

    dsk_active.drive_num = 0;
    dsk_active.drive = &dsk_drives[0];

    dsk_active.side = 0;
    dsk_active.track = 0;
    dsk_active.sector = 0x1;
    dsk_active.pos = 0;
    dsk_active.data_len = 0;
    dsk_active.step_dir_out = false;
    dsk_active.index_irq = false;
    dsk_active.sector_dm = false; 
    dsk_active.buf_update_needed = false;
    dsk_active.track_writeback = false;

    dsk_reg_status = 0x00;
    dsk_reg_cmd = 0x03;
    dsk_reg_track = 0x00;
    dsk_reg_sector = 0x00;
    dsk_reg_data = 0x00;
    dsk_reg_irq = 0x80;     //Active low IRQ
    dsk_reg_drq = 0x80;     //Active low DRQ

    dsk_irq_enable = false;
    dsk_next_status = dsk_reg_status;
    
}
//volatile uint32_t dsk_next_track;

uint32_t dsk_seek_next_idam(uint32_t start){
    for(uint32_t i=start; i<6400-4; i++){
        //ID Address Mark
        if( dsk_buf[i]   == 0xA1 &&
            dsk_buf[i+1] == 0xA1 &&
            dsk_buf[i+2] == 0xA1 &&
            dsk_buf[i+3] == 0xFE
        ){
            return(i+4);
        }
    }
    return 0;
}

bool dsk_set_active_sector(uint32_t sector){
    //printf("<sas>");
    for(uint32_t i=0; i<6400-16; i++){
        //ID Address Mark
        if( dsk_buf[i]   == 0xA1 &&
            dsk_buf[i+1] == 0xA1 &&
            dsk_buf[i+2] == 0xA1 &&
            dsk_buf[i+3] == 0xFE)
        {
            uint32_t sector_len = 0x0080 << dsk_buf[i+7];
            //printf("<%d:%d:%ld>",dsk_buf[i+4],dsk_buf[i+6],sector_len);
            //TODO Side compare
            if(dsk_buf[i+4] == dsk_reg_track &&
               dsk_buf[i+6] == sector)
            {
                dsk_active.sector = sector;
                dsk_active.data_len = sector_len;
                i = i + 8; //AM
                i = i + 2; //CRC
                i = i +22; //Gap
                i = i +12;
                //Data Address Mark
                if( dsk_buf[i]    == 0xA1 &&
                    dsk_buf[i+1]  == 0xA1 &&
                    dsk_buf[i+2]  == 0xA1 &&
                    (dsk_buf[i+3] == 0xFB || dsk_buf[i+3] == 0xF8)
                ){
                    if(DSK_IS_CMD(RSEC)){
                        dsk_active.sector_dm = (dsk_buf[i+3] == 0xF8); //Signal Delete mark
                    }
                    //dsk_active.data_ptr = (uint8_t*)(dsk_buf + i + 4);
                    dsk_active.data_start = i + 4;
                    return true;
                }
            }else{
                //Skip this sector
                i = i + 8; //AM
                i = i + 2; //CRC
                i = i +22; //Gap
                i = i +12;
                i = i + 4; //DAM
                i = i + sector_len; //Sector Data
            }
        }
    }
    return false;
}

volatile uint32_t dsk_byte_cnt;

void dsk_task(void){
    static uint8_t dsk_next_track = 0;
    static uint32_t dsk_rw_countdown;

    if(dsk_irq_enable && dsk_reg_irq == 0x00){     //Slow IO signal, just toggle it - fingers crossed
        ext_pulse(EXT_IRQ);
        //printf("[irq %d]",dsk_state);
    }

    switch(dsk_state){
        case DSK_IDLE:
        /*
            if(dsk_next_state != DSK_IDLE){
                dsk_state = dsk_next_state;
                dsk_next_state = DSK_IDLE;
                dsk_reg_status = dsk_next_status;
            }
        */  
            if(multicore_fifo_rvalid()){
                uint32_t raw_from_act = sio_hw->fifo_rd;
                bool is_cmd = !!(raw_from_act & 0x80000000);
                uint8_t cmd_from_act = (uint8_t)(raw_from_act & 0x000000FF);
                uint8_t ctrl_from_act = (uint8_t)((raw_from_act & 0x0000FF00) >> 8);
                if(is_cmd)
                    dsk_next_track = dsk_cmd(cmd_from_act);
                else
                    dsk_next_track = dsk_active.track;
                dsk_irq_enable = (ctrl_from_act & 0x01);
                //TODO: Read CLK, DDEN
                uint8_t side = (ctrl_from_act >> 4) & 0x01;
                uint8_t drive = (ctrl_from_act >> 5) & 0x03;
                if(is_cmd && dsk_active.track_writeback && (  drive != dsk_active.drive_num 
                                                    || side != dsk_active.side 
                                                    || dsk_next_track != dsk_active.track)){  
                    dsk_flush_track();
                }

                if(is_cmd){                         //Only trigger drive/side change together with command 
                    dsk_set_active_drive(drive);
                    dsk_set_active_side(side);
                }

                if(is_cmd && dsk_active.buf_update_needed){     //Only trigger track update when BUSY
                    dsk_set_active_track(dsk_active.track);
                    dsk_active.buf_update_needed = false;
                }
                /*
                dsk_state =         (from_act & 0x000000FF);
                dsk_next_track =    (from_act & 0x0000FF00) >>  8;
                dsk_reg_status =    (from_act & 0x00FF0000) >> 16;
                */
            }
            break;
        case DSK_SEEK:
            //printf("-seek-");
            led_set(true);
            if(dsk_next_track != dsk_active.track){     //Drive and side checks done on receive of CMD
                dsk_set_active_track(dsk_next_track);
            }
            dsk_set_status(DSK_STAT_TRACK_00, dsk_next_track==0);
            dsk_set_status(DSK_STAT_HLOADED, dsk_reg_cmd & DSK_FLAG_LOAD_HEAD);
            if(dsk_reg_cmd && DSK_FLAG_VERIFY){
                //TODO Verify only checking first IDAM, not all. Problem?
                uint32_t pos = dsk_seek_next_idam(0);
                dsk_set_status(DSK_STAT_HLOADED,true);
                    dsk_set_status(DSK_STAT_SEEK_ERR,dsk_buf[pos] != dsk_reg_track);
                dsk_active.pos = pos;
            }
            dsk_state = DSK_TOGGLE_IRQ;
            break;
        case DSK_READ_PREP:
            //printf("-rprep-");
            if(dsk_set_active_sector(dsk_reg_sector)){
                dsk_active.pos = dsk_active.data_start;
                dsk_byte_cnt = 0;
                dsk_reg_drq = 0x80;
                dsk_set_status(DSK_STAT_DRQ,false);
                dsk_state = DSK_READ;
                dsk_rw_countdown = dsk_active.data_len * 2;  //passes before we end the read
                //TODO Multi handling
            }else{
                printf("fail sec %d",dsk_reg_sector);
                dsk_set_status(DSK_STAT_RNF,true);
                dsk_state = DSK_TOGGLE_IRQ;
            }
            break;
        case DSK_READ:
            if(dsk_reg_drq == 0x80){
                if(dsk_active.pos < (dsk_active.data_start + dsk_active.data_len)){
                    sleep_us(4);
                    led_set(true);
                    //printf(".");
                    dsk_reg_data = dsk_buf[dsk_active.pos++];
                    dsk_reg_drq = 0x00;
                    dsk_set_status(DSK_STAT_DRQ,true);
                }else{
                    //printf("+RExit %ld %ld %ld+",dsk_byte_cnt, dsk_active.pos, dsk_active.data_start + dsk_active.data_len);
                    dsk_state = DSK_TOGGLE_IRQ;
                }
            }else{
                if(!(dsk_rw_countdown--)){
                    dsk_state = DSK_TOGGLE_IRQ;
                }
            }
            break;
        case DSK_WRITE_PREP:
            if(dsk_set_active_sector(dsk_reg_sector)){
                //printf("WS:%ld:%ld",dsk_active.track,dsk_active.sector);
                dsk_active.pos = dsk_active.data_start;
                dsk_reg_drq = 0x00;
                dsk_set_status(DSK_STAT_DRQ,true);
                dsk_state = DSK_WRITE;
                dsk_rw_countdown = dsk_active.data_len * 2;  //passes before we end the read
                //TODO Multi handling
            }else{
                dsk_set_status(DSK_STAT_RNF,true);
                dsk_state = DSK_TOGGLE_IRQ;
            }
            break;
        case DSK_WRITE:
            if(dsk_reg_drq == 0x80){
                if(dsk_active.pos < (dsk_active.data_start + dsk_active.data_len)){
                    led_set(true);
                    //printf(".");
                    //Write ops moved to dsk_rw() for multicore safety. 
                    //dsk_buf[dsk_active.pos++] = dsk_reg_data;
                    dsk_reg_drq = 0x00;
                    dsk_set_status(DSK_STAT_DRQ,true);
                    dsk_active.track_writeback = true;
                }else{
                    //printf("+RExit %ld %ld %ld+",dsk_byte_cnt, dsk_active.pos, dsk_active.data_start + dsk_active.data_len);
                    dsk_state = DSK_TOGGLE_IRQ;
                }
            }else{
                if(!(dsk_rw_countdown--)){
                    dsk_state = DSK_TOGGLE_IRQ;
                }
            }
            break;
        case DSK_READ_ADDR:
            do {
                dsk_active.data_start = dsk_seek_next_idam(dsk_active.data_start);
            }while(dsk_active.data_start == 0);

            dsk_active.data_len = 6;
            //dsk_active.data_ptr = (uint8_t *)(dsk_buf + dsk_active.pos);
            dsk_active.pos = dsk_active.data_start;
            dsk_state = DSK_READ;
            break;
        case DSK_TOGGLE_IRQ:
            //printf("-toggle-");
            dsk_reg_irq = 0x00;     //Assert register IRQ
            //IOREGS(DSK_IO_CTRL) = dsk_reg_irq;
            dsk_state = DSK_CLEANUP;
            break;
        case DSK_CLEANUP:
            led_set(false);
            dsk_set_status(DSK_STAT_BUSY, false);
            dsk_state = DSK_IDLE;
            break;
        default:
            dsk_state = DSK_IDLE;
    }
    IOREGS(DSK_IO_CMD) = dsk_reg_status;    //Not directly mapped due to combined use with CMD
    IOREGS(DSK_IO_CTRL) = dsk_reg_irq;      //Not directly mapped due to combined with CTRL
}

//Stop activity but keep mounts
void dsk_stop(void){
    dsk_irq_enable = false;
    led_set(false);
    dsk_set_status(DSK_STAT_BUSY, false);
    dsk_state = DSK_IDLE;
}

/*
//NB Runs on core1 action loop. 
void dsk_act(uint8_t raw_cmd){
    DSK_CMD cmd;
    IOREGS(DSK_IO_CMD) = DSK_STAT_BUSY; //Assume busy
    cmd = raw_cmd >> 5;
    uint8_t cmd_flags = raw_cmd & 0x1f;
    uint8_t act_next_status = 0x00;
    uint8_t act_next_track = 0x00;
    enum DSK_STATE act_next_state = DSK_IDLE;
    dsk_reg_cmd = raw_cmd;
    //Type I commands
    if((raw_cmd & 0x80) == 0x00){
        act_next_track = dsk_active.track;
        //dsk_set_status(DSK_STAT_BUSY,true);
        act_next_status = DSK_STAT_BUSY;
        switch(cmd){
            case SEEK: //and RESTORE
                if(cmd_flags && DSK_FLAG_IS_SEEK){
                    act_next_track = dsk_reg_data;
                    dsk_reg_track = act_next_track;
                }else{ //Restore cmd
                    act_next_track = 0;
                    dsk_reg_track = act_next_track;
                }
                break;
            case STEP:
                if(dsk_active.step_dir_out){
                    if(dsk_active.track > 0){
                        act_next_track = dsk_active.track - 1;
                    }
                }else{
                    act_next_track = dsk_active.track + 1;
                }
                break;
            case STEP_IN:
                act_next_track = dsk_active.track + 1;
                dsk_active.step_dir_out = false;
                break;
            case STEP_OUT:
                if(dsk_active.track > 0){
                    act_next_track = dsk_active.track + 1;
                }
                dsk_active.step_dir_out = false;
                break;
            default:
                //fail
                break;
        };
        act_next_state = DSK_SEEK;
        //TODO wp
        //TODO error check

        if(cmd_flags && DSK_FLAG_UPD_TRACK){
            dsk_reg_track = act_next_track;
        }

    //TYPE II-IV commands
    }else{
        switch(cmd){
            case RSEC:
                //dsk_set_status(DSK_STAT_BUSY,true);
                act_next_status = DSK_STAT_BUSY;
               //dsk_set_status(DSK_STAT_HLOADED,true);
                act_next_state = DSK_READ_PREP;
                break;
            case WSEC:
                //dsk_set_status(DSK_STAT_BUSY,true);
                act_next_status = DSK_STAT_BUSY;
                //dsk_set_status(DSK_STAT_HLOADED,true);
                act_next_state = DSK_WRITE_PREP;
                break;
            case MISC:
                //Force interrupt
                if(cmd_flags && DSK_FLAG_IS_FINT){
                    //DSK_FLAG_TOREADY
                    //DSK_FLAG_TONREADY
                    dsk_active.index_irq = !!(dsk_reg_cmd & DSK_FLAG_INDEX);
                    //DSK_FLAG_IMMEDIATE
                    if(cmd_flags & DSK_FLAG_IMMEDIATE){
                        //dsk_reg_irq = 0x00;
                        act_next_state = DSK_TOGGLE_IRQ;
                    }else{
                        act_next_state = DSK_CLEANUP;
                    }
                }else{
                //Read address
                    //dsk_set_status(DSK_STAT_BUSY,true);
                    act_next_status = DSK_STAT_BUSY;
                    //dsk_set_status(DSK_STAT_HLOADED,true);
                    act_next_state = DSK_READ_ADDR;
                }
                break;
            case TRACK:
                //dsk_set_status(DSK_STAT_BUSY,true);
                act_next_status = DSK_STAT_BUSY;
                //dsk_set_status(DSK_STAT_HLOADED,true);
                if(cmd_flags && DSK_FLAG_IS_WRITE){
                    dsk_active.data_len = 6400;
                    //dsk_active.data_ptr = (uint8_t *)(dsk_buf);
                    dsk_active.data_start = 0;
                    act_next_state = DSK_WRITE;
                }else{
                //Read track
                    dsk_active.data_len = 6400;
                    //dsk_active.data_ptr = (uint8_t *)(dsk_buf);
                    dsk_active.data_start = 0;
                    act_next_state = DSK_READ;
                }
                break;
            default:
                act_next_state = DSK_IDLE;
                //fail
                break;
        };

    }
    sio_hw->fifo_wr = act_next_status << 16 | act_next_track << 8 | act_next_state;
    IOREGS(DSK_IO_CMD) = act_next_status;    //Not directly mapped due to combined use with CMD
}
*/

//char dsk_state_id[][5] = { 
//    {"idle"}, {"seek"}, { "reap"}, { "read"}, { "wrip"}, { "writ"}, { "reaa"}, { "tirq"}, { "clean"}
//};
uint8_t dsk_cmd(uint8_t raw_cmd){
    DSK_CMD cmd;
    dsk_reg_status = 0x00;
    //IOREGS(DSK_IO_CMD) = DSK_STAT_BUSY; //Assume busy
    cmd = raw_cmd >> 5;
    uint8_t cmd_flags = raw_cmd & 0x1f;
    uint8_t dsk_next_track = dsk_active.track;
    dsk_reg_cmd = raw_cmd;
    //Type I commands
    if((raw_cmd & 0x80) == 0x00){
        dsk_next_track = dsk_active.track;
        dsk_set_status(DSK_STAT_BUSY,true);
        switch(cmd){
            case SEEK: //and RESTORE
                if(cmd_flags & DSK_FLAG_IS_SEEK){
                    dsk_next_track = dsk_reg_data;
                }else{ //Restore cmd
                    dsk_next_track = 0;
                    dsk_reg_track = dsk_next_track;
                }
                break;
            case STEP:
                if(dsk_active.step_dir_out){
                    if(dsk_active.track > 0){
                        dsk_next_track = dsk_active.track - 1;
                    }
                }else{
                    dsk_next_track = dsk_active.track + 1;
                }
                break;
            case STEP_IN:
                dsk_next_track = dsk_active.track + 1;
                dsk_active.step_dir_out = false;
                break;
            case STEP_OUT:
                if(dsk_active.track > 0){
                    dsk_next_track = dsk_active.track + 1;
                }
                dsk_active.step_dir_out = false;
                break;
            default:
                //fail
                break;
        };
        dsk_state = DSK_SEEK;
        //TODO wp
        //TODO error check

        //Don't seek beyond max tracks
        if(dsk_next_track >= dsk_active.drive->tracks){
            dsk_next_track = dsk_active.drive->tracks - 1;
        }
        //Also triggers for SEEK 
        if(cmd_flags && DSK_FLAG_UPD_TRACK){
            dsk_reg_track = dsk_next_track;
        }

    //TYPE II-IV commands
    }else{
        switch(cmd){
            case RSEC:
                dsk_set_status(DSK_STAT_BUSY,true);
                //dsk_set_status(DSK_STAT_HLOADED,true);
                dsk_state = DSK_READ_PREP;
                break;
            case WSEC:
                dsk_set_status(DSK_STAT_BUSY,true);
                //dsk_set_status(DSK_STAT_HLOADED,true);
                dsk_state = DSK_WRITE_PREP;
                break;
            case MISC:
                //Force interrupt
                if(cmd_flags && DSK_FLAG_IS_FINT){
                    //DSK_FLAG_TOREADY
                    //DSK_FLAG_TONREADY
                    dsk_active.index_irq = !!(dsk_reg_cmd & DSK_FLAG_INDEX);
                    //DSK_FLAG_IMMEDIATE
                    if(cmd_flags & DSK_FLAG_IMMEDIATE){
                        //dsk_reg_irq = 0x00;
                        dsk_state = DSK_TOGGLE_IRQ;
                    }else{
                        dsk_state = DSK_CLEANUP;
                    }
                }else{
                //Read address
                    dsk_set_status(DSK_STAT_BUSY,true);
                    //dsk_set_status(DSK_STAT_HLOADED,true);
                    dsk_state = DSK_READ_ADDR;
                }
                break;
            case TRACK:
                dsk_set_status(DSK_STAT_BUSY,true);
                //dsk_set_status(DSK_STAT_HLOADED,true);
                if(cmd_flags && DSK_FLAG_IS_WRITE){
                    dsk_active.data_len = 6400;
                    //dsk_active.data_ptr = (uint8_t *)(dsk_buf);
                    dsk_active.data_start = 0;
                    dsk_state = DSK_WRITE;
                }else{
                //Read track
                    dsk_active.data_len = 6400;
                    //dsk_active.data_ptr = (uint8_t *)(dsk_buf);
                    dsk_active.data_start = 0;
                    dsk_state = DSK_READ;
                }
                break;
            default:
                dsk_state = DSK_IDLE;
                //fail
                break;
        };

    }
    IOREGS(DSK_IO_CMD) = dsk_reg_status;    //Not directly mapped due to combined use with CMD
    //printf("%s %02x %02x\n", dsk_state_id[dsk_state], raw_cmd, dsk_next_track);
    return dsk_next_track;
}

void dsk_act(uint8_t raw_cmd){
    sio_hw->fifo_wr = 0x80000000 | (dsk_reg_ctrl << 8) | raw_cmd;
    dsk_set_status(DSK_STAT_BUSY,true);
    IOREGS(DSK_IO_CMD) = dsk_reg_status;
}

void dsk_rw(bool is_write, uint8_t data){  //data reg accessed. minimum work here
    //dsk_reg_drq = 0x80; //active low
    //dsk_set_status(DSK_STAT_DRQ,false);
    //IOREGS(DSK_IO_CMD) = dsk_reg_status;    //Not directly mapped due to combined use with CMD
    if(is_write && dsk_state == DSK_WRITE){
        dsk_buf[dsk_active.pos++] = data;
    }
   /* if(dsk_state == DSK_READ || dsk_state == DSK_WRITE){
        dsk_byte_cnt++;
        dsk_active.pos++;
    }
    */
}

//Microdisc control register
//Bits 7:EPROM 6-5:drv_sel 4:side_sel 3:DDEN 2:Read CLK/2 1:ROM/RAM 0:IRQ_EN
void dsk_set_ctrl(uint8_t raw_reg){
    if((dsk_reg_ctrl ^ raw_reg) & 0x7d)                  //Only send changed dsk bits
        sio_hw->fifo_wr = 0x00000000 | (raw_reg << 8);   //Transfer with CMD in dsk_act
    dsk_reg_ctrl = raw_reg;             
/*
    //TODO: Read CLK, DDEN
    uint8_t side = (raw_reg >> 4) & 0x01;
    uint8_t drive = (raw_reg >> 5) & 0x03;
    dsk_set_active_drive(drive);
    dsk_set_active_side(side);
*/
}
