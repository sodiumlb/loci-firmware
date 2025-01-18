#include "main.h"
#include "sys/aut.h"
#include "sys/mia.h"
#include "mia/api/api.h"
#include "mia/api/mnt.h"

uint8_t mnt_mount(uint8_t drive, char* path);

bool auto_run(void){
    // DSK
    if(0==mnt_mount(0,"1:/autorun.dsk")){
        API_A = MIA_BOOTSET_B11 | MIA_BOOTSET_FDC | MIA_BOOTSET_RESUME /*| MIA_BOOTSET_FAST*/;
        return true;
    }
    // TAP
    if(0==mnt_mount(4,"1:/autorun.tap")){
        API_A = MIA_BOOTSET_B11 | MIA_BOOTSET_TAP | MIA_BOOTSET_TAP_ALD | MIA_BOOTSET_RESUME /*| MIA_BOOTSET_FAST*/;
        return true;
    }

    return false;
}
