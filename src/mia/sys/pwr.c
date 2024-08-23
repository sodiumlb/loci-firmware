/*
 * Copyright (c) 2024 Sodiumlightbaby
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "sys/pwr.h"
#include "sys/ext.h"
#include <stddef.h>
#include <stdio.h>

static enum {
    PWR_PASSIVE,
    PWR_VEXT,
    PWR_VBUS,
    PWR_BUS2EXT,
    PWR_ERROR
} pwr_state;

static bool pwr_sense_vext;
static bool pwr_sense_vbus;

void pwr_init(void)
{
    pwr_state = PWR_PASSIVE;
    pwr_sense_vext = ext_get(EXT_SWVEXT);
    pwr_sense_vbus = ext_get_cached(EXT_SWVBUS);

    /* Hardware rev 1.2 has broken switch setup
    if(pwr_sense_vbus && pwr_sense_vext){   //Power ON on both VEXT and VBUS 
        ext_set_dir(EXT_SWVEXT,true);
        ext_put(EXT_SWVEXT, false);
        pwr_state = PWR_ERROR;
    }*/

   //TODO: Power switch setup is currently not optimal while supporting both 1.2 and 1.3 devices
   //Turn off EXT power switch if powered from USB
   if(pwr_sense_vbus && !pwr_sense_vext){
        ext_set_dir(EXT_SWVEXT, true);
        ext_put(EXT_SWVEXT, false);
        pwr_state = PWR_VBUS;
   //Turn on VBUS power switch if powered from EXT and not USB 
   }else if(pwr_sense_vext && !pwr_sense_vbus){
        ext_set_dir(EXT_SWVBUS, true);
        ext_put(EXT_SWVBUS, true);
        pwr_state = PWR_VEXT;
   }

}


void pwr_task(void)
{
    switch(pwr_state){
        //TODO Add delay before deciding power direction?
        case PWR_PASSIVE:
        /*
            pwr_sense_vext = ext_get_cached(EXT_SWVEXT);
            pwr_sense_vbus = ext_get_cached(EXT_SWVBUS);
            if(pwr_sense_vext && !pwr_sense_vbus){
                printf("Switch VBUS OUT ON");
                ext_set_dir(EXT_SWVBUS,true);
                ext_put(EXT_SWVBUS, false);
                ext_put(EXT_SWVBUS, true);
                pwr_state = PWR_VEXT;
            }
        */
            break;
        case PWR_VEXT:
            break;
        case PWR_VBUS:
            //TODO implement Oric power switch functionality
            break;
        case PWR_BUS2EXT:
            break;
        case PWR_ERROR:
            break;
    }
}
