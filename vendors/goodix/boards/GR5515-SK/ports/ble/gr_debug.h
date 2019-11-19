#ifndef __GR_DEBUG__
#define __GR_DEBUG__

#include "stdlib.h"
#include "stdint.h"
#include "gr551xx.h"
#include "gr_config.h"


#if GR_DEBUG_CODE > 0u

/****************************************************************************
 * function & Vars in this file, are used for debug, not used in release code
 ****************************************************************************/


#define GR_MM_TABLE_LEN                     1024

typedef struct {
    uint16_t    is_saved;
    uint16_t    size;
    uint32_t    addr;
} GR_MM_T;


/* 
 * Not Thread-Safe Now , just for memory-leak test!
 */
void    gr_mm_init(void);
void *  gr_mm_malloc(uint32_t size);
void    gr_mm_free(void * ptr);
void    gr_mm_report(void);

void    gr_debug_force_bkpt(void);

#endif

void    gr_ble_task_startup(void * ctxt);

#endif /*__GR_DEBUG__*/
