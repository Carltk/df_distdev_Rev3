/**
*/

#ifndef NV_STORE_H__
#define NV_STORE_H__

#include <stdint.h>
#include "fds.h"
#include "application.h"

// ddpc_nv_immediate_t  - defined in application.h
// ddpc_nv_panic_t      - defined in application.h

typedef struct
{   bool fds_initialised;
    bool gc_required;
    bool delete_pending;
    bool delete_next;
    bool need_panic_save;
    bool do_immediate_save;
} flash_control_t;
extern flash_control_t volatile flash_control;


// Prototypes defined in NV_Store.c
ret_code_t NV_Store_init(void);
void start_nv_storage(void);
ret_code_t do_fds_gc(void);
bool check_gc_required(bool set_gc_flag);

ret_code_t if_do_store_panic(void);
ret_code_t StoreImmediate(void);

void fds_rec_delete(uint32_t file, uint32_t key);
void delete_all_begin(void);
void delete_all_process(void);
void show_immediate(void);
void show_panic(void);


#endif
