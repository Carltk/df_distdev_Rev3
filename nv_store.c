
#include "nv_store.h"

#include "nrf.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "fds.h"
#include "app_timer.h"

#include "nrfx_log.h"
#include "nrf_log_ctrl.h"


/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/* Flag block for flash control */
flash_control_t volatile flash_control;

uint32_t tt_start;

void TestTimer(bool start)
{   uint32_t ticks_per_ms = APP_TIMER_TICKS(1);
    
    if (start)
    {   tt_start = app_timer_cnt_get();     }
    else
    {   uint32_t tt_stop = app_timer_cnt_get();
        NRFX_LOG_INFO("*** Event Timer: [%ld] ticks (%d ticks per mS) ***", app_timer_cnt_diff_compute(tt_stop, tt_start), ticks_per_ms);
    }
}

void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {   flash_control.fds_initialised = true;   }
            break;
        case FDS_EVT_WRITE:
            TestTimer(false);                                // !!!CK Here - testing
            if (p_evt->result == NRF_SUCCESS)
            {   NRFX_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRFX_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRFX_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
            break;
        case FDS_EVT_UPDATE:
            TestTimer(false);                                // !!!CK Here - testing
            if (p_evt->result == NRF_SUCCESS)
            {   NRFX_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRFX_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
            }
            break;
        case FDS_EVT_DEL_RECORD:
            TestTimer(false);                                // !!!CK Here - testing
            if (p_evt->result == NRF_SUCCESS)
            {   NRFX_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRFX_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRFX_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            flash_control.delete_pending = false;
            break;
        //case FDS_EVT_DEL_FILE:
        case FDS_EVT_GC:
            TestTimer(false);                                // !!!CK Here - testing
            if (p_evt->result == NRF_SUCCESS)
            {   ddpc.nv_panic.flash_erase += 1;
                flash_control.need_panic_save = true;
                NRFX_LOG_INFO("Garbage Collection Successful");  }
            break;
        default:
            break;
    }

    if (p_evt->result == NRF_SUCCESS)
    {   NRFX_LOG_INFO("Event: %s received (NRF_SUCCESS)", fds_evt_str[p_evt->id]);   }
    else
    {   NRFX_LOG_INFO("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str(p_evt->result));   }
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

/**@brief   Wait for fds to initialize. */
void wait_for_fds_ready(void)
{   while (!flash_control.fds_initialised)
    {   power_manage();      }
}

/**@brief   Initialise the Device Non-Volatile storage. */
ret_code_t NV_Store_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    ret = fds_register(fds_evt_handler);    // Register first to receive an event when initialization is complete. 
    if (ret != NRFX_SUCCESS) goto NI_x;     
    
    ret = fds_init();
    if (ret != NRFX_SUCCESS) goto NI_x;         
    
    wait_for_fds_ready();                   // Wait for fds to initialize. 
    NRF_LOG_INFO("Flash Data Storage - Initialised and ready for use");
    return(ret);

NI_x:
    NRF_LOG_INFO("Flash Data Storage - Initialisation Failure");
    return(ret);
}

/**@brief   Check then initialise if necessary the DDPC non-volatile structures. */
ret_code_t nv_store_check_init()
{   // Check for intialised flash, set defaults if required
    ret_code_t ret = NRFX_SUCCESS;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    ret = fds_record_find(IMMEDIATE_FILE, IMMEDIATE_REC_KEY, &desc, &tok);
    if (ret == NRFX_SUCCESS)
    {   NRF_LOG_INFO("FDS Immediate space exists.");   }
    else
    {   fds_record_t fds_immediate = FDS_IMMEDIATE_REC;
        
        ddpc.nv_panic.flash_writes += 1;

        TestTimer(true);                                // !!!CK Here - testing
        ret = fds_record_write(&desc, &fds_immediate);
        if (ret == NRF_SUCCESS)
        {   flash_control.need_panic_save = true;   }
        else
        {   NRF_LOG_INFO("FDS Immediate creation .. Error - %s", fds_err_str(ret));
            return(ret);
        }

        ret = fds_record_close(&desc);                  // Close the record when done reading.
    }

    ret = fds_record_find(PANIC_FILE, PANIC_REC_KEY, &desc, &tok);
    if (ret == NRFX_SUCCESS)
    {   NRF_LOG_INFO("FDS Panic space exists.");   }
    else
    {   fds_record_t fds_panic = FDS_PANIC_REC;
        
        TestTimer(true);                                // !!!CK Here - testing
        ddpc.nv_panic.flash_writes += 1;
        ret = fds_record_write(&desc, &fds_panic);
        
        if (ret == NRF_SUCCESS)
        {   flash_control.need_panic_save = false;
            NRF_LOG_INFO("FDS Panic file written OK");      // System config not found; write a new one.
        }
        else
        {   NRF_LOG_INFO("FDS Panic creation .. Error - %s", fds_err_str(ret)); }

        ret = fds_record_close(&desc);                  // Close the record when done reading.
    }

    return(ret);
}

ret_code_t immediate_to_shadow()
{   ret_code_t ret = NRFX_SUCCESS;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    ret = fds_record_find(IMMEDIATE_FILE, IMMEDIATE_REC_KEY, &desc, &tok);
    if (ret == NRF_SUCCESS)
    {   fds_flash_record_t flash_rec = {0};            // A config file is in flash. Let's update it. 
        
        ret = fds_record_open(&desc, &flash_rec);      // Open the record and read its contents.
        APP_ERROR_CHECK(ret);

        memcpy(&ddpc.nv_immediate, flash_rec.p_data, sizeof(ddpc_nv_immediate_t));   //  Copy the configuration from flash into m_dummy_cfg. 

        show_immediate();

        ret = fds_record_close(&desc);                  // Close the record when done reading.
        APP_ERROR_CHECK(ret);
    }

    return(ret);
}

ret_code_t panic_to_shadow()
{   ret_code_t ret = NRFX_SUCCESS;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    ret = fds_record_find(PANIC_FILE, PANIC_REC_KEY, &desc, &tok);
    if (ret == NRF_SUCCESS)
    {   fds_flash_record_t flash_rec = {0};
        
        ret = fds_record_open(&desc, &flash_rec);                       // Open the record and read its contents.
        APP_ERROR_CHECK(ret);

        memcpy(&ddpc.nv_panic, flash_rec.p_data, sizeof(ddpc_nv_panic_t));   //  Copy the configuration from flash into m_dummy_cfg. 

        show_panic();
        ret = fds_record_close(&desc);                              // Close the record when done reading.
        APP_ERROR_CHECK(ret);
    }

    return(ret);
}

ret_code_t Flash2Shadow()
{   // Read Flash files into RAM shadow
    ret_code_t ret = NRFX_SUCCESS;

    ret = immediate_to_shadow();
    APP_ERROR_CHECK(ret);

    ret = panic_to_shadow();
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("Immediate and Panic Flash read into RAM shadow");  

    return(ret);
}

ret_code_t StoreImmediate(void)
{   // Save immediate section (nv_immediate) of RAM shadow to flash 
    ret_code_t ret = NRFX_SUCCESS;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    ret = fds_record_find(IMMEDIATE_FILE, IMMEDIATE_REC_KEY, &desc, &tok);
    if (ret == NRF_SUCCESS)
    {   
        TestTimer(true);                                // !!!CK Here - testing
        fds_record_t fds_immediate = FDS_IMMEDIATE_REC;
        ret = fds_record_update(&desc, &fds_immediate);     // Write the updated record to flash.
        if (ret == NRF_SUCCESS) 
        {   ddpc.nv_panic.flash_writes += 1;
            flash_control.need_panic_save = true;
            NRF_LOG_INFO("StoreImmediate .. Successful");     
        }
        else
        {   NRF_LOG_INFO("StoreImmediate .. Error - %s", fds_err_str(ret));     }

        ret = fds_record_close(&desc);                              // Close the record when done reading.
    }
    else
    {   NRF_LOG_INFO("StoreImmediate .. Error: Record not found");  }

    flash_control.do_immediate_save = false;                        // Clear the flag

    return(ret);
}

ret_code_t StorePanic()
{   //Save panic section (nv_panic) of RAM shadow to flash 
    ret_code_t ret = NRFX_SUCCESS;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    ret = fds_record_find(PANIC_FILE, PANIC_REC_KEY, &desc, &tok);
    if (ret == NRF_SUCCESS)
    {       
        fds_record_t fds_panic = FDS_PANIC_REC;
        ddpc.nv_panic.flash_writes += 1;                // A little wierd .. pre-increment the flash-write counter rather than post-inc the flag is de-aaserted after the update below

        TestTimer(true);                                // !!!CK Here - testing
        ret = fds_record_update(&desc, &fds_panic);     // Write the updated record to flash.
        if (ret == NRF_SUCCESS) 
        {   flash_control.need_panic_save = false;  
            NRF_LOG_INFO("StorePanic .. Successful");        
        }
        else
        {   NRF_LOG_INFO("StorePanic .. Error - %s", fds_err_str(ret));     }

        ret = fds_record_close(&desc);                              // Close the record when done reading.
    }
    else
    {   NRF_LOG_INFO("StorePanic .. Error: Record not found");  }

    return(ret);
}

ret_code_t if_do_store_panic(void)
{   if (flash_control.need_panic_save)
    {   return(StorePanic());   }

    return(NRFX_SUCCESS);
}

ret_code_t get_nv_stat()
{   ret_code_t ret = NRFX_SUCCESS;
    fds_stat_t stat = {0};

    ret = fds_stat(&stat);
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("NV Status");
    NRF_LOG_INFO(" - %d valid recs.", stat.valid_records);
    NRF_LOG_INFO(" - %d dirty recs (GC-ready).", stat.dirty_records);
    NRF_LOG_INFO(" - %d corruption.", stat.corruption);
    NRF_LOG_INFO(" - %d freeable recs", stat.freeable_words);
    NRF_LOG_INFO(" - %d largest contig", stat.largest_contig);
    NRF_LOG_INFO(" - %d open recs", stat.open_records);
    NRF_LOG_INFO(" - %d pages avail", stat.pages_available);
    NRF_LOG_INFO(" - %d reserved words", stat.words_reserved);
    NRF_LOG_INFO(" - %d words used", stat.words_used);
    NRF_LOG_FLUSH();
}

void start_nv_storage(void)
{   ret_code_t ret = NRFX_SUCCESS;

    //fds_rec_delete(CONFIG_FILE, CONFIG_REC_KEY);
    //fds_rec_delete(IMMEDIATE_FILE, IMMEDIATE_REC_KEY);
    //fds_rec_delete(PANIC_FILE, PANIC_REC_KEY);

    get_nv_stat();

    if (nv_store_check_init() == NRF_SUCCESS)
    {   APP_ERROR_CHECK(Flash2Shadow());    }
}

ret_code_t do_fds_gc(void)
{   TestTimer(true);                                // !!!CK Here - testing
    return(fds_gc());   
}

bool check_gc_required(bool set_gc_flag)
{
    bool ret = false;
    fds_stat_t stat = {0};

    ret_code_t rstat = fds_stat(&stat);
    if (rstat == NRF_SUCCESS)
    {   if (stat.freeable_words > (stat.largest_contig / 2))        // clear if freeable (i.e. dirty) words > half the remaining words
        {   ret = true;
            if (set_gc_flag)
            {   flash_control.gc_required = true;   }
            NRF_LOG_INFO("GC advised .. Freeable [%d] > 50% available [%d]", stat.freeable_words, (stat.largest_contig / 2) ); 
        }
    }
    else
    {   NRF_LOG_INFO("Unable to get FDS status");      }

    return(ret);
} 


/**@brief   Delete a specific record by fileID/keyID. */
void fds_rec_delete(uint32_t file, uint32_t key)
{
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};

    if (fds_record_find(file, key, &desc, &tok) == NRF_SUCCESS)
    {
        TestTimer(true);                                // !!!CK Here - testing
        ret_code_t rc = fds_record_delete(&desc);
        if (rc == NRF_SUCCESS)
        {   NRFX_LOG_INFO("Record deletion error: %s", fds_err_str(rc));    }
        else
        {   NRFX_LOG_INFO("Record deletion [ %x : %x ] - Done", file, key);    }
    }
    else
    {   NRFX_LOG_INFO("Record deletion .. file [ %x : %x ] not found", file, key);    }
}

/**@brief   Begin deleting all records, one by one. */
void delete_all_begin(void)
{   flash_control.delete_next = true;   }

bool record_delete_next(void)
{   bool ret = false;
    
    fds_find_token_t  tok   = {0};
    fds_record_desc_t desc  = {0};

    if (fds_record_iterate(&desc, &tok) == NRF_SUCCESS)
    {   
        flash_control.delete_pending = true;
        ret_code_t rc = fds_record_delete(&desc);
        if (rc == NRF_SUCCESS)
        {   ret = true;     }
    }
    // else  /* No records left to delete. */

    return(ret);
}

/**@brief   Process a delete all command.
 *
 * Delete records, one by one, until no records are left.
 */
void delete_all_process(void)
{
    if (flash_control.delete_next & !flash_control.delete_pending)
    {   NRF_LOG_INFO("Deleting next record.");

        flash_control.delete_next = record_delete_next();
        if (!flash_control.delete_next)
        {   NRF_LOG_INFO("No records left to delete."); }
    }
}

void show_immediate(void)
{   NRFX_LOG_INFO("Immediate Rec: Init[%x], Address[%x]", ddpc.nv_immediate.initialised , ddpc.nv_immediate.dev_address);   }

void show_panic(void)
{   NRFX_LOG_INFO("Panic Rec: Boots[%ld], FlashWrites[%ld], Erases[%ld], Uptime[%ld]", ddpc.nv_panic.boot_count, ddpc.nv_panic.flash_writes, ddpc.nv_panic.flash_erase, ddpc.nv_panic.uptime_mins); }