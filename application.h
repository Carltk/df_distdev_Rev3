#ifndef DF_APPLICATION_H__
#define DF_APPLICATION_H__

#include <stdint.h>
#include "app_error.h"


// states of pump->pump_state (low nibble)
#define PUMP_STATE_IDLE     0          // Nozzle hung up and 
#define PUMP_STATE_CALL     1          // Nozzle lifted
#define PUMP_STATE_AUTH     2          // Pump has received and authorisation message but has not yet started
#define PUMP_STATE_BUSY     3          // Pump has a transaction in process
#define PUMP_STATE_OR_LEAK  4
#define PUMP_STATE_OR_TRANS 5
#define PUMP_STATE_PAY      6          // Pump is paying off .. i.e. transaction stopped 
#define PUMP_STATE_STOP     7          // Pump transaction has stopped
#define PUMP_STATE_ERROR    0x0F

#define DDPC_INIT_MAGIC_NUM 0xA5        // a magic number to save into the system struct to determine if we've ever run (and if Flash is valid)
#define NUM_DEV_TYPES       4

#define DDPC_ATTRIB_UPTIME      1
#define DDPC_ATTRIB_BOOTS       2
#define DDPC_ATTRIB_FLASH_WR    3
#define DDPC_ATTRIB_TEMP        4
#define DDPC_ATTRIB_RND         5
#define DDPC_ATTRIB_PUMPSTAT    6
#define DDPC_ATTRIB_NOZZLE      7
#define DDPC_ATTRIB_RELAY       8
#define DDPC_ATTRIB_PULSES      9
#define DDPC_ATTRIB_PUMP_STATE  10

typedef enum {
    NORMAL = 0,
    TIMEOUT,
    COMMS
} pump_caller;

/**
 * @brief non-volatile structures -> parameters describing the object at a system level
 * This defines structures that will be stored to NV_Storage
 *    - Some variables are saved on power-down (panic saves)
 *    - Others that change infrequently are saved immediately     
 */

#define PANIC_FILE     (0x8020)         // a file ID and key for flash storage of the panic structures
#define PANIC_REC_KEY  (0x7020)
typedef struct      // Struct to hold vars that need to be panic-saved
{   uint32_t uptime_mins;               // device uptime. Range of 0 - 8000+ years
    uint32_t boot_count;                // number of device boots
    uint32_t flash_writes;              // number of flash writes
    uint32_t flash_erase;
} ddpc_nv_panic_t;

#define FDS_PANIC_REC           \
{   .file_id = PANIC_FILE,      \
    .key = PANIC_REC_KEY,       \
    .data.p_data = &ddpc.nv_panic,  \
    .data.length_words = (sizeof(ddpc_nv_panic_t) + 3) / sizeof(uint32_t),  \
}

#define IMMEDIATE_FILE     (0x8030)     // a file ID and key for flash storage of the immediate structures
#define IMMEDIATE_REC_KEY  (0x7030)
typedef struct      // Struct to hold vars that are written immediately
{   uint8_t  initialised;               // magic-number that identifies initialised flash
    uint16_t dev_address;               // The comms address of the device
} ddpc_nv_immediate_t;


#define FDS_IMMEDIATE_REC                   \
{   .file_id = IMMEDIATE_FILE,              \
    .key = IMMEDIATE_REC_KEY,               \
    .data.p_data = &ddpc.nv_immediate,      \
    .data.length_words = (sizeof(ddpc_nv_immediate_t) + 3) / sizeof(uint32_t),  \
}

/*

    ToDo:
        * On Startup - check for valid storage of flash
            # Valid:
                - Read Panic File data into the memory structure ddpc.nv_panic
                - Read Immediate File data into the memory structure ddpc.nv_immediate
            # Not Valid:
                - Default the structs
                - Immediate-write both structs
            # Increment the boot count. Set ddpc.need_panic_save

        * Update the ddpc.nv_panic.uptime_seconds in the application timer handler. Set ddpc.need_panic_save

        * Implement panic-save detection
        * Implement panic-save 
            - Inc the ddpc.nv_panic.flash_writes


    Functions Required:
        CheckInit()         - Check for intialised flash, set defaults if required
        Flash2Shadow()      - Read Flash files into RAM shadow
        StoreImmediate()    - Save immediate section (nv_immediate) of RAM shadow to flash 
        StorePanic()        - Save panic section (nv_panic) of RAM shadow to flash 

*/

/**
 * @brief ddpc structure -> parameters describing the object at a system level
 */
typedef struct
{
    ddpc_nv_immediate_t nv_immediate;   // RAM shadow of the immediate NV Storage
    ddpc_nv_panic_t nv_panic;           // RAM shadow of the panic-save NV Storage
    uint8_t need_panic_save;            // Flag that indictaes that NV needs to be panic-saved

    // non-permanent device storage
    uint32_t curr_temp;                 
    uint8_t dev_type[NUM_DEV_TYPES];    // Device type(s)
    
    uint8_t my_rnd;
} ddpc_t;
extern ddpc_t ddpc;

// THIS_ADDRESS is defined in the Conditional Compilation section of sdk_config.h

#define DF_DDPC_DFLT    \
{   .nv_immediate.initialised = DDPC_INIT_MAGIC_NUM,    \
    .nv_immediate.dev_address = THIS_ADDRESS,           \
    .nv_panic.uptime_mins = 0,      \
    .nv_panic.boot_count = 1,       \
    .nv_panic.flash_writes = 0,     \
    .nv_panic.flash_erase = 0,      \
    .curr_temp = 0,                 \
    .dev_type[0] = DEV_TYPE_PUMP,   \
    .dev_type[1] = DEV_TYPE_COMP,   \
    .dev_type[2] = DISCOVERY_TYPE,  \
    .dev_type[3] = ALL_CALL_TYPE,   \
    .my_rnd = 0,                    \
}
// ddpc = DF_DDPC_DFLT;

// pump status flags
#define PUMP_STATUS_AVAILABLE   0x01

#define PUMP_STATUS_NOZZLE      0x10    // State of the nozzle
#define PUMP_STATUS_PULSES      0x20    // New Pulses available (clear on read pulses attribute byte)
#define PUMP_STATUS_RELAY       0x40    // State of the output relay


/**
 * @brief pump structure -> parameters describing the logical pump device
 */
typedef struct
{
    uint8_t pump_status;        // status byte as reported to the Rabbit Controller

    // individual control components
    uint8_t nozzle;
    uint8_t relay;
    uint32_t curr_pulses;           // Pulse count shadow of the Timer/Counter

    // the pump logical device
    uint8_t pump_state;             // internal state of the pump (drives the state machine)
    uint32_t transaction_count;     

    uint32_t timer;

} pump_controller_t;
extern pump_controller_t pump;

#define DF_PUMP_DFLT        \
{                           \
    .pump_status = 0,       \
    .pump_state = 0,        \
    .curr_pulses = 0,       \
    .transaction_count = 0  \
}
// pump = DF_PUMP_DFLT;

#define CONTROLLER_STATUS_ALIVE   0x01

#define CONTROLLER_STATUS_NOZZLE  0x10    // State of the nozzle
#define CONTROLLER_STATUS_PULSES  0x20    // New Pulses available (clear on read pulses attribute byte)
#define CONTROLLER_STATUS_RELAY   0x40    // State of the output relay

/**
 * @brief component structure -> parameters describing the logical conponents in the device
 */
typedef struct
{   uint8_t comp_status;

    // the components
    uint8_t nozzle;
    uint8_t relay;
    uint32_t curr_count;

} comp_controller_t;

extern comp_controller_t components;

#define DF_COMP_DFLT        \
{                           \
    .comp_status = 0,       \
}
// components = DF_COMP_DFLT;

#define APP_STATE_100MS     0
#define APP_STATE_1S        1 
#define APP_STATE_10S       2 
#define APP_STATE_1M        3 
#define NUM_APP_STATES      4

/**
 * @brief app_state structure -> controls the app_timer which runs the state machine
 */
typedef struct
{
    // define the tick intervals to perform actions
    int16_t reset_value[NUM_APP_STATES];    // Array of count values for the intervals
    int16_t current_count[NUM_APP_STATES];  // countdown vals for the intervals                           

} app_state_t;

extern app_state_t app_state;

#define DF_APP_STATE_DLFT   \
{                           \
    .reset_value[APP_STATE_100MS] = 1,      \
    .reset_value[APP_STATE_1S] = 10,        \
    .reset_value[APP_STATE_10S] = 100,      \
    .reset_value[APP_STATE_1M] = 600,       \
    .current_count[APP_STATE_100MS] = 1,    \
    .current_count[APP_STATE_1S] = 10,      \
    .current_count[APP_STATE_10S] = 100,    \
    .current_count[APP_STATE_1M] = 600,     \
}
// app_state = DF_APP_STATE_DLFT;

// Global function prototypes
ret_code_t application_init(void);
void update_temp(void);

void do_factory_default(bool SuperDflt);
void pump_action_cmd(pump_controller_t *this_pump, uint8_t cmd);
void pump_change_state(pump_controller_t *this_pump, uint8_t new_state, uint32_t timeout);












#endif // DF_APPLICATION_H__