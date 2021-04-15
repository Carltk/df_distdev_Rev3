
#ifndef DF_LED_CONTROL_H__
#define DF_LED_CONTROL_H__

#include <stdint.h>
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrfx_i2s.h"

#include "hardware_init.h"
#include "console_comms.h"
#include "application.h"

/*
* New LED controller uses the WS2812C - programmable RGB LED
    https://datasheet.lcsc.com/szlcsc/1810231210_Worldsemi-WS2812C_C114587.pdf
    
* Basic control uses the nrf52840's I2S peripheral 
    Code based on: https://electronut.in/nrf52-i2s-ws2812/
    
    The WS2812 takes 3 colour channels x 8 bits of resolution. It uses a NRZ encoding scheme where 0=<0.4uS ON, 0.8uS OFF>, 1=<0.8uS ON, 0.4uS ON>.
    Therefore, each WS2812 bit is ~1.25uS and each colour channel's word is ~10uS.
    To latch the colour channels into the LED driver, the WS2812 needs a >50uS reset pulse.
    The i2s peripheral runs at 4x that rate so each WS2812 bit is 4 i2s bits. To encode an ON, send 1110 from the i2s, OFF=1000 from i2s.
    The i2s peripheral send out a tx buffer that contains 3 colour channels of 32 bits.
    The reset pulse can be sent by sending sufficient tx buffer elements of 0 to exceed 50uS. 
    As we're running i2s at 10uS per 32bit buffer element we can make this 6 elements.



On top of the hardware drive layer is a pattern control layer.
This is controlled by a stack of data structures
    Each element has:
    * Colour (24 bits (3x8) RGB)
    * Display pattern - rolls through 32 bits in 1/8 second ticks
    * Cycle time (next element will be displayed after cycle time)
    * Element Timeout - array element will be deleted after timeout unless a permament (0?) timeout
    
    * Element current status

*/

#define NUM_PROC_LED_SLOTS  20           // The number of Processor LED control slots. Display routine will cycle through these slots if they are filled
#define NUM_COLOURS         3
#define BYTES_IN_UINT32     4

// extern nrfx_i2s_config_t i2s_config;

extern const uint8_t PROC_LED_RED[];
extern const uint8_t PROC_LED_GREEN[];
extern const uint8_t PROC_LED_YELLOW_GREEN[];
extern const uint8_t PROC_LED_ORANGE[];
extern const uint8_t PROC_LED_YELLOW[];
extern const uint8_t PROC_LED_CYAN[];
extern const uint8_t PROC_LED_BLUE[];
extern const uint8_t PROC_LED_MAGENTA[];
extern const uint8_t PROC_LED_VIOLET[];
extern const uint8_t PROC_LED_WHITE[];

// LED flash patterns. This is a rolling buffer of bits. Each bit dwells for 1/8 second i.e. F denotes 1/2 second 

typedef enum LED_FLASH                      
{   LED_FLASH_OFF =     0x00000000,
    LED_FLASH_ON =      0xFFFFFFFF,
    LED_FLASH_FAST =    0xAAAAAAAA,
    LED_FLASH_MED =     0x0F0F0F0F,
    LED_FLASH_SLOW =    0x00FF00FF,
    LED_FLASH_BLIP =    0x01010101,
    LED_FLASH_SPARSE =  0x00010001,
    LED_SINGLE_FLASH =  0x0000000F,
    LED_DOUBLE_FLASH =  0x00000F0F,
    LED_TRIPLE_FLASH =  0x000F0F0F,
} led_flash_pattern_t;

typedef struct
{
    uint32_t rolledFlashPattern;   // Current rolling value of the flahsPattern each iteration rolls through this value    
    uint8_t cycleDwellCount;       // Countdown for the cycle dwell countdown (at 0 rolls on to the next slot) 
    uint8_t slotTimeLeft;          // Countdown for the slot timeout (at 0 the slot is deleted) 
} procled_status_t;

typedef struct
{   bool inUse;
    uint8_t colourVal[NUM_COLOURS];           // Colour value array as [Red,Green,Blue]

    uint32_t flashPattern;          // Flash pattern allows control over the display pattern. Each bit represents 1/8 second.
    uint8_t  cycleDwell;            // Count of how long (in 1/8 sec increments) to remain in this slot before yielding to the next slot
    uint8_t slotTimeout;            // Count of how long before deleting itself (max = 255 / 8 =~ 30Sec) 

    uint8_t  linkNext;              // index of the next structure (0xFF to just use index-next)
    uint32_t (*p_run_after_dwell)();    // pointer to a function to run on cycleDwell timeout (NULL to disable)

    procled_status_t status;        // Status of this procled slot
} proc_led_t;

#define PROCLED_OK_DEFAULT              \
{   .colourVal = PROC_LED_GREEN,        \
    .flashPattern = LED_FLASH_MED,      \
    .cycleDwell = 16,                   \
    .slotTimeout = 0,                   \
    .linkNext = 0xFF,                   \
    .status.rolledFlashPattern = 0,     \
    .status.cycleDwellCount = 0,        \
    .status.slotTimeout = 0,            \
}


/**
 * @brief Function for initializing the Status LED(s)
 * Uses plain GPIO for LED output control and APP_TIMER for timer functions
 */
ret_code_t df_led_init(void);


/**
 * @brief Function to clear all LED pattern slots
  */
void wipeAllLEDSlots(void);

/**
 * @brief Function to add an LED flash pattern to the stack
 *
 * @param[in] colourAry - an array of RGB for the colour of the LED
 * @param[in] flashPattern - the flash pattern mask for the led use one of LED_FLASH_<xxx>
 * @param[in] cycleDwell - how many 1/8 time slices to stay before moving to the next pattern
 * @param[in] slotTimeout - how many 1/8 time slices to exist before automatic deletion of the slot
 * @param[in] p_func - pointer to a function to run on dwell timeout of this slot
 * @param[in] link - index into the procled[] array to service next once this slot's cycleDwell times out
 * @param[out] thisSlot - the slot in the procled[] array that this data was stored in
 *
 */
uint8_t addLEDPattern(const uint8_t * colourAry, uint32_t flashPattern, uint8_t cycleDwell, uint8_t slotTimeout, void * p_func, uint8_t link);

/**
 * @brief Function to create a LED flash pattern closed loop.
 *   Looped slots must be added backwards i.e. last colour first
 *
 * @param[in] topIdx - the first procled[] index added (i.e. last in the pattern)
 * @param[in] bottomIdx - the last procled[] index added (i.e. first in the pattern)
*/
void loopLEDPattern(uint8_t topIdx, uint8_t bottomIdx);

/**
 * @brief Function to delete a LED flash pattern slot
 *
 * @param[in] PL - the array index of the procled[] slot to clear
 * @param[out] linkNext - the value of the linkNext from the slot (or 0xFF if self-referenced or no linkNext)
*/
uint8_t clearLEDSlot(uint8_t Idx);

/**
 * @brief Function to poke a changed LED display value into the LED
 *
 * @param[in] if value is supplied, copy it into procled_current (set to 0xFF to use current value of procled_current)
*/
void ledNewValPoke(uint8_t Idx);

/**
 * @brief Function to set up standard operating mode system flashes
 * 
 *
 * @param[in] no value
*/
static void setLEDSystemStatus(void);


/**
 * @brief Function to make comms status flash pattern based on the current comms state
 *
 * @param idx       feed-in flash pattern index for chaining
 * @param single    single mode (for run-mode) or dual-line for system status
 * @param cs        comms state to signal via LED
 * @return          feed-out index for chaining
 * */
uint8_t makeCommsStatusFlashes(uint8_t idx, comms_state_t cs, bool single);

/**
 * @brief Function to make pump status flash pattern based on the current pump state
 *
 * @param idx       feed-in flash pattern index for chaining
 * @param single    single mode (for run-mode) or dual-line for system status
 * @param ps        pump state to signal via LED
 * @return          feed-out index for chaining
 * */
uint8_t makePumpStatusFlashes(uint8_t idx, pump_state_t ps, bool single);

#endif // DF_LED_CONTROL_H__