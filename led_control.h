
#ifndef DF_LED_CONTROL_H__
#define DF_LED_CONTROL_H__

#include <stdint.h>
#include "nrf_gpio.h"
#include "hardware_init.h"

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

#define NUM_PROC_LED_SLOTS 20           // The number of Processor LED control slots. Display routine will cycle through these slots if they are filled

#define PROC_LED_RED        {255,0,0}       // Red used for Error status
#define PROC_LED_GREEN      {0,255,0}       // Green used for OK status    

#define PROC_LED_ORANGE     {255,128,0}     // Orange used for "Transaction" indication
#define PROC_LED_YELLOW     {255,255,0}     // Yellow used for pushbutton timing indication
#define PROC_LED_CYAN       {0,255,255}     // Cyan used for "Comms" indication
#define PROC_LED_BLUE       {0,0,255}       
#define PROC_LED_MAGENTA    {255,0,255}     // Magenta used for "Memory" indication

typedef enum LED_FLASH
{   LED_FLASH_OFF =     0x00000000,
    LED_FLASH_ON =      0xFFFFFFFF,
    LED_FLASH_FAST =    0xAAAAAAAA,
    LED_FLASH_MED =     0xF0F0F0F0,
    LED_FLASH_SLOW =    0xFF00FF00,
    LED_FLASH_BLIP =    0x01010101,
    LED_FLASH_SPARSE =  0x00010001,
} led_flash_pattern_t;

typedef struct
{
    uint32_t flashPatternStatus;   // Current rolling value of the flahsPattern each iteration rolls through this value    
    uint8_t cycleDwellCount;       // Countdown for the cycle dwell countdown (at 0 rolls on to the next slot) 
    uint8_t slotTimeLeft;          // Countdown for the slot timeout (at 0 the slot is deleted) 
    
    
} procled_status_t;

typedef struct
{   uint8_t colourVal[3];           // Colour value array as [Red,Green,Blue]

    uint32_t flashPattern;          // Flash pattern allows control over the display pattern. Each bit represents 1/8 second.
    uint8_t  cycleDwell;            // Count of how long (in 1/8 sec increments) to remain in this slot before yielding to the next slot
    uint32_t slotTimeout;           


    procled_status_t status;        // Status of this procled slot

} proc_led_t;
extern proc_led_t procled[NUM_PROC_LED_SLOTS];      // An array of proc_led control slots
extern uint8_t procled_index[NUM_PROC_LED_SLOTS];   // An array of pointers to the slots to allow for easier addition/deletion & reordering
extern uint8_t procled_current;                     // Variable to hold the index to procled_index[] that is currently being processes


//TODO create a function to add new packets to the array
//TODO create a function to reorganise the procled_index array based on packets being added/removed
//TODO create a function to remove packets from the array


#define PROCLED_OK_DEFAULT              \
{   .colourVal = PROC_LED_GREEN,        \
    .flashPattern = LED_FLASH_MED,      \
    .cycleDwell = 16,                   \
    .slotTimeout = 0,                   \
    .status.flashPatternStatus = 0,     \
    .status.cycleDwellCount = 0,        \
    .status.slotTimeout = 0,            \
}





/**
 * @brief Function for initializing the Status LED(s)
 * Uses plain GPIO for LED output control and APP_TIMER for timer functions
 */
ret_code_t df_led_init(void);

/**
 * @brief Function to start an LED flash pattern
 *
 * @param[in] led - the number (or name) of the LED
 * @param[in] controlmask - the flash pattern mask for the led use one of LED_FLASH_<xxx>
 * @param[in] timeout - the timeout for this pattern in mSec
 *
 */
void led_go(uint32_t controlmask, uint32_t timeout_ms);

/**
 * @brief Function to stop an LED flash pattern and turn off the LED
 * @param[in] led - the number (or name) of the LED
 */
void led_stop();

/**
 * @brief Function to emit a single 1/8th second blip from the LED
 * @param[in] led - the number (or name) of the LED
 */
void led_single_blip();


#endif // DF_LED_CONTROL_H__