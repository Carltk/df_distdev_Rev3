
#ifndef DF_LED_CONTROL_H__
#define DF_LED_CONTROL_H__

#include <stdint.h>
#include "nrf_gpio.h"
#include "hardware_init.h"

/** @brief 'potted' led-control flash patterns */
typedef enum LED_FLASH
{   LED_FLASH_OFF =     0x00000000,
    LED_FLASH_ON =      0xFFFFFFFF,
    LED_FLASH_FAST =    0xAAAAAAAA,
    LED_FLASH_MED =     0xF0F0F0F0,
    LED_FLASH_SLOW =    0xFF00FF00,
    LED_FLASH_BLIP =    0x01010101,
    LED_FLASH_SPARSE =  0x00010001,
} led_flash_pattern_t;

typedef enum LED_STATE
{   LED_STATE_IDLE,
    LED_STATE_INIT,
    LED_STATE_RUNNING,
    LED_STATE_FINISHED,
} led_state_enum_t;

/** @brief A led-state type definition */
typedef struct
{   uint8_t     led_state;
    uint32_t    current_mask;           // the in-process control-pattern 
    uint32_t    countdown;              // current state of the countdown in 1/8 second ticks
    int8_t      led_on_off;             // on/off state of the led
} led_state_t;

/** @brief A led-control type definition */
typedef struct
{   uint8_t     led_num;
    uint8_t     port_pin;
    uint8_t     inverted;               // Flag to indicate an inverted output
    uint8_t     single_tick;            // Flag to send a single tick to the LED    
    uint32_t    control_mask;           // a control-pattern 
    uint32_t    timeout;                // time in milliseconds to run this pattern before going back to default
    uint32_t    control_mask_dflt;      // the default contro-pattern
    led_state_t led_state;              // current state of this led    

} led_control_t;

extern led_control_t LEDS[NUM_LEDS];        // There will be a global array of LED control structures


/**
 * @brief Macro to return a blank configuration structure
 */
#define LED_CONTROL_INIT()                              \
{                                                       \
    .led_num = 0,                                       \
    .port_pin = 0,                                      \
    .inverted = 0,                                      \
    .single_tick = 0,                                   \
    .control_mask = LED_FLASH_OFF,                      \
    .timeout = 0,                                       \
    .control_mask_dflt = LED_FLASH_OFF,                 \
    .led_state.led_state = LED_STATE_IDLE,              \
    .led_state.current_mask = LED_FLASH_OFF,            \
    .led_state.countdown = 0,                           \
    .led_state.led_on_off = 0,                          \
}

/**
 * @brief Function for initializing the Status LED(s)
 *
 * Uses plain GPIO for LED output control and APP_TIMER for timer functions
 *
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
void led_go(uint8_t led, uint32_t controlmask, uint32_t timeout_ms);

/**
 * @brief Function to stop an LED flash pattern and turn off the LED
 *
 * @param[in] led - the number (or name) of the LED
 *
 */
void led_stop(uint8_t led);

/**
 * @brief Function to emit a single 1/8th second blip from the LED
 *
 * @param[in] led - the number (or name) of the LED
 *
 */
void led_single_blip(uint8_t led);


/**
 * @brief Function to force the led to a certain state
 *
 * @param[in] *lc - pointer to the led control structure
 * @param[in] state - the required state of the LED 0=OFF, 1=ON, 0xFF=TOGGLE
 *
 */
void led_op_do(led_control_t *lc, uint8_t state);


#endif // DF_LED_CONTROL_H__