#include "df_routines.h"
#include "led_control.h"
#include "hardware_init.h"
#include "app_timer.h"
#include "nrfx_clock.h"

#include "nrf_delay.h"

#include "nrfx_log.h"

#define LED_MS_PER_TICK     125

// *** Global Variables ***
led_control_t LEDS[NUM_LEDS];        // an array of LED control structures
APP_TIMER_DEF(led_timer);                   // Timer definition

// *** function prototypes ***
void set_all_masks(led_control_t *lc, uint32_t mask);
void led_timer_handle(void * p_context);
uint8_t update_led_output(led_control_t *lc);
void led_start_fn(led_control_t *lc);
void led_running_fn(led_control_t *lc);
uint32_t roll_mask_n(uint32_t mask, uint8_t rollVal);
uint32_t roll_mask(uint32_t mask);
void lfclk_request(void);

ret_code_t df_led_init()
{
    ret_code_t ret = NRFX_SUCCESS;
    
    uint32_t newMask;
    uint8_t i;
    
    led_control_t lc = LED_CONTROL_INIT();                  // Build a led control stucture with default values
    uint8_t led_list[NUM_LEDS] = LED_PIN_LIST;              // make an array of led pin assignments for use in building led control structures
    
    for (i = 0; i < NUM_LEDS; i++)                          // build the array of led_control structures with default values
    {   lc.led_num = i;                                     // .. and insert the unique led num
        lc.port_pin = led_list[i];                          // .. and pin assignment

        if (i == RELAY_OUT)
        {   lc.inverted = 0;    
            df_gpio_cfg_output(lc.port_pin, NRF_GPIO_PIN_S0S1);   // Configure the port control pin    
        }
        else
        {   lc.inverted = 1;        
            // if (i == LED_HB)    df_led_cfg_output(lc.port_pin, NRF_GPIO_PIN_S0D1);      // Heartbeat
            // else                df_led_cfg_output(lc.port_pin, NRF_GPIO_PIN_S0D1);      // Error

            if (i == LED_HB)    df_gpio_cfg_output(lc.port_pin, NRF_GPIO_PIN_S0D1);      // Heartbeat
            else                df_gpio_cfg_output(lc.port_pin, NRF_GPIO_PIN_S0D1);      // Error

/*
    xxx no NRF_GPIO_PIN_S0S1 = GPIO_PIN_CNF_DRIVE_S0S1, ///< !< Standard '0', standard '1'.
    xxx no NRF_GPIO_PIN_H0S1 = GPIO_PIN_CNF_DRIVE_H0S1, ///< !< High-drive '0', standard '1'.
    xxx no NRF_GPIO_PIN_S0H1 = GPIO_PIN_CNF_DRIVE_S0H1, ///< !< Standard '0', high-drive '1'.
    xxx no NRF_GPIO_PIN_H0H1 = GPIO_PIN_CNF_DRIVE_H0H1, ///< !< High drive '0', high-drive '1'.
    xxx no NRF_GPIO_PIN_D0S1 = GPIO_PIN_CNF_DRIVE_D0S1, ///< !< Disconnect '0' standard '1'.
    xxx no NRF_GPIO_PIN_D0H1 = GPIO_PIN_CNF_DRIVE_D0H1, ///< !< Disconnect '0', high-drive '1'.
    xxx no NRF_GPIO_PIN_S0D1 = GPIO_PIN_CNF_DRIVE_S0D1, ///< !< Standard '0', disconnect '1'.
    NRF_GPIO_PIN_H0D1 = GPIO_PIN_CNF_DRIVE_H0D1, ///< !< High-drive '0', disconnect '1'.


*/


        }
        
        led_op_do(&lc, 0);                                   // & turn it off
        LEDS[i] = lc;                                       // add the modified structure to the led control array
    }    
    
    set_all_masks(&LEDS[LED_HB], LED_FLASH_SLOW);            // set the default value for the Heartbeat led into current and default masks                       
    led_go(LED_HB, LED_FLASH_FAST, 10000);                  // initialise the HB_LED to fast-flash for 10Sec

    set_all_masks(&LEDS[HB_ERR], LED_FLASH_OFF);            // set the default value for the Error led into current and default masks                       
    led_go(HB_ERR, LED_FLASH_BLIP, 5000);                   // initialise the ERR_LED to fast-flash for 2Sec

    set_all_masks(&LEDS[RELAY_OUT], LED_FLASH_OFF);         // set the default value for the Error led into current and default masks                       
    led_go(RELAY_OUT, LED_FLASH_ON, 2000);                    // initialise the ERR_LED to fast-flash for 25ec


/*    for (i=1; i<NUM_LEDS; i++)                              // initialise all other LEDs to non-synchronised sparse flash
    {   newMask = roll_mask_n(LED_FLASH_SPARSE, (i*5));
        led_go(i, newMask, 5000);      
    }   */

    // This is done in main now ->   lfclk_request();                                        // request (init) a lfclk device for the app_timer configured for LED control

    // This is done in main/hardware_init now -> ret = app_timer_init();                                 // initialise the app_timer facility

    ret = app_timer_create(&led_timer, APP_TIMER_MODE_REPEATED, led_timer_handle);  // Create an AppTimer for the Led controller
    if (ret != NRFX_SUCCESS) goto IL_x;     
    NRFX_LOG_INFO("LED Control - app timer created");           

    ret = app_timer_start(led_timer, APP_TIMER_TICKS(LED_MS_PER_TICK), NULL);       // Start the AppTimer
    if (ret != NRFX_SUCCESS) goto IL_x; 
    app_timer_resume();                                     // Ensure the RTC (source of AppTimer) is (re)started

IL_x:
    return(ret);
}

void led_timer_handle(void * p_context)
{
    led_control_t *lc;

    UNUSED_PARAMETER(p_context);

    for (uint8_t i; i < NUM_LEDS; i++)          // Loop around all the led control blocks
    {   lc = &LEDS[i];
        
        switch (lc->led_state.led_state)
        {
            case LED_STATE_IDLE:                // LED idle ..nothing to do
                break;
            case LED_STATE_INIT:                // LED initialised .. start counting 
                led_start_fn(lc);
                break;
            case LED_STATE_RUNNING:             // LED running .. count & move
                led_running_fn(lc);
                break;
            case LED_STATE_FINISHED:            // LED finished .. clear & go back to default state
                break;
            default:                            // something else .. something has gone wrong .. do a cleanup like finished
                break;
        }
    }
}

void led_start_fn(led_control_t *lc)
{   // called on the first timer tick after a LED output has been enabled
    lc->led_state.current_mask = lc->control_mask;              // move the required mask into the current state
    lc->led_state.countdown = lc->timeout / LED_MS_PER_TICK;    // convert timeout (ms) to 1/8 sec ticks
    lc->led_state.led_on_off = update_led_output(lc);
    lc->led_state.led_state = LED_STATE_RUNNING;                // Set the new state to running
}

void led_running_fn(led_control_t *lc)
{   // called every timer tick when an LED is in a running state
    
    if (lc->single_tick)                                            // if a single-tick has been flagged
    {   led_op_do(lc, lc->led_state.led_on_off);
        lc->led_state.led_on_off ^= 1;       
        lc->single_tick = 0;                                        // disable the flag
        return;
    }

    if (lc->led_state.countdown)                                    // LED timer is running
    {   lc->led_state.countdown -= 1;                               // .. decrement it
        if (lc->led_state.countdown)                                // check if this is a timeout
        {   lc->led_state.led_on_off = update_led_output(lc);  }     // still running .. send the output
        else                                                        // Timed out
        {   led_op_do(lc, 0);                                       // .. turn off the led

            if (lc->control_mask_dflt)                              // check if there is a default value
            {   lc->led_state.countdown = 0;                        // .. there is a default state .. set it to run forever
                lc->control_mask = lc->control_mask_dflt;           // .. move the default mask into the current mask
                lc->led_state.led_state = LED_STATE_INIT;           // .. set to call the init function
            }
            else                                                    // no default mask set
            {   lc->led_state.led_state = LED_STATE_IDLE;    }      // Set the new state to idle
        }
    }
    else                                                            // LED timer is 0 .. i.e. run forever
    {   lc->led_state.led_on_off = update_led_output(lc);   }
}

uint8_t update_led_output(led_control_t *lc)
{   
    lc->led_state.current_mask = roll_mask(lc->led_state.current_mask);

    uint8_t next_bit = (char)(lc->led_state.current_mask & 0x00000001);

    if (lc->led_state.led_on_off != next_bit)                        // if the state of the bit has changed 
    {   led_op_do(lc, next_bit);    }

    return next_bit;
}


uint32_t roll_mask_n(uint32_t mask, uint8_t rollVal)
{   uint32_t tempMask = mask;

    for (uint8_t i=0; i<rollVal; i++)
    {   tempMask = roll_mask(tempMask);     }

    return(tempMask);
}

uint32_t roll_mask(uint32_t mask)
{   uint8_t shifted_bit;

    shifted_bit = (char)mask & 0x00000001;          // get the lsbit
    mask >>= 1;                                     // shift down the mask
    mask |= (shifted_bit ? 0x80000000 : 0L);        // roll the shifted bit up to the msbit

    return(mask);
}

void set_all_masks(led_control_t *lc, uint32_t mask)
{   lc->control_mask = mask;
    lc->control_mask_dflt = mask;
}

void led_control_handler()
{   } // function to parse the led_control_t structures and move them on (if reqauired)

void led_go(uint8_t led, uint32_t controlmask, uint32_t timeout_ms)
{
    if (IS_IN_RANGE(led, 0, (NUM_LEDS - 1)))
    {   led_control_t *lc = &LEDS[led];    
        lc->control_mask = controlmask;             // set the control-pattern 
        lc->timeout = timeout_ms;                   // time in milliseconds to run this pattern before going back to default
        lc->led_state.led_state = LED_STATE_INIT;   // start up the state
    }
}

void led_single_blip(uint8_t led)
{
    if (IS_IN_RANGE(led, 0, (NUM_LEDS - 1)))
    {   led_control_t *lc = &LEDS[led];    
        lc->led_state.led_state = LED_STATE_RUNNING;    // start up the state
        lc->single_tick = 1;                            // Set flag to send a single tick to the LED
    }
}

void led_op_do(led_control_t *lc, uint8_t state)
{   if (state == 0xFF)
    {   nrf_gpio_pin_toggle(lc->port_pin);  }
    else if (state ^ lc->inverted)
    {   nrf_gpio_pin_set(lc->port_pin);  }
    else
    {   nrf_gpio_pin_clear(lc->port_pin);  }
}

void led_stop(uint8_t led)
{
    if (IS_IN_RANGE(led, 0, (NUM_LEDS - 1)))
    {   led_control_t *lc = &LEDS[led];    
        lc->led_state.led_state = LED_STATE_IDLE;   // stop the state
        led_op_do(lc, 0);                           // .. turn off the led
    }        
}



