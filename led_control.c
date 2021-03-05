#include "df_routines.h"
#include "led_control.h"
#include "hardware_init.h"
#include "app_timer.h"
#include "nrfx_clock.h"
#include "nrfx_i2s.h"

#include "nrf_delay.h"

#include "nrfx_log.h"

#define LED_MS_PER_TICK 125                         // LED flash timer resolution = 125mS = 1/8 second
#define RESET_BITS      6                           // Enough reset bits to output >50uS at 10uS per 32-bit buffer
#define I2S_BUFFER_SIZE 3 + RESET_BITS              // i2s buffer is 3x colour channels + reset blocks

// *** Global Variables ***
APP_TIMER_DEF(led_timer);                   // Timer definition
static volatile bool g_i2s_start = true;
static volatile bool g_i2s_running = false;

proc_led_t procled[NUM_PROC_LED_SLOTS];      // An array of proc_led control slots
uint8_t procled_index[NUM_PROC_LED_SLOTS];   // An array of pointers to the slots to allow for easier addition/deletion & reordering
uint8_t procled_current;                     // Variable to hold the index to procled_index[] that is currently being processes

static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];


// *** function prototypes ***
void set_all_masks(led_control_t *lc, uint32_t mask);
void led_timer_handle(void * p_context);
uint8_t update_led_output(led_control_t *lc);
void led_start_fn(led_control_t *lc);
void led_running_fn(led_control_t *lc);
uint32_t roll_mask_n(uint32_t mask, uint8_t rollVal);
uint32_t roll_mask(uint32_t mask);
void lfclk_request(void);

uint32_t calcChannelValue(uint8_t level);
nrfx_i2s_data_handler_t i2s_data_handler((nrfx_i2s_buffers_t const * p_released, uint32_t status);

void setDefaultLEDState(bool startupMode, bool clearAll);



// This is the I2S data handler - all data exchange related to the I2S transfers is done here.
nrfx_i2s_data_handler_t i2s_data_handler((nrfx_i2s_buffers_t const * p_released, uint32_t status)
{   if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
    {   NRFX_LOG_INFO("TEST** i2s: Status Next Buffers Needed (finished?)");
        nrfx_i2s_stop();        
        g_i2s_running = false;
    }      
}

// because of the way i2s encodes signals (i.e. Left/Right)
uint32_t calcChannelValue(uint8_t level)
{   uint32_t val = 0;
    
    if(level == 0)                 // 0 
    {   val = 0x88888888;   }
    else if (level == 255)         // 255
    {   val = 0xeeeeeeee;  }
    else                           // apply 4-bit 0xe HIGH pattern wherever level bits are 1. 
    {   val = 0x88888888;
        for (uint8_t i = 0; i < 8; i++) {
            if((1 << i) & level) {
                uint32_t mask = ~(0x0f << 4*i);
                uint32_t patt = (0x0e << 4*i);
                val = (val & mask) | patt;
            }
        }
        val = (val >> 16) | (val << 16);        // swap 16 bits
    }
    return val;

}

void fillBuffers(uint8_t RVal, uint8_t GVal, uint8_t BVal)
{   m_buffer_tx[0] = calcChannelVal(GVal);              // buffers hold GRB
    m_buffer_tx[1] = calcChannelVal(RVal);
    m_buffer_tx[2] = calcChannelVal(BVal);
    
    memset(&m_buffer_tx[3], 0x00, (4 * RESET_BITS));    // Clear 6 buffers from [3] onwards to make the >50uS reset pulse
}

ret_code_t sendLEDBlocks(procled_status_t * PL)
{   ret_code_t ret;

    if (!g_i2s_running)
    {   ret = nrfx_i2s_start(m_buffer_tx, I2S_BUFFER_SIZE, 0);
        APP_ERROR_CHECK(ret);
        g_i2s_running = true;
        fillBuffers(PL->colourVal[0], PL->colourVal[1], PL->colourVal[2]);
    }
}

ret_code_t df_led_init()
{   ret_code_t ret = NRFX_SUCCESS;
    
    uint32_t newMask;
    uint8_t i;

    // This is done in main now ->   lfclk_request();                               // request (init) a lfclk device for the app_timer configured for LED control
    // This is done in main/hardware_init now -> ret = app_timer_init();            // initialise the app_timer facility

    ret = app_timer_create(&led_timer, APP_TIMER_MODE_REPEATED, led_timer_handle);  // Create an AppTimer for the Led controller
    if (ret != NRFX_SUCCESS) goto IL_x;     
    NRFX_LOG_INFO("LED Control - app timer created");           

    ret = app_timer_start(led_timer, APP_TIMER_TICKS(LED_MS_PER_TICK), NULL);       // Start the AppTimer
    if (ret != NRFX_SUCCESS) goto IL_x; 
    app_timer_resume();                                                             // Ensure the RTC (source of AppTimer) is (re)started

    nrfx_i2s_config_t i2s_config = NRFX_I2S_DEFAULT_CONFIG;
    i2s_config.sdin_pin = I2S_SDIN_PIN;
    i2s_config.sdout_pin = I2S_SDOUT_PIN;
    i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10;                                    // < 32 MHz / 10 = 3.2 MHz.
    i2s_config.ratio     = NRF_I2S_RATIO_32X;                                       // < LRCK = MCK / 32.
    i2s_config.channels  = NRF_I2S_CHANNELS_STEREO;

    ret = nrfx_i2s_init(&i2s_config, i2s_data_handler);

    if (ret != NRFX_SUCCESS)
    goto IL_x;
    NRFX_LOG_INFO("i2s module initialised"); 

    setDefaultLEDState(true, true);                   // Set up the initial flash state

IL_x:
    return(ret);
}

void setDefaultLEDState(bool startupMode, bool clearAll)
{
    if (clearAll)
    {   memset(&procled, 0x00, sizeof(proc_led_t * NUM_PROC_LED_SLOTS));                // Clear the blink pattern array
        memset(&procled_index, 0xFF, sizeof(NUM_PROC_LED_SLOTS));                       // Clear the pattern order indexing array
    }

    if (startuptMode)       // startupMode - Make the rainbox startup flash
    {   addLEDPattern(PROC_LED_RED, LED_FLASH.LED_FLASH_ON, 2, 2);      // Red, ON, dwell=2/8S, delete=2/8S
        addLEDPattern(PROC_LED_ORANGE, LED_FLASH.LED_FLASH_ON, 2, 2);
        addLEDPattern(PROC_LED_YELLOW, LED_FLASH.LED_FLASH_ON, 2, 2);
        addLEDPattern(PROC_LED_GREEN, LED_FLASH.LED_FLASH_ON, 2, 2); 
        addLEDPattern(PROC_LED_CYAN, LED_FLASH.LED_FLASH_ON, 2, 2);  
        addLEDPattern(PROC_LED_BLUE, LED_FLASH.LED_FLASH_ON, 2, 2);  
        addLEDPattern(PROC_LED_MAGENTA, LED_FLASH.LED_FLASH_ON, 2, 2);
    }
    
    addLEDPattern(PROC_LED_GREEN, LED_FLASH.LED_FLASH_MEDIUM, 16, 0xFF);     // Green 0.5S ON, 0.5S OFF. cycle every 2S, Never timeout 
    procled_current = 0;
}

bool addLEDPattern(uint8_t * colourAry, uint32_t flashPattern, uint8_t cycleDwell, uint8_t slotTimeout)
{   // Add a new LED patter to the stack 
    uint8_t i;
    
    for (i=0;i<NUM_PROC_LED_SLOTS;i++)
    {   if (procled_index[i])
        {
            



        }



    }





}


void led_timer_handle(void * p_context)
{    proc_led_t *PL;   

    UNUSED_PARAMETER(p_context);

    for (uint8_t i; i < NUM_LEDS; i++)          // Loop around all the led control blocks
    {   
        if (procled_index[i] != 0xFF)           // find the next 
        {   
            PL =  

        
        
        NUM_PROC_LED_SLOTS
        
        lc = &LEDS[i];
        
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



