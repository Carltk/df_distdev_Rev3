#include "app_timer.h"
#include "nrfx_clock.h"
#include "nrfx_i2s.h"

#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME led_control
// <0=> Off, <1=> Error, <2=> Warning, <3=> Info, <4=> Debug 
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "df_routines.h"
#include "led_control.h"
#include "hardware_init.h"


#define LED_MS_PER_TICK 125                         // LED flash timer resolution = 125mS = 1/8 second
#define RESET_BITS      6                           // Enough reset bits to output >50uS at 10uS per 32-bit buffer
#define I2S_BUFFER_SIZE 3 + RESET_BITS              // i2s buffer is 3x colour channels + reset blocks

// *** Global Variables ***

bool i2s_running;
bool i2s_is_dark;
#define I2S_ERR_TH  10
uint8_t i2s_errcount;

static nrfx_i2s_config_t i2s_config = NRFX_I2S_DEFAULT_CONFIG;     // i2s definition
APP_TIMER_DEF(led_timer);                                   // Timer definition

proc_led_t procled[NUM_PROC_LED_SLOTS];      // An array of proc_led control slots
uint8_t procled_current;                     // Variable to hold currently processed procled array index

uint32_t m_buffer_tx[I2S_BUFFER_SIZE];
uint32_t m_buffer_rx[I2S_BUFFER_SIZE];

// *** static function prototypes ***
static void i2s_data_handler(nrfx_i2s_buffers_t const * p_released, uint32_t status);
static uint32_t calcChannelVal(uint8_t level);
static void fillBuffers(uint8_t RVal, uint8_t GVal, uint8_t BVal, bool ledState);
static ret_code_t sendLEDBlocks(proc_led_t * PL, bool ledState);
static void setStartupLEDState(void);
static uint8_t roll_mask(uint32_t * mask);
static void led_timer_handle(void * p_context);

// *** global function prototypes in header file ***

// Colour data structures for the multi-colour LED
#define dimmer 2        // Brightscale MAX=128 but scale down to reduce brightness !!!!NEVER set this to 0!!!!

const uint8_t PROC_LED_RED[]     = {(uint8_t)(255/dimmer),0,0};       // Red used for Error status
const uint8_t PROC_LED_GREEN[]   = {0,(uint8_t)(255/dimmer),0};       // Green used for OK status    
const uint8_t PROC_LED_YELLOW_GREEN[] = {(uint8_t)(194/dimmer),(uint8_t)(194/dimmer),0};       // Yellow- Green 
const uint8_t PROC_LED_ORANGE[]  = {(uint8_t)(255/dimmer),(uint8_t)(128/dimmer),0};     // Orange used for "Transaction" indication
const uint8_t PROC_LED_YELLOW[]  = {(uint8_t)(255/dimmer),(uint8_t)(255/dimmer),0};     // Yellow used for pushbutton timing indication
const uint8_t PROC_LED_CYAN[]    = {0,(uint8_t)(255/dimmer),(uint8_t)(255/dimmer)};     // Cyan used for "Comms" indication
const uint8_t PROC_LED_BLUE[]    = {0,0,(uint8_t)(255/dimmer)};       
const uint8_t PROC_LED_MAGENTA[] = {(uint8_t)(255/dimmer),0,(uint8_t)(255/dimmer)};     // Magenta used for "Memory" indication
const uint8_t PROC_LED_VIOLET[] =  {(uint8_t)(143/dimmer),0,(uint8_t)(255/dimmer)};     // Magenta used for "Memory" indication
const uint8_t PROC_LED_WHITE[]   = {(uint8_t)(255/dimmer),(uint8_t)(255/dimmer),(uint8_t)(255/dimmer)};   // White


// *** Global Functions ***
ret_code_t df_led_init(void)
{   ret_code_t ret = NRFX_SUCCESS;
    
    uint32_t newMask;
    uint8_t i;

    i2s_running = false;
    i2s_is_dark = false;
    i2s_errcount = 0;

    i2s_config.sdin_pin = I2S_SDIN_PIN;
    i2s_config.sdout_pin = I2S_SDOUT_PIN;
    i2s_config.sck_pin = I2S_SCK_PIN; 
    i2s_config.lrck_pin = I2S_LRCK_PIN;
    i2s_config.mck_pin = I2S_MCK_PIN; 
    i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10;                                    // < 32 MHz / 10 = 3.2 MHz.
    i2s_config.ratio     = NRF_I2S_RATIO_32X;                                       // < LRCK = MCK / 32.
    i2s_config.channels  = NRF_I2S_CHANNELS_STEREO;

    if (ret != NRFX_SUCCESS)
    goto IL_x;
//    NRF_LOG_INFO("i2s module initialised"); 

    ret = app_timer_create(&led_timer, APP_TIMER_MODE_REPEATED, led_timer_handle);  // Create an AppTimer for the Led controller
    if (ret != NRFX_SUCCESS) goto IL_x;     
//    NRF_LOG_INFO("LED Control - app timer created");           

    ret = app_timer_start(led_timer, APP_TIMER_TICKS(LED_MS_PER_TICK), NULL);       // Start the AppTimer
    if (ret != NRFX_SUCCESS) goto IL_x; 

    setStartupLEDState();                   // Set up the initial flash state
    ret = nrfx_i2s_init(&i2s_config, i2s_data_handler);

IL_x:
    return(ret);
}

void ledNewValPoke(uint8_t Idx)
{   if (Idx < NUM_PROC_LED_SLOTS) procled_current = Idx;                    // set the index to be used
    //app_timer_stop(led_timer);                                              // Stop the timer
    led_timer_handle(NULL);                                                 // Call the timer handler to load new values
    //app_timer_start(led_timer, APP_TIMER_TICKS(LED_MS_PER_TICK), NULL);     // (re)Start the timer
}

void wipeAllLEDSlots(void)
{   memset(&procled, 0x00, (sizeof(proc_led_t) * NUM_PROC_LED_SLOTS));  }

uint8_t addLEDPattern(const uint8_t * colourAry, uint32_t flashPattern, uint8_t cycleDwell, uint8_t slotTimeout, void * p_func, uint8_t link)
{   // Add a new LED pattern to a blank slot in the stack 
    uint8_t i;
    uint8_t thisSlot = 0xFF;
    proc_led_t * PL;
    
    for (i=0;i<NUM_PROC_LED_SLOTS;i++)  
    {   if (!procled[i].inUse)                  // find and unused slot
        {   PL = &procled[i];
            memcpy(&PL->colourVal, colourAry, NUM_COLOURS);

            PL->flashPattern = flashPattern;
            PL->status.rolledFlashPattern = flashPattern;
            PL->cycleDwell = cycleDwell;      
            PL->status.cycleDwellCount = cycleDwell;       
            PL->slotTimeout= slotTimeout;           
            PL->status.slotTimeLeft = slotTimeout;         
            PL->p_run_after_dwell = p_func;
            PL->linkNext = link;              
            PL->inUse = true;
            thisSlot = i;                       // return the index of this slot
            break;
        }
    }

//    NRF_LOG_INFO("leds: added pattern at [%d]", thisSlot);                 

    return(thisSlot);
}

void loopLEDPattern(uint8_t topIdx, uint8_t bottomIdx)
{   procled[topIdx].linkNext = bottomIdx;    
    procled_current = bottomIdx;
}

uint8_t LEDSlot(proc_led_t * PL)
{   return(slotFromObject(PL, procled, sizeof(proc_led_t)));    }

uint8_t clearLEDSlot(uint8_t Idx)
{   if (Idx >= NUM_PROC_LED_SLOTS) return(0xFF);    // Make sure index isn't out of range

    proc_led_t * PL = &procled[Idx];                // Make pointer to the procled structure
    uint8_t linkNext = PL->linkNext;                // Save the Slot's linkNext attribute

    memset(PL, 0x00, sizeof(proc_led_t));           // Wipe all bytes in the Slot
    
    if (linkNext == LEDSlot(PL)) linkNext = 0xFF;   // Don't pass on the linkNext if it was pointing to itself
    return(linkNext);
}


// **** Static Functions - internal to this Module ******


// This is the I2S data handler - all data exchange related to the I2S transfers is done here.
static void i2s_data_handler(nrfx_i2s_buffers_t const * p_released, uint32_t status)
{   
    if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
    {   nrfx_i2s_stop();   
        i2s_running = false;
        i2s_errcount = 0;
        //NRF_LOG_INFO("leds: i2s stop");
    }  
}

// because of the way i2s encodes signals (i.e. Left/Right)
static uint32_t calcChannelVal(uint8_t level)
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

static void fillBuffers(uint8_t RVal, uint8_t GVal, uint8_t BVal, bool ledState)
{   uint8_t i;

    if (ledState)
    {   m_buffer_tx[0] = calcChannelVal(GVal);               // ledState ON .. fill with colours from the procLed pattern
        m_buffer_tx[1] = calcChannelVal(RVal);
        m_buffer_tx[2] = calcChannelVal(BVal);
        i2s_is_dark = false;
        //NRF_LOG_INFO("leds: sending [R%d,G%d,B%d]", RVal, GVal, BVal);    
    }
    else
    {   for (uint8_t i=0; i<NUM_COLOURS;i++)                // ledState OFF .. fill all 3 colour values with 0
        {   m_buffer_tx[i] =  0x88888888;   
            i2s_is_dark = true;
        }
        //NRF_LOG_INFO("leds: sending dark");  
    }

    memset(&m_buffer_tx[3], 0x00, (4 * RESET_BITS));    // Clear 6 buffers from [3] onwards to make the >50uS reset pulse
    
}

static ret_code_t sendLEDBlocks(proc_led_t * PL, bool ledState)
{   
    ret_code_t ret = 99;
    
    nrfx_i2s_buffers_t m_buffer;
    m_buffer.p_tx_buffer = m_buffer_tx;
    m_buffer.p_rx_buffer = m_buffer_rx;

    if (!i2s_running)
    {   fillBuffers(PL->colourVal[0], PL->colourVal[1], PL->colourVal[2], ledState);
        ret = nrfx_i2s_start(&m_buffer, I2S_BUFFER_SIZE, 0);
        nrf_delay_ms(1);
        //NRF_LOG_INFO("leds: i2s Start");                 
        i2s_running = true;
    }
    else
    {   i2s_errcount += 1;
        if (i2s_errcount >= I2S_ERR_TH)
        {   i2s_running = false;
            i2s_errcount = 0;
        }
    }

    return(ret);
}

static void setLEDSystemStatus()
{   uint8_t a, c;
    
    wipeAllLEDSlots();   
    // system status has 3 flashes: 1 - system state; 2 - comms state; 3 - pump state
    // Add patterns backwards
    a = addLEDPattern(PROC_LED_WHITE, LED_FLASH_OFF, 8, 16, setLEDSystemStatus, 0xFF);   // just a place-holder (1/8 sec) for the reload routine
    c = makePumpStatusFlashes(a, pump.pump_state, true);
    c = makeCommsStatusFlashes(c, con_comms.comms_state, true);
    c = makeSysStatusFlashes(c, ddpc, true);
    loopLEDPattern(a, c);
}

static void setStartupLEDState(void)
{   uint8_t a, c;

    wipeAllLEDSlots();

    // add patterns backwards to capture the nextLink
    a = addLEDPattern(PROC_LED_WHITE, LED_FLASH_OFF, 1, 1, NULL, 0xFF);   // Blank 
    c = addLEDPattern(PROC_LED_WHITE, LED_FLASH_ON, 1, 1, NULL, a);   
    c = addLEDPattern(PROC_LED_CYAN, LED_FLASH_ON, 1, 1, NULL, c);  
    c = addLEDPattern(PROC_LED_BLUE, LED_FLASH_ON, 1, 1, NULL, c);  
    c = addLEDPattern(PROC_LED_GREEN, LED_FLASH_ON, 1, 1, NULL, c); 
    c = addLEDPattern(PROC_LED_YELLOW, LED_FLASH_ON, 1, 1, NULL, c);
    c = addLEDPattern(PROC_LED_ORANGE, LED_FLASH_ON, 1, 1, NULL, c);
    c = addLEDPattern(PROC_LED_RED, LED_FLASH_ON, 1, 1, NULL, c);      // Red, ON, dwell=2/8S, delete=2/8S
    loopLEDPattern(a, c);
}

void setIdentLEDState(void)
{   uint8_t a, c;
    wipeAllLEDSlots();
                
    a = addLEDPattern(PROC_LED_RED, LED_ESCALATE, 32, 64, NULL, 0xFF);
    c = addLEDPattern(PROC_LED_GREEN, LED_ESCALATE, 32, 64, NULL, a);
    c = addLEDPattern(PROC_LED_BLUE, LED_ESCALATE, 32, 64, NULL, c);
    loopLEDPattern(a, c);
}

static uint8_t roll_mask(uint32_t * mask)
{   uint8_t shifted_bit;

    shifted_bit = (char)(*mask & 0x00000001);               // get the lsbit
    *mask >>= 1;                                     // shift down the mask
    *mask |= (shifted_bit ? 0x80000000 : 0L);        // roll the shifted bit up to the msbit

    return(shifted_bit);
}

static void led_timer_handle(void * p_context)
{   proc_led_t *PL;   
    uint8_t i;
    uint8_t unusedCnt=0;
    uint8_t ledState;

    UNUSED_PARAMETER(p_context);

    for (i=procled_current;i<NUM_PROC_LED_SLOTS;i++)
    {
        if (procled[i].inUse)    
        {   PL = &procled[i];

            ledState = roll_mask(&PL->status.rolledFlashPattern);       // roll the flash pattern

            if (ledState || (ledState==0 && !i2s_is_dark))
            {   sendLEDBlocks(PL, ledState); }

            if (PL->status.slotTimeLeft != 0xFF)                     // Check to see if this is an expireable slot
            {   PL->status.slotTimeLeft -= 1;                        // decrement its counter
                if (PL->status.slotTimeLeft == 0)                    // if it reaches 0
                {   //NRF_LOG_INFO("leds: slot timeout on idx [%d]", i);                 
                    procled_current = clearLEDSlot(i);             // delete the slot
                    return;       
                }
            }

            PL->status.cycleDwellCount -= 1;                        // Check to see if we've timed out this colour
            if (PL->status.cycleDwellCount == 0)                    // yes?
            {   //NRF_LOG_INFO("leds: cycle dwell timeout on idx [%d]", i);                 
                PL->status.cycleDwellCount = PL->cycleDwell;        // reset the cycle dwell counter
                
                if (PL->linkNext != 0xFF)                           // see if we have a next-slot-link
                {   procled_current = PL->linkNext; }               // if so, set up the jump for next timer handler
                else
                {   procled_current = i+1;  }                       // No next-slot-link .. set up for next slot in the stack

                if (PL->p_run_after_dwell != NULL)                  // if there's a dwell-timeout function
                {   (*PL->p_run_after_dwell)();   }                 //  .. run it here
            }
            
            return;       
        }
        else
        {   unusedCnt+=1;   }
    }
   
    if (i >= NUM_PROC_LED_SLOTS) procled_current = 0;
    if (procled_current >= NUM_PROC_LED_SLOTS) procled_current = 0;
    if (unusedCnt >= NUM_PROC_LED_SLOTS) 
    {   setLEDSystemStatus();             }   // if there are no active slots, reinit to OK green flash
}

uint8_t makeSysStatusFlashes(uint8_t idx, ddpc_t dd, bool single)
{   uint8_t c;
    uint8_t * colourAry;
    
    colourAry = (char*)PROC_LED_GREEN;                        // default to Green

    if (dd.nv_panic.flash_erase > 10000)                // Excessive Flash erases 
    {   colourAry = (char*)PROC_LED_RED;   }                  // .. Red flash
    else if (dd.curr_temp > OVER_TEMP_THRESHOLD)        // Too hot
    {   colourAry = (char*)PROC_LED_MAGENTA;   }              // .. Magenta Flash
    else if (dd.nv_immediate.dev_address == 0xFFFE)     // Factory Default Address
    {   colourAry = (char*)PROC_LED_ORANGE;   }               // .. Orange Flash
    else if ((dd.nv_immediate.dev_address & 0xFF00) == 0xFF00)     // Temporary Address
    {   colourAry = (char*)PROC_LED_YELLOW;   }               // .. Yellow Flash

    c = addLEDPattern(colourAry, LED_6ON_2OFF, 8, 16, NULL, idx);

    if (single == false)
    {   c = addLEDPattern(PROC_LED_GREEN, LED_FLASH_ON, 8, 16, NULL, c);     }

    return(c);
}

uint8_t makeCommsStatusFlashes(uint8_t idx, comms_state_t cs, bool single)
{   uint8_t c;

    // 2 modes of operation .. system status (2 lines), running mode (1 line)
    switch (cs)
    {   case COMMS_INIT:
        case COMMS_DISCONNECTED:
            c = addLEDPattern(PROC_LED_WHITE, LED_6ON_2OFF, 8, 16, NULL, idx);      // .5Sec on, 0.5Sec off (twice)
            break;
        case COMMS_AUTOBAUD:
            c = addLEDPattern(PROC_LED_MAGENTA, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case COMMS_VISIBLE:
            c = addLEDPattern(PROC_LED_YELLOW, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case COMMS_ADOPTING: 
            c = addLEDPattern(PROC_LED_ORANGE, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case COMMS_FOSTERED:
            c = addLEDPattern(PROC_LED_YELLOW_GREEN, LED_6ON_2OFF, 8, 16, NULL, idx);
            break;
        case COMMS_ONLINE:
            c = addLEDPattern(PROC_LED_GREEN, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case COMMS_ERROR:
        default:
            c = addLEDPattern(PROC_LED_RED, LED_6ON_2OFF, 8, 16, NULL, idx);
            break;
    }

    if (single == false)
    {   c = addLEDPattern(PROC_LED_BLUE, LED_FLASH_ON, 8, 16, NULL, c);     }

    return(c);
}

uint8_t makePumpStatusFlashes(uint8_t idx, pump_state_t ps, bool single)
{   uint8_t c;

    // 2 modes of operation .. system status (2 lines), running mode (1 line)
    switch (pump.pump_state & 0x0F)
    {   case PUMP_STATE_IDLE:
            c = addLEDPattern(PROC_LED_GREEN, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case PUMP_STATE_CALL:
            c = addLEDPattern(PROC_LED_ORANGE, LED_6ON_2OFF, 8, 16, NULL, idx);
            break;
        case PUMP_STATE_AUTH:
            c = addLEDPattern(PROC_LED_CYAN, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;
        case PUMP_STATE_BUSY:
            c = addLEDPattern(PROC_LED_BLUE, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;        
        case PUMP_STATE_OR_LEAK:
            c = addLEDPattern(PROC_LED_VIOLET, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;        
        case PUMP_STATE_OR_TRANS:
            c = addLEDPattern(PROC_LED_MAGENTA, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;        
        case PUMP_STATE_PAY:
            c = addLEDPattern(PROC_LED_YELLOW, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;        
        case PUMP_STATE_STOP:
            c = addLEDPattern(PROC_LED_ORANGE, LED_6ON_2OFF, 8, 16, NULL, idx);    
            break;        
        case PUMP_STATE_ERROR:
        default:
            c = addLEDPattern(PROC_LED_RED, LED_6ON_2OFF, 8, 16, NULL, idx);
            break;
    }

    if (single == false)
    {   c = addLEDPattern(PROC_LED_VIOLET, LED_6ON_2OFF, 8, 16, NULL, c);     }

    return(c);
}

