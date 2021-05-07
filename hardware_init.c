#include "hardware_init.h"

#include "nrfx_ppi.h"       
#include "nrfx_timer.h"
#include "nrfx_rng.h"
#include "nrfx_gpiote.h"
#include "nrfx_clock.h"
#include "app_button.h"
#include "app_timer.h"

#include "nrf_delay.h"      // !!!CK - just for testing txEnable output

#define NRF_LOG_MODULE_NAME hardware
// <0=> Off, <1=> Error, <2=> Warning, <3=> Info, <4=> Debug 
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#include "led_control.h"
#include "df_routines.h"
#include "console_comms.h"
#include "application.h"
#include "analog.h"

// local declarations
ret_code_t df_inputs_init(void);
ret_code_t df_relay_init(void);
ret_code_t df_bl_trigger_init(void);
ret_code_t df_pulser_init(void);

void df_nozzle_handler(uint8_t pin_no, uint8_t button_action);
void df_mode_handler(uint8_t pin_no, uint8_t button_action);
void df_pulser_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void df_psense_handler(uint8_t pin_no, uint8_t button_action);
void df_gpin_handler(uint8_t pin_no, uint8_t button_action);
void rng_handler(uint8_t rng_data);

// Variables
const nrfx_rng_config_t rng = NRFX_RNG_DEFAULT_CONFIG;          // Random number generator
hardware_t hardware;                    // Hardware shadow

ret_code_t df_hardware_init(void)
{   ret_code_t ret = NRF_SUCCESS;

    APP_ERROR_CHECK(nrfx_rng_init(&rng, rng_handler));
    NRF_LOG_INFO("RNG initialised and started");

    APP_ERROR_CHECK(df_led_init()); 
    NRF_LOG_INFO("LED Handler - Initialised"); 

    APP_ERROR_CHECK(df_inputs_init());              // Init the Nozzle handler
    NRF_LOG_INFO("Nozzle Handler - Initialised"); 

    APP_ERROR_CHECK(df_relay_init());               // Init the Ouput Relay
    NRF_LOG_INFO("Relay Control - Initialised"); 

    APP_ERROR_CHECK(df_bl_trigger_init());          // Init the Bootloader-trigger
    NRF_LOG_INFO("Bootloader Triger - Initialised"); 

    APP_ERROR_CHECK(df_pulser_init());              // Initialise the Pulser Counter
    NRF_LOG_INFO("Pulser Handler - Initialised");     

    //APP_ERROR_CHECK(df_or_sense_init());              // Initialise the Pulser Counter
    //NRF_LOG_INFO("Override Sense Handler - Initialised");     


    return ret;
}

// ***********************************************
// *** Nozzle Switch with Debounce - using the app_button API
// ***********************************************
#define BUTTON_DEBOUNCE_MS 20

#define digi_in_nozzle      0
#define digi_in_mode        1
#define digi_in_magsense    2
#define digi_in_psense      3
#define digi_in_gpin        4

// number of interrupt sources here is dependent on sdk_config.GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS

static app_button_cfg_t p_button[] = { {NOZZLE_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, df_nozzle_handler}, 
                                       {MODE_PB_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, df_mode_handler},
                                       {MAG_SENSE_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, df_mode_handler},
                                       {PSENSE_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, df_psense_handler}, 
                                       {GPIN_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, df_gpin_handler}   };
                                       

ret_code_t df_inputs_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    hardware.nozzle_pin[0] = NOZZLE_PIN;            // There is only one nozzle at the moment
    hardware.nozzle_inverted[0] = true;
    hardware.pushbutton_pin[MODE_PB_IDX] = MODE_PB_PIN;
    hardware.pushbutton_pin[MAG_PB_IDX] = MAG_SENSE_PIN;
    hardware.gpin_pin[0] = GPIN_PIN;
    hardware.gpin_inverted[0] = true;
    hardware.psense_pin[0] = PSENSE_PIN; 
    hardware.psense_inverted[0] = false;

    hardware.or_senseA[0] = OR_SENSE_1;
    hardware.or_senseB[0] = OR_SENSE_2;
    //hardware.nozzle_pin[1] = NOZZLE_2_PIN;        // Add more like this

    ret = app_button_init(p_button, sizeof(p_button) / sizeof(p_button[digi_in_nozzle]), BUTTON_DEBOUNCE_MS);
    if (ret != NRFX_SUCCESS) goto NI_x;
    
    if (app_button_is_pushed(digi_in_mode))                     // Initialisation with Mode button pressed
    {   hardware.pushbutton_time[0] = 1;                        // Kludge here.. add a value to the timer so that df_mode_handler doesn't override the factDflt LED flash pattern
        do_factory_default(true);   }                           // Do a full factory default
    else
    {   ret = app_button_enable();  }

NI_x:   
    df_nozzle_handler(NOZZLE_PIN, app_button_is_pushed(digi_in_nozzle));    // Set the intial Nozzle state

    return(ret);
}

/**
 * @brief Function to Handle Nozzle switch transitions
 *
 * This fn is linked in to the GPIOTE device for the Nozzle Switch
 */
void df_nozzle_handler(uint8_t pin_no, uint8_t button_action)
{   
    uint8_t pinval;

    for (uint8_t i=0;i<NUM_NOZZLES;i++)
    {   if (hardware.nozzle_pin[i] = pin_no)                // Find the nozzle we're looking for
        {   
            pinval = button_action ^ hardware.nozzle_inverted[i];

            hardware.nozzle[i] = pinval;             // mirror the state of the hardware to the generic hardware abstraction
            pump.nozzle = pinval;                    // also the pump view
            components.nozzle = pinval;              // and the component view

            if (pinval)  
            {   pump.pump_flags |= PUMP_FLAG_NOZZLE;                 // set the pump status bit
                components.comp_flags |= CONTROLLER_FLAG_NOZZLE;     // and component status bit
            }
            else 
            {   pump.pump_flags &= ~PUMP_FLAG_NOZZLE;    
                components.comp_flags &= ~CONTROLLER_FLAG_NOZZLE;
            }

            NRF_LOG_INFO("Nozzle [%d] State [%x]", i, pinval); 

            break;
        }
    }

}

void df_mode_handler(uint8_t pin_no, uint8_t button_action)
{   uint8_t a;
    NRF_LOG_INFO("Mode pin_no [%d] State [%x]", pin_no, button_action);                     
    
    for (uint8_t i=0;i<NUM_PBS;i++)                                                 // The mode button handler is in the application timer handler so that different hold-times can be determined
    {   if (hardware.pushbutton_pin[i] = pin_no) 
        {   hardware.pushbutton[i] = button_action; 

            if ((button_action) && (hardware.pushbutton_time[i] == 0))
            {   wipeAllLEDSlots();
                a = addLEDPattern(PROC_LED_YELLOW, LED_FLASH_MED, 120, 120, NULL, 0xFF);     // Start a 1sec flash in yellow
                loopLEDPattern(a, a);                                                       // change the linkNext pointer to itself
            }

            NRF_LOG_INFO("Mode Button [%d] State [%x]", i, button_action);             
            break;
        }
    }    
}


void df_psense_handler(uint8_t pin_no, uint8_t button_action)
{   uint8_t pinval;    
    
    for (uint8_t i=0;i<NUM_PSENSE;i++)                         // The mode button handler is in the application timer handler so that different hold-times can be determined
    {   if (hardware.psense_pin[i] = pin_no)    
        {   pinval = button_action ^ hardware.psense_inverted[i];
            hardware.psense[i] = pinval; 
            NRF_LOG_INFO("PowerSense [%d] State [%x]", i, pinval); 
            break;
        }
    }    
}


void df_gpin_handler(uint8_t pin_no, uint8_t button_action)
{
    uint8_t pinval;    

    for (uint8_t i=0;i<NUM_GPIS;i++)                         // The mode button handler is in the application timer handler so that different hold-times can be determined
    {   if (hardware.gpin_pin[i] = pin_no) 
        {   pinval = button_action ^ hardware.gpin_inverted[i]; 
            hardware.gpin[i] = pinval; 
            NRF_LOG_INFO("GPIn [%d] State [%x]", i, pinval); 
            break;
        }
    }    
}


// ***********************************************
// *** Bootloader-Trigger Control - simple GPIO output
// ***********************************************

/**
 * @brief Function to initialise tre Bootloader-Trigger output
 * The Bootloader-trigger is an output pin that is tied to the Processor's Reset pin
 */
ret_code_t df_bl_trigger_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    nrf_gpio_pin_set(BOOTLOADER_PIN);            // Turn it off before config so we don't get a RESET pulse
    nrf_gpio_cfg_output(BOOTLOADER_PIN);         // Configure the Bootloader
    nrf_gpio_pin_set(BOOTLOADER_PIN);            // Turn off the Bootloader trigger (i.e. self-reset pin)

    return(ret);
}

void trigger_bootloader(void)
{   nrf_gpio_pin_clear(BOOTLOADER_PIN);  }

// ***********************************************
// *** Pump control relay config - simple GPIO output
// ***********************************************
ret_code_t df_relay_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    // Relay can be normal GPIO
    nrf_gpio_cfg_output(RELAY_PIN);         // Configure the Relay output
    df_relay_change(RELAY_PIN, 0);          // Start with it turned off    

    return(ret);
}

void df_relay_change(uint32_t channel, uint8_t state)
{   
    uint8_t chan = channel;
    if (channel == 0) { chan = RELAY_PIN;   }

    if (state)
    {   nrf_gpio_pin_set(chan);   }
    else
    {   nrf_gpio_pin_clear(chan);     }
}


// ***********************************************
// ***** Pulser declarations & configuration *****
// ***********************************************
nrfx_timer_t pulse_counter = NRFX_TIMER_INSTANCE(DF_PULSER_TIMER_INST);           // create a counter/timer device for counting pulses (channel 1/3) - Leave Timer0 available for a softdevice

/**
* @brief Part-function for setting up the Pulser timer channel - see df_pulser_setup()
*/
void df_pulser_timer_event_handler(nrf_timer_event_t event_type, void *p_context)
//void df_pulser_timer_event_handler(void)
{
//   NRF_LOG_INFO("Pulser Counter event handler"); 
}

ret_code_t pulser_timer_setup(void)
{   ret_code_t ret = NRFX_SUCCESS;

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;                          // config packet for timer init    
    //timer_cfg.mode = NRF_TIMER_MODE_LOW_POWER_COUNTER;
    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    
    //ret = nrfx_timer_init(&pulse_counter, &timer_cfg, (nrfx_timer_event_handler_t)stub); // send to init
    ret = nrfx_timer_init(&pulse_counter, &timer_cfg, df_pulser_timer_event_handler);      // send to init    
    if (ret != NRFX_SUCCESS) goto TS_x;

    nrfx_timer_enable(&pulse_counter); 
    nrfx_timer_resume(&pulse_counter);

TS_x:
    return(ret);
}


//void df_pulser_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {}
/**
* @brief Part-function for setting up the Pulser gpiote channel - see df_pulser_setup()
*/

void df_gpiote_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//   nrfx_timer_increment(&pulse_counter);
//    NRF_LOG_INFO("Pulser GPIOTE event handler at pin[%d], action[%x]", pin, action); 
}

ret_code_t pulser_gpiote_setup()                         
{   ret_code_t ret = NRFX_SUCCESS;

    //nrfx_gpiote_in_config_t i_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);  // config packet for gpiote channel .. sense on falling edge
    //nrfx_gpiote_in_config_t i_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);    // config packet for gpiote channel .. sense on both edges
    nrfx_gpiote_in_config_t i_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
    i_config.pull = NRF_GPIO_PIN_PULLUP;                                            // gpiote config values .. set pin pullup
	
    //ret = nrfx_gpiote_in_init(PULSER_PIN, &i_config, (nrfx_gpiote_evt_handler_t)stub);    // configure PULSER_PIN, event handler callback is set to NULL to keep it from triggering unecessary CB calls.
    ret = nrfx_gpiote_in_init(PULSER_PIN, &i_config, df_gpiote_event_handler);    // configure PULSER_PIN, event handler callback is set to NULL to keep it from triggering unecessary CB calls.
    if (ret != NRFX_SUCCESS) goto GS_x;
    
    nrfx_gpiote_in_event_enable(PULSER_PIN, true);                                  // and enable the GPIOTE channel interrupt

GS_x:
    return(ret);
}



static nrf_ppi_channel_t ppi_pulser;
/**
* @brief Part-function for setting up the Pulser ppi channel - see df_pulser_setup()
*/
ret_code_t pulser_ppi_setup(void)
{   ret_code_t ret = NRFX_SUCCESS;

    ret = nrfx_ppi_channel_alloc(&ppi_pulser);
    if (ret != NRFX_SUCCESS) goto PS_x;
    
    ret = nrfx_ppi_channel_assign(ppi_pulser, nrfx_gpiote_in_event_addr_get(PULSER_PIN), nrfx_timer_task_address_get(&pulse_counter, NRF_TIMER_TASK_COUNT));
    if (ret != NRFX_SUCCESS) goto PS_x;
    
    nrfx_ppi_channel_enable(ppi_pulser);

PS_x:
    return(ret);
}

/**
 * @brief Function for setting up the Pulser input channel
 *
 * The Pulser channel uses a GPIOTE channel to capture the hardware transition
 * this sends an event to a timer/counter channel over the PPI interface
 * to count the pulses without calling the processor for enhanced reliability
 */
ret_code_t df_pulser_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    ret = pulser_gpiote_setup();
    if (ret != NRFX_SUCCESS) goto PI_x;

    ret = pulser_timer_setup();
    if (ret != NRFX_SUCCESS) goto PI_x;

    ret = pulser_ppi_setup();
    if (ret != NRFX_SUCCESS) goto PI_x;

PI_x:
    return(ret);
}


/**
 * @brief System Clock intialisation
 *
 * General-purpose system clock, for LED_Control and NV Storage
 * 
 */
void clock_handler(nrfx_clock_evt_type_t event)
{      return;  }

void clock_init(void)
{   nrfx_err_t ret = nrfx_clock_init(clock_handler);        // Initialise the clock
    APP_ERROR_CHECK(ret);
    nrfx_clock_enable();
    nrfx_clock_lfclk_start(); 

    while (!nrfx_clock_lfclk_is_running()) {;}

    NRF_LOG_INFO("GP Clock (NV, LED) - Initialised"); 
}

/**@brief   Initialize the App timer. */
void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("GP Timer (NV, LED) - Initialised"); 
}


/**
* @brief callback function for the RNG
*/
void rng_handler(uint8_t rng_data)
{
    nrfx_rng_stop();
    ddpc.my_rnd = rng_data;

    if (ddpc.nv_immediate.dev_address == DISCOVERY_DFLT_ADDR)
    {  con_comms.discovery_holdoff = ((ddpc.my_rnd >> 5) + 1);        }

    NRF_LOG_DEBUG("Address Discovery holdoff changed to [%d]", con_comms.discovery_holdoff);

}

void update_rng(void)
{   nrfx_rng_start();    }


/**
* @brief extends the std nrf_gpio_cfg_output() to allow drive strength to be defined
*/
void df_gpio_cfg_output(uint32_t pin_number, uint32_t drive_type)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}

uint32_t get_sys_ms(void)
{   return((app_timer_cnt_get() * ( (APP_TIMER_CONFIG_RTC_FREQUENCY + 1 ) * 1000 ) / APP_TIMER_CLOCK_FREQ));   }