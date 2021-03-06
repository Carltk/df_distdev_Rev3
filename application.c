
#include "application.h"
#include "nrfx_temp.h"
#include "nrfx_log.h"

#include "app_button.h"

#include "app_timer.h"
#include "console_comms.h"
#include "hardware_init.h"
#include "led_control.h"
#include "nv_store.h"

// variables  
APP_TIMER_DEF(app_timer);                                       // Timer definition

const nrfx_temp_config_t tmp_config = NRFX_TEMP_DEFAULT_CONFIG; // On-chip Temperature monitor

ddpc_t ddpc = DF_DDPC_DFLT;
pump_controller_t pump = DF_PUMP_DFLT;
comp_controller_t components = DF_COMP_DFLT;
app_state_t app_state = DF_APP_STATE_DLFT;
static uint8_t console_port_check = 0;

// function prototypes
void temp_handler(int raw_temp);
ret_code_t init_temp(void);
void app_timer_handle(void * p_context);
void handle_push_button(void);
void pump_state_machine(pump_controller_t *this_pump, pump_caller caller);
void pump_clear_for_transaction(pump_controller_t *this_pump);
void ShowFullSysStatus(void);

#define APP_MS_PER_TICK  100

// function definitions
ret_code_t application_init(void)
{
    ret_code_t ret = NRFX_SUCCESS;
    
    ret = app_timer_create(&app_timer, APP_TIMER_MODE_REPEATED, app_timer_handle);      // Create an AppTimer for the application 
    if (ret != NRFX_SUCCESS) goto AI_x;     

    ret = app_timer_start(app_timer, APP_TIMER_TICKS(APP_MS_PER_TICK), &app_state);     // Start the AppTimer for the application
    if (ret != NRFX_SUCCESS) goto AI_x; 
    //app_timer_resume();                                                                 // Ensure the RTC (source of AppTimer) is (re)started
    NRFX_LOG_INFO("App timer created and Started");           

    ret = init_temp();                                                                  // start temperature monitoring
    if (ret != NRFX_SUCCESS) goto AI_x;     
    NRFX_LOG_INFO("Temperature module initialised");           

    update_rng();
    
    pump_state_machine(&pump, NORMAL);
    NRFX_LOG_INFO("Pump State Machine started");           

AI_x:
    return(ret);
}

void app_timer_handle(void * p_context)
{
    auto uint8_t i;
    auto uint32_t l;
    app_state_t *as;

    as = p_context;

    for (i=0;i<NUM_APP_STATES;i++)
    {
        as->current_count[i] -= 1;                              // dec the countdown value

        if (as->current_count[i] <= 0)
        {   as->current_count[i] = as->reset_value[i];          // reset the counter

            switch (i)                              
            {   case APP_STATE_100MS:
                    if (hardware.pushbutton[0])                 // If the Mode button is pressed
                    {   hardware.pushbutton_time[0] += 1;   }   // .. advance the count

                    l = nrfx_timer_capture(&pulse_counter, DF_PULSER_TIMER_INST);   // Check the pulse counter
                    if (l != pump.curr_pulses)
                    {   
                        pump.transaction_count = pump.transaction_count + (l - pump.curr_pulses);
                        pump.curr_pulses = l;                       // Update the pulse counts
                        
                        pump.pump_flags |= PUMP_FLAG_PULSES;     // Flag more pulses
                        //NRFX_LOG_INFO("Counter is now [%ld]", l); 
                    }
                        
                    pump_state_machine(&pump, NORMAL);

                    break;
                case APP_STATE_1S:
                    if (hardware.pushbutton_time[0])            // if the mode button has a time-count
                    {   handle_push_button();   }               // Check the current state for release / actions etc

                    if (pump.timer >= 0)   
                    {   pump.timer--;
                        if (pump.timer == 0) 
                        {   pump_state_machine(&pump, TIMEOUT);  
                            NRFX_LOG_INFO("Pump Timer Timed Out"); 
                        }
                    }

                    if (flash_control.delete_next)
                    {   delete_all_process();   }

                    if (flash_control.do_immediate_save)
                    {   StoreImmediate();       }
                    
                    if (hardware.relay_test[0] > 0)                     // Are we in relay test mode ?
                    {   hardware.relay_test[0] -= 1;                        // if so .. time it down
                        if (hardware.relay_test[0] == 0)                // Once it's back to 0
                        {   df_relay_change(hardware.relay[0], 0);  }   // .. turn the relay off
                    }
                    
                    break;
                case APP_STATE_10S:                             
                    if (flash_control.gc_required)
                    {   do_fds_gc();    
                        flash_control.gc_required = false;
                    }
                    else
                    {   check_gc_required(true);    }

                    break;
                case APP_STATE_1M:
                    update_temp();                              // Update the temperature reading                    
                    ddpc.nv_panic.uptime_mins += 1;
                    flash_control.need_panic_save = true;

                    break;
                default:
                    break;
            }

            comms_timer_tick(i);                // Call the rolling comms timer function
        }
    }
}

void handle_push_button(void)
{
    if (hardware.pushbutton[0] == 0)        // perform the actions when the button is released
    {   
        NRFX_LOG_INFO("Button pressed for %d seconds", ((hardware.pushbutton_time[0] / 10) + 1));                   
        
        if (hardware.pushbutton_time[0] > 80)
        {
            switch ((hardware.pushbutton_time[0] / 10) + 1)  // time is 100mS slices .. make seconds
            {
                case 0: case 1:                              // 0-2 seconds
                    ShowFullSysStatus();                     // Make the system status stack
                    break;
                case 2: case 3: case 4:                     // 2-5 sec            
                    pump_clear_for_transaction(&pump);
                    break;
                case 5: case 6: case 7: case 8: case 9:     // 5-10 sec            
                    do_factory_default(false);
                    break;
                default:                                    // more than 10 seconds
                    trigger_bootloader();
                    break;
            }
        }
        
        hardware.pushbutton_time[0] = 0;                            // Clear the timer/counter once processing is finished
    }
}


void ShowFullSysStatus(void)
{   uint8_t a, c;
    
    wipeAllLEDSlots();

    // explode the hardware status block into LED flashes

    a = addLEDPattern(PROC_LED_VIOLET, LED_DOUBLE_FLASH, 16, 32, NULL, 0xFF);    
    c = addLEDPattern(PROC_LED_ORANGE, LED_SINGLE_FLASH, 8, 16, NULL, c);
    c = makeCommsStatusFlashes(c, con_comms.comms_state, false);
    c = makePumpStatusFlashes(c, pump.pump_state, false);

    loopLEDPattern(a, c);       // Write the loop-back index into the first pattern block & init the pattern
}

/**
* @brief Function for initializing the on-chip temperature monitor
*
* @retval NRFX_SUCCESS                   Driver was successfully initialized.
* @retval NRFX_ERROR_ALREADY_INITIALIZED Driver was already initialized.
*/
ret_code_t init_temp(void)
{
    nrfx_err_t ret = NRFX_ERROR_NOT_SUPPORTED;
    ret = nrfx_temp_init(&tmp_config, temp_handler);
    return(ret);
}

/** @brief Interrupt handler function for temperature readings */
void temp_handler(int raw_temp)
{   ddpc.curr_temp = nrfx_temp_calculate(raw_temp);
    NRFX_LOG_INFO("Temperature is [%d]", ddpc.curr_temp);
}

/**
* @brief Function for requesting a temperature reading
*   It should be called periodically e.g. from the system tick timer
*
* @retval void
*/
void update_temp(void)
{   nrfx_temp_measure();    }

void set_address_default(void)
{
    ddpc.nv_immediate.dev_address = DISCOVERY_DFLT_ADDR;
    flash_control.do_immediate_save = true;

    con_comms.discovery_temp_addr = DISCOVERY_DFLT_ADDR;
}

/**
* @brief Function for clearing Device memory back to Factory default settings
*/
void do_factory_default(bool SuperDflt)
{   auto int a;
    
    wipeAllLEDSlots();                                                                  // LED Control .. clear current pattern
    a = addLEDPattern(PROC_LED_VIOLET, LED_FLASH_FAST, 40, 40, NULL, 0xFF);         // LED Control .. Start a fast flash in violet
    loopLEDPattern(a, a);                                                               // LED Control .. change the linkNext pointer to itself    

    update_rng();   

    NRFX_LOG_INFO("*** Factory Default Done (All-delete=%d) ***", SuperDflt);

    if (SuperDflt)
    {   delete_all_begin(); }
    else
    {   set_address_default();   }

    app_button_enable();

    //flash_control.gc_required = true;
}

void pump_action_cmd(pump_controller_t *this_pump, uint8_t cmd)
{
    NRFX_LOG_INFO("Received a Pump Action command of [%x]", cmd);    
    switch (cmd)
    {   case PUMP_CMD_AUTHORISE:    
            if ((this_pump->pump_state < PUMP_STATE_BUSY) || (this_pump->pump_state == PUMP_STATE_OR_LEAK) )
            {   pump_change_state(this_pump, PUMP_STATE_AUTH, 30);  }
            break;
        case PUMP_CMD_STOP:
            pump_change_state(this_pump, PUMP_STATE_PAY, 20);
            break;
        case PUMP_CMD_PAID:                                         // Pay-off the pump
            pump_change_state(this_pump, PUMP_STATE_STOP, 0);       // and go to stop state
            break;    
        default:
            break;
    }
}

void pump_change_state(pump_controller_t *this_pump, uint8_t new_state, uint32_t timeout)
{   this_pump->pump_state = new_state;   
    this_pump->timer = timeout;    
    NRFX_LOG_INFO("Pump State changed to %x", new_state);  
}

void pump_clear_for_transaction(pump_controller_t *this_pump)
{   
    this_pump->transaction_count = 0;        // Wipe the current pulse count
}

void pump_start_transaction(pump_controller_t *this_pump)
{
    pump_clear_for_transaction(this_pump);
    df_relay_change(RELAY_PIN, 1);
}

void pump_stop_transaction(pump_controller_t *this_pump)
{
    df_relay_change(RELAY_PIN, 0);
}


void pump_state_machine(pump_controller_t *this_pump, pump_caller caller)
{
      switch (this_pump->pump_state & 0x0F)  
      {
        case PUMP_STATE_IDLE:                                   // Pump is Idle .. no transaction taking place
            if (this_pump->nozzle)                                   // Nozzle lifted .. go to call state
            {   pump_change_state(this_pump, PUMP_STATE_CALL, 0);     
                break;
            }
            
            if (this_pump->transaction_count)                           // If there are new pulses
            {   pump_change_state(this_pump, PUMP_STATE_OR_LEAK, 0);  } // Change to Override state
            
            break;            
        case PUMP_STATE_CALL:                               // Pump Nozzle has been picked up 
            if (this_pump->nozzle == 0)                          // Nozzle down .. go to idle state
            {   pump_change_state(this_pump, PUMP_STATE_IDLE, 0);       
                break;
            }
            
            if (this_pump->transaction_count)                            // If there are new pulses
            {   pump_change_state(this_pump, PUMP_STATE_OR_TRANS, 0);  } // Change to Override state

            break;            
        case PUMP_STATE_AUTH:                               // Console has authorised a Transaction
            if (this_pump->nozzle)  
            {   pump_start_transaction(this_pump);                  // Clear any buffers & registers for transaction start
                pump_change_state(this_pump, PUMP_STATE_BUSY, 0);   // Nozzle lifted .. go to busy state
            }

            break;            
        case PUMP_STATE_BUSY:                               // Pump is in a normal Transaction
            if (caller == TIMEOUT)                       // A timeout has been seen
            {   pump_stop_transaction(this_pump);                    // End of transaction
                pump_change_state(this_pump, PUMP_STATE_PAY, 30);    // change state to wait for pay off
            }
            else
            {
                if (this_pump->nozzle == 0)                              // Nozzle down 
                {   pump_stop_transaction(this_pump);                    // End of transaction
                    pump_change_state(this_pump, PUMP_STATE_PAY, 30);    // change state to wait for pay off
                }
            }

            break;            
        case PUMP_STATE_OR_LEAK:                        // Pulses seen but no nozzle control
            if (this_pump->nozzle) 
            {    pump_change_state(this_pump, PUMP_STATE_OR_TRANS, 0);   }   // change state to OR_Trans if the nozzle transitions
            break;
        case PUMP_STATE_OR_TRANS:                       // Pulses seen after a nozzle transition 
            if (this_pump->nozzle == 0) 
            {    pump_change_state(this_pump, PUMP_STATE_PAY, 30);  }   // Nozzle down .. go to paying state so Console can make a transaction

            break;
        case PUMP_STATE_PAY:            // Pump has been stopped by this device (nozzle, internal timeout etc)
            if (caller == TIMEOUT)                                      // A timeout has been seen
            {   if (this_pump->transaction_count == 0)                       // if there are no transaction counts
                {   pump_change_state(this_pump, PUMP_STATE_STOP, 0);   }    // .. we can automatically abort the payoff state
                else
                {   pump_change_state(this_pump, PUMP_STATE_PAY, 30);   }    // .. still pulses in the transaction -> restart the timer in the same state
            }
            break;            
        case PUMP_STATE_STOP:
            pump_clear_for_transaction(this_pump);
            pump_change_state(this_pump, PUMP_STATE_IDLE, 0);           // Go back to idle state once clean-up is complete
            break;    
        default:
            pump_change_state(this_pump, PUMP_STATE_IDLE, 0);           // Unknown state .. Go back to idle 
            break;
    }
}
