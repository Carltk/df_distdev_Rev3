/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// a preprocessor trick to allow all globals to be decalred easily
#define EXTERN
#include "application.h"            // application.h holds all globals

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "df_boards.h"
#include "hardware_init.h"
#include "nrf.h"
#include "sdk_config.h"

#include "nrfx_clock.h"
#include "nrfx_power.h"

#include "nrfx_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fds.h"
#include "nv_store.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "app_error.h"
#include "app_util.h"

#include "df_routines.h"

#include "console_comms.h"
#include "led_control.h"
#include "application.h"

#define LOG_LEVEL_DEFAULT       3
#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER  

static uint8_t HelloDDPC[] = "U";
static uint8_t HelloLen = sizeof(HelloDDPC);


int main(void)
{   ret_code_t ret; 
    char c;
    uint8_t need_reset = 0;

    gpio_output_voltage_setup();    // Set up the onboard voltage regulators

    NRFX_LOG_INFO("*** Starting DF_DistDev Application ***");

    clock_init();                   // Init the system clock for FDS and LED Control
    timer_init();                   // and the timer library

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    show_immediate();
    show_panic();

    NRFX_LOG_INFO("  RAM Shadow dev_address [%d], uptime [%ld]", ddpc.nv_immediate.dev_address, ddpc.nv_panic.uptime_mins); 

    APP_ERROR_CHECK(NV_Store_init());

    //ret = nrfx_power_init(NULL);
    //APP_ERROR_CHECK(ret);

    APP_ERROR_CHECK(df_hardware_init());               // Set up the hardware specific to the Distributed Device
    APP_ERROR_CHECK(ConsoleSerialPortInit());
    APP_ERROR_CHECK(application_init());

    start_nv_storage();

    check_gc_required(true);                            // Check if we need to do garbage collection (just set a flag for app_timer to handle it)

    ddpc.nv_panic.boot_count += 1;                      // inc the boot count
    flash_control.need_panic_save = true;

    NRF_LOG_INFO("DF_DistDev - Main Loop Starting");

    ConsoleWrite(HelloDDPC, HelloLen);

    while (true)
    {
        if (!NRF_LOG_PROCESS())
        {   __WFE();    }
        
        if (got_new_console_packet())
        {    handle_new_console_packet();   }
        
      //nrf_delay_ms(2000);
      //update_rng();

    }
}


/** @} */
