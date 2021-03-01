<<<<<<< HEAD
/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
/** @file
*
* @defgroup gpiote_example_main main.c
* @{
* @ingroup nrf_gpiote_example
* @brief GPIOTE Example Application main file.
*
* This file contains the source code for a sample application using GPIOTE.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#define MY_BUTTON BUTTON_1
#define SECONDPIN 11
#define OUTPUT BSP_LED_0

static nrf_drv_timer_t timer NRF_DRV_TIMER_INSTANCE(1)

//static uint32_t cap;
void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){}

void GPIOTE_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

			nrf_gpio_pin_toggle(OUTPUT);
			uint32_t cap =	NRF_TIMER1->CC[0];
			NRF_LOG_INFO("Capture value: %d",cap);
	
	
}

static void timer_setup(void)
{
	ret_code_t err_code;
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	err_code = nrf_drv_timer_init(&timer, &timer_cfg, timer_dummy_handler);
	APP_ERROR_CHECK(err_code);
	
}
	
static void gpiote_setup()
{
	ret_code_t err_code;
	nrf_gpio_cfg_output(OUTPUT);
	
	nrf_drv_gpiote_in_config_t i_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	i_config.pull = NRF_GPIO_PIN_PULLUP;
	
	err_code = nrf_drv_gpiote_in_init(BUTTON_1, &i_config, NULL); 						//configure BUTTON_1, event handler callback is set to NULL to keep it from triggering unecessary CB calls.
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
	
	
	i_config.is_watcher = false;
	i_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
	i_config.pull = NRF_GPIO_PIN_NOPULL; 																			//Pullup is not really necessary as pullup is enabled on BUTTON_1.
	
	err_code = nrf_drv_gpiote_in_init(SECONDPIN, &i_config, GPIOTE_handler); 	//Configure the second pin. Event handler assigned to store value.
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(SECONDPIN, true);
	
	
}

static void ppi_setup(void)
{
	ret_code_t err_code;
	nrf_ppi_channel_t ppi1, ppi2;
	err_code = nrf_drv_ppi_channel_alloc(&ppi1);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_alloc(&ppi2);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_assign(ppi1, nrf_drv_gpiote_in_event_addr_get(BUTTON_1), nrf_drv_timer_task_address_get(&timer, NRF_TIMER_TASK_CLEAR));
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_assign(ppi2, nrf_drv_gpiote_in_event_addr_get(SECONDPIN), nrf_drv_timer_capture_task_address_get(&timer, NRF_TIMER_TASK_CAPTURE0));
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_ppi_channel_enable(ppi1);
	nrf_drv_ppi_channel_enable(ppi2);
}

static void log_init(void)
{
	ret_code_t err_code;
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	log_init();
	
	ret_code_t err_code;
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	timer_setup();
	gpiote_setup();
	ppi_setup();
	
	nrf_drv_timer_enable(&timer);

    while (true)
    {

			// Do Nothing - GPIO can be toggled without software intervention.
    }
}


/** @} */
=======
/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
/** @file
*
* @defgroup gpiote_example_main main.c
* @{
* @ingroup nrf_gpiote_example
* @brief GPIOTE Example Application main file.
*
* This file contains the source code for a sample application using GPIOTE.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#define MY_BUTTON BUTTON_1
#define SECONDPIN 11
#define OUTPUT BSP_LED_0

static nrf_drv_timer_t timer NRF_DRV_TIMER_INSTANCE(1)

//static uint32_t cap;
void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){}

void GPIOTE_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

			nrf_gpio_pin_toggle(OUTPUT);
			uint32_t cap =	NRF_TIMER1->CC[0];
			NRF_LOG_INFO("Capture value: %d",cap);
	
	
}

static void timer_setup(void)
{
	ret_code_t err_code;
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	err_code = nrf_drv_timer_init(&timer, &timer_cfg, timer_dummy_handler);
	APP_ERROR_CHECK(err_code);
	
}
	
static void gpiote_setup()
{
	ret_code_t err_code;
	nrf_gpio_cfg_output(OUTPUT);
	
	nrf_drv_gpiote_in_config_t i_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	i_config.pull = NRF_GPIO_PIN_PULLUP;
	
	err_code = nrf_drv_gpiote_in_init(BUTTON_1, &i_config, NULL); 						//configure BUTTON_1, event handler callback is set to NULL to keep it from triggering unecessary CB calls.
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
	
	
	i_config.is_watcher = false;
	i_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
	i_config.pull = NRF_GPIO_PIN_NOPULL; 																			//Pullup is not really necessary as pullup is enabled on BUTTON_1.
	
	err_code = nrf_drv_gpiote_in_init(SECONDPIN, &i_config, GPIOTE_handler); 	//Configure the second pin. Event handler assigned to store value.
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(SECONDPIN, true);
	
	
}

static void ppi_setup(void)
{
	ret_code_t err_code;
	nrf_ppi_channel_t ppi1, ppi2;
	err_code = nrf_drv_ppi_channel_alloc(&ppi1);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_alloc(&ppi2);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_assign(ppi1, nrf_drv_gpiote_in_event_addr_get(BUTTON_1), nrf_drv_timer_task_address_get(&timer, NRF_TIMER_TASK_CLEAR));
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_assign(ppi2, nrf_drv_gpiote_in_event_addr_get(SECONDPIN), nrf_drv_timer_capture_task_address_get(&timer, NRF_TIMER_TASK_CAPTURE0));
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_ppi_channel_enable(ppi1);
	nrf_drv_ppi_channel_enable(ppi2);
}

static void log_init(void)
{
	ret_code_t err_code;
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	log_init();
	
	ret_code_t err_code;
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	timer_setup();
	gpiote_setup();
	ppi_setup();
	
	nrf_drv_timer_enable(&timer);

    while (true)
    {

			// Do Nothing - GPIO can be toggled without software intervention.
    }
}


/** @} */
>>>>>>> 53725b0687dc7551c5d07165db1bcc5a856561fa
