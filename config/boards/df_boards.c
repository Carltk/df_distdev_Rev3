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
#include "df_boards.h"

#if defined(BOARDS_WITH_USB_DFU_TRIGGER)
#include "nrf_dfu_trigger_usb.h"
#endif
#include <stdint.h>
#include <stdbool.h>

#if (defined(BOARD_PCA10059) || defined(BOARD_PCA10059_DF) || defined(BOARD_DDPC_REV2) || defined(BOARD_DDPC_REV3)  ) 
/**
 * Function for configuring UICR_REGOUT0 register
 * to set GPIO output voltage to 3.0V.
 */
uint8_t gpio_output_voltage_setup(void)
{
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);
        //NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
        //                    (voltage << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
        return(1);
    }
    return(0);
}


// Debug control is handled by 3 registers:
//  NRF_UICR->APPROTECT - Access Port Protection blocks the debugger from read write access. 
//                            Enabled=0xXXXXXX00, Disabled=0xXXXXXXFF (Default State)
//  NRF_UICR->DEBUGCTRL - Debug Control configures the CPU's debug features
//                          CPUIDEN - CPU ITM & ETM functionality. ETM trace is only in Parallel mode, Serial-
//                            Enabled=0xXXXXXXFF, Disabled=0xXXXXXX00
//                          CPUPBEN - CPU flash patch & breakpoint unit
//                            Enabled=0xXXXXFFXX, Disabled=0xXXXX00XX
//  NRF_CLOCK->TRACECONFIG - Configuration of the CPU Trace pins
//                          TRACEPORTSPEED - Speed of port clock TRACECLK = speed/2
//                            Mask: 0xXXXXXXX(xxnn) where nn: 0=32MHz, 1=16MHz, 2=8MHz, 3=4MHz
//                          TRACEMUX - Which trace port pins are in use
//                            Mask: 0xXXX(xxnn)XXXX 
//                                  where nn: 0=GPIO (no trace routed to pins all GPIO)
//                                            1=Serial (SWO routed to pin, all others can be GPIO)
//                                            2=Parallel (TRACECLK & TRACEDATA(0-3) routed to pins) 
uint8_t ddpc_rev3_configure_debug(void)
{
    uint8_t made_changes = 0;

    // Default value of APPROTECT is OK .. no need to program    
    // NRF_UICR->APPROTECT = (NRF_UICR->APPROTECT & ~((uint32_t)UICR_APPROTECT_PALL_Msk)) |
    //                       ( UICR_APPROTECT_PALL_Disabled << UICR_APPROTECT_PALL_Pos);    

    // Debug Control CPUPBEN->CPUPBEN (default value is OK)
    // NRF_UICR->DEBUGCTRL = (NRF_UICR->DEBUGCTRL & ~((uint32_t)UICR_DEBUGCTRL_CPUFPBEN_Msk)) |
    //                        ( UICR_DEBUGCTRL_CPUFPBEN_Enabled << UICR_DEBUGCTRL_CPUFPBEN_Pos);        

    // Debug Control CPUPBEN->CPUIDEN (default value is OK)
    // NRF_UICR->DEBUGCTRL = (NRF_UICR->DEBUGCTRL & ~((uint32_t)UICR_DEBUGCTRL_CPUNIDEN_Msk)) |
    //                        ( UICR_DEBUGCTRL_CPUNIDEN_Enabled << UICR_DEBUGCTRL_CPUNIDEN_Pos);        

    // TraceConfig TRACECONFIG->TRACEMUX .. set to serial mode
    if ((NRF_CLOCK->TRACECONFIG & CLOCK_TRACECONFIG_TRACEMUX_Msk) !=
                (CLOCK_TRACECONFIG_TRACEMUX_Serial << CLOCK_TRACECONFIG_TRACEMUX_Pos))
    {

        NRF_CLOCK->TRACECONFIG = (NRF_CLOCK->TRACECONFIG & ~((uint32_t)CLOCK_TRACECONFIG_TRACEMUX_Msk)) |
                                 ( CLOCK_TRACECONFIG_TRACEMUX_Serial << CLOCK_TRACECONFIG_TRACEMUX_Pos);        
                // #define CLOCK_TRACECONFIG_TRACEMUX_GPIO (0UL) /*!< No trace signals routed to pins. All pins can be used as regular GPIOs. */
                // #define CLOCK_TRACECONFIG_TRACEMUX_Serial (1UL) /*!< SWO trace signal routed to pin. Remaining pins can be used as regular GPIOs. */
                // #define CLOCK_TRACECONFIG_TRACEMUX_Parallel (2UL) /*!< All trace signals (TRACECLK and TRACEDATA[n]) routed to pins. */
        made_changes = 1;
    }

    // TraceConfig TRACECONFIG->TRACEPORTSPEED (default value is OK)
    // NRF_CLOCK->TRACECONFIG = (NRF_CLOCK->TRACECONFIG & ~((uint32_t)CLOCK_TRACECONFIG_TRACEPORTSPEED_Msk)) |
    //                          ( CLOCK_TRACECONFIG_TRACEPORTSPEED_32MHz << CLOCK_TRACECONFIG_TRACEPORTSPEED_Pos);        
            // #define CLOCK_TRACECONFIG_TRACEPORTSPEED_32MHz (0UL) /*!< 32 MHz trace port clock (TRACECLK = 16 MHz) */
            // #define CLOCK_TRACECONFIG_TRACEPORTSPEED_16MHz (1UL) /*!< 16 MHz trace port clock (TRACECLK = 8 MHz) */
            // #define CLOCK_TRACECONFIG_TRACEPORTSPEED_8MHz (2UL) /*!< 8 MHz trace port clock (TRACECLK = 4 MHz) */
            // #define CLOCK_TRACECONFIG_TRACEPORTSPEED_4MHz (3UL) /*!< 4 MHz trace port clock (TRACECLK = 2 MHz) */
    return(made_changes);
}

#endif

#if LEDS_NUMBER > 0
static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
#endif

#if BUTTONS_NUMBER > 0
static const uint8_t m_board_btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
#endif

#if LEDS_NUMBER > 0
bool bsp_board_led_state_get(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    bool pin_set = nrf_gpio_pin_out_read(m_board_led_list[led_idx]) ? true : false;
    return (pin_set == (LEDS_ACTIVE_STATE ? true : false));
}

void bsp_board_led_on(uint32_t led_idx)
{
        ASSERT(led_idx < LEDS_NUMBER);
        nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 1 : 0);
}

void bsp_board_led_off(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 0 : 1);
}

void bsp_board_leds_off(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        bsp_board_led_off(i);
    }
}

void bsp_board_leds_on(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        bsp_board_led_on(i);
    }
}

void bsp_board_led_invert(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_toggle(m_board_led_list[led_idx]);
}

static void bsp_board_leds_init(void)
{
    #if (defined(BOARD_PCA10059) || defined(BOARD_PCA10059_DF))
    // If nRF52 USB Dongle is powered from USB (high voltage mode),
    // GPIO output voltage is set to 1.8 V by default, which is not
    // enough to turn on green and blue LEDs. Therefore, GPIO voltage
    // needs to be increased to 3.0 V by configuring the UICR register.
    if (NRF_POWER->MAINREGSTATUS &
       (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
    {
        gpio_output_voltage_setup();
    }
    #endif

    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_cfg_output(m_board_led_list[i]);
    }
    bsp_board_leds_off();
}

uint32_t bsp_board_led_idx_to_pin(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    return m_board_led_list[led_idx];
}

uint32_t bsp_board_pin_to_led_idx(uint32_t pin_number)
{
    uint32_t ret = 0xFFFFFFFF;
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        if (m_board_led_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}
#endif //LEDS_NUMBER > 0



#if BUTTONS_NUMBER > 0
bool bsp_board_button_state_get(uint32_t button_idx)
{
    ASSERT(button_idx < BUTTONS_NUMBER);
    bool pin_set = nrf_gpio_pin_read(m_board_btn_list[button_idx]) ? true : false;
    return (pin_set == (BUTTONS_ACTIVE_STATE ? true : false));
}

static void bsp_board_buttons_init(void)
{
    uint32_t i;
    for (i = 0; i < BUTTONS_NUMBER; ++i)
    {
        nrf_gpio_cfg_input(m_board_btn_list[i], BUTTON_PULL);
    }
}

uint32_t bsp_board_pin_to_button_idx(uint32_t pin_number)
{
    uint32_t i;
    uint32_t ret = 0xFFFFFFFF;
    for (i = 0; i < BUTTONS_NUMBER; ++i)
    {
        if (m_board_btn_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

uint32_t bsp_board_button_idx_to_pin(uint32_t button_idx)
{
    ASSERT(button_idx < BUTTONS_NUMBER);
    return m_board_btn_list[button_idx];
}
#endif //BUTTONS_NUMBER > 0


void bsp_board_init(uint32_t init_flags)
{
    #if defined(BOARDS_WITH_USB_DFU_TRIGGER) && (defined(BOARD_PCA10059) || defined(BOARD_PCA10059_DF)  || defined(BOARD_DDPC_REV2))
    (void) nrf_dfu_trigger_usb_init();
    #endif

    #if LEDS_NUMBER > 0
    if (init_flags & BSP_INIT_LEDS)
    {
        bsp_board_leds_init();
    }
    #endif //LEDS_NUMBER > 0

    #if BUTTONS_NUMBER > 0
    if (init_flags & BSP_INIT_BUTTONS)
    {
        bsp_board_buttons_init();
    }
    #endif //BUTTONS_NUMBER > 0
}
