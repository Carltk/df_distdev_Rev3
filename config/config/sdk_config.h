/**
/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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



#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H
// <<< Use Configuration Wizard in Context Menu >>>\n
#ifdef USE_APP_CONFIG
#include "app_config.h"
#endif

// ==============================
//  Conditional compile options
// ==============================

#define DEV_KIT_CONNECTED   1               // Are we operating with a DevKit (or J-Link) connection

#define USE_RTT_BACKEND     1               // Use the Segger RTT Console as backend for Logging and CLI
#define USE_USB_BACKEND     1               // Use USB Virtual Serial port as backend for logging and CLI

#define DISCOVERY_DFLT_ADDR   0xFFFE 
#define THIS_ADDRESS    DISCOVERY_DFLT_ADDR     // The default device address used at Factory Default

// <h> Board Definition
//#define BOARD_DDPC_REV3
// Board definition is done in Project Options / Common / Code / Preprocessor / Preprocessor Definitions
// </h> 
//==========================================================


// <h> Application 

//==========================================================
// <h> application_info - Software Component

//==========================================================
// <s> APP_NAME - Application name

#ifndef APP_LONG_NAME
#define APP_LONG_NAME "Datafuel DistDev Pump Controller"
#endif

#ifndef APP_NAME
#define APP_NAME "DF_DDPC"
#endif

// <o> APP_ID - Application ID 
#ifndef APP_ID
#define APP_ID 0
#endif

// <h> APP_VERSION - Application version (semantic versioning)
//==========================================================
// <o> APP_VERSION_MAJOR - Major version  <0-1000> 

#ifndef APP_VERSION_MAJOR
#define APP_VERSION_MAJOR 0
#endif

// <o> APP_VERSION_MINOR - Minor version  <0-1000> 
#ifndef APP_VERSION_MINOR
#define APP_VERSION_MINOR 1
#endif

// <o> APP_VERSION_PATCH - Patch version  <0-1000> 
#ifndef APP_VERSION_PATCH
#define APP_VERSION_PATCH 0
#endif

// <o> APP_VERSION_PRERELEASE - Patch version  <0-1000> 
#ifndef APP_VERSION_PRERELEASE
#define APP_VERSION_PRERELEASE "PR"
#endif

// <s> APP_VERSION_METADATA - Metadata, e.g. "+some-string.01-01-2018-23-59-59"
// <i> If not empty, this string should include the leading plus (+).
// <i> This string might be normalized at run-time to not contain characters
// <i> illegal in Semantic Versioning.
#ifndef APP_VERSION_METADATA
#define APP_VERSION_METADATA "+" __DATE__ " " __TIME__
#endif


// </h> 
//==========================================================



//==========================================================
// <h> nrf_dfu_trigger_usb - USB DFU Trigger library
//==========================================================
// <q> NRF_DFU_TRIGGER_USB_USB_SHARED  - Flag indicating whether USB is used for other purposes in the application.
#ifndef NRF_DFU_TRIGGER_USB_USB_SHARED
#define NRF_DFU_TRIGGER_USB_USB_SHARED 1
#endif

// <o> NRF_DFU_TRIGGER_USB_INTERFACE_NUM - The USB interface to use for the DFU Trigger library.  <0-255> 
// <i> According to the USB Specification, interface numbers cannot have
// <i> gaps. Tailor this value to adhere to this limitation.
#ifndef NRF_DFU_TRIGGER_USB_INTERFACE_NUM
#define NRF_DFU_TRIGGER_USB_INTERFACE_NUM 2
#endif

// </h> 
//==========================================================
// </h> 
//==========================================================



// <h> nRF_Drivers 
//==========================================================
// <e> GPIOTE_ENABLED - nrf_drv_gpiote - GPIOTE peripheral driver - legacy layer
//==========================================================
#ifndef GPIOTE_ENABLED
#define GPIOTE_ENABLED 1
#endif
// <o> GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS - Number of lower power input pins 
#ifndef GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 5
#endif

// <o> GPIOTE_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef GPIOTE_CONFIG_IRQ_PRIORITY
#define GPIOTE_CONFIG_IRQ_PRIORITY 6
#endif

// </e>

// <e> NRFX_CLOCK_ENABLED - nrfx_clock - CLOCK peripheral driver
//==========================================================
#ifndef NRFX_CLOCK_ENABLED
#define NRFX_CLOCK_ENABLED 1
#endif
// <o> NRFX_CLOCK_CONFIG_LF_SRC  - LF Clock Source
 
// <0=> RC 
// <1=> XTAL 
// <2=> Synth 
// <131073=> External Low Swing 
// <196609=> External Full Swing 

#ifndef NRFX_CLOCK_CONFIG_LF_SRC
#define NRFX_CLOCK_CONFIG_LF_SRC 1
#endif

// <o> NRFX_CLOCK_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_CLOCK_CONFIG_IRQ_PRIORITY
#define NRFX_CLOCK_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_CLOCK_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_CLOCK_CONFIG_LOG_ENABLED
#define NRFX_CLOCK_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_CLOCK_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_CLOCK_CONFIG_LOG_LEVEL
#define NRFX_CLOCK_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_CLOCK_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_CLOCK_CONFIG_INFO_COLOR
#define NRFX_CLOCK_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_CLOCK_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_CLOCK_CONFIG_DEBUG_COLOR
#define NRFX_CLOCK_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRFX_GPIOTE_ENABLED - nrfx_gpiote - GPIOTE peripheral driver
//==========================================================
#ifndef NRFX_GPIOTE_ENABLED
#define NRFX_GPIOTE_ENABLED 1
#endif
// <o> NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS - Number of lower power input pins 
#ifndef NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
#define NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 1
#endif

// <o> NRFX_GPIOTE_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_GPIOTE_CONFIG_IRQ_PRIORITY
#define NRFX_GPIOTE_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_GPIOTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_GPIOTE_CONFIG_LOG_ENABLED
#define NRFX_GPIOTE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_GPIOTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_GPIOTE_CONFIG_LOG_LEVEL
#define NRFX_GPIOTE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_GPIOTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_GPIOTE_CONFIG_INFO_COLOR
#define NRFX_GPIOTE_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_GPIOTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_GPIOTE_CONFIG_DEBUG_COLOR
#define NRFX_GPIOTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRFX_POWER_ENABLED - nrfx_power - POWER peripheral driver
//==========================================================
#ifndef NRFX_POWER_ENABLED
#define NRFX_POWER_ENABLED 1
#endif
// <o> NRFX_POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_POWER_CONFIG_IRQ_PRIORITY
#define NRFX_POWER_CONFIG_IRQ_PRIORITY 6
#endif

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef NRFX_POWER_CONFIG_DEFAULT_DCDCEN
#define NRFX_POWER_CONFIG_DEFAULT_DCDCEN 0
#endif

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef NRFX_POWER_CONFIG_DEFAULT_DCDCENHV
#define NRFX_POWER_CONFIG_DEFAULT_DCDCENHV 0
#endif

// </e>

// <e> NRFX_PPI_ENABLED - nrfx_ppi - PPI peripheral allocator
//==========================================================
#ifndef NRFX_PPI_ENABLED
#define NRFX_PPI_ENABLED 1
#endif
// <e> NRFX_PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PPI_CONFIG_LOG_ENABLED
#define NRFX_PPI_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_PPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_PPI_CONFIG_LOG_LEVEL
#define NRFX_PPI_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_PPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PPI_CONFIG_INFO_COLOR
#define NRFX_PPI_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_PPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PPI_CONFIG_DEBUG_COLOR
#define NRFX_PPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRFX_PRS_ENABLED - nrfx_prs - Peripheral Resource Sharing module
//==========================================================
#ifndef NRFX_PRS_ENABLED
#define NRFX_PRS_ENABLED 1
#endif
// <q> NRFX_PRS_BOX_0_ENABLED  - Enables box 0 in the module.
 

#ifndef NRFX_PRS_BOX_0_ENABLED
#define NRFX_PRS_BOX_0_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_1_ENABLED  - Enables box 1 in the module.
 

#ifndef NRFX_PRS_BOX_1_ENABLED
#define NRFX_PRS_BOX_1_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_2_ENABLED  - Enables box 2 in the module.
 

#ifndef NRFX_PRS_BOX_2_ENABLED
#define NRFX_PRS_BOX_2_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_3_ENABLED  - Enables box 3 in the module.
 

#ifndef NRFX_PRS_BOX_3_ENABLED
#define NRFX_PRS_BOX_3_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_4_ENABLED  - Enables box 4 in the module.
 

#ifndef NRFX_PRS_BOX_4_ENABLED
#define NRFX_PRS_BOX_4_ENABLED 1
#endif

// <e> NRFX_PRS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PRS_CONFIG_LOG_ENABLED
#define NRFX_PRS_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_PRS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_PRS_CONFIG_LOG_LEVEL
#define NRFX_PRS_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_PRS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_INFO_COLOR
#define NRFX_PRS_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_PRS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_DEBUG_COLOR
#define NRFX_PRS_CONFIG_DEBUG_COLOR 0
#endif

// </e>


// <e> NRFX_SAADC_ENABLED - nrfx_saadc - SAADC peripheral driver
//==========================================================
#ifndef NRFX_SAADC_ENABLED
#define NRFX_SAADC_ENABLED 1
#endif
// <o> NRFX_SAADC_CONFIG_RESOLUTION  - Resolution
 
// <0=> 8 bit 
// <1=> 10 bit 
// <2=> 12 bit 
// <3=> 14 bit 

#ifndef NRFX_SAADC_CONFIG_RESOLUTION
#define NRFX_SAADC_CONFIG_RESOLUTION 2
#endif

// <o> NRFX_SAADC_CONFIG_OVERSAMPLE  - Sample period
 
// <0=> Disabled 
// <1=> 2x 
// <2=> 4x 
// <3=> 8x 
// <4=> 16x 
// <5=> 32x 
// <6=> 64x 
// <7=> 128x 
// <8=> 256x 

#ifndef NRFX_SAADC_CONFIG_OVERSAMPLE
#define NRFX_SAADC_CONFIG_OVERSAMPLE 0
#endif

// <q> NRFX_SAADC_CONFIG_LP_MODE  - Enabling low power mode

#ifndef NRFX_SAADC_CONFIG_LP_MODE
#define NRFX_SAADC_CONFIG_LP_MODE 0
#endif

// <o> NRFX_SAADC_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_SAADC_CONFIG_IRQ_PRIORITY
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_SAADC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_SAADC_CONFIG_LOG_ENABLED
#define NRFX_SAADC_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_SAADC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_SAADC_CONFIG_LOG_LEVEL
#define NRFX_SAADC_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_SAADC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_SAADC_CONFIG_INFO_COLOR
#define NRFX_SAADC_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_SAADC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_SAADC_CONFIG_DEBUG_COLOR
#define NRFX_SAADC_CONFIG_DEBUG_COLOR 0
#endif

// </e>
// </e>


// <e> NRFX_TEMP_ENABLED - nrfx_temp - TEMP peripheral driver
//==========================================================
#ifndef NRFX_TEMP_ENABLED
#define NRFX_TEMP_ENABLED 1
#endif

// <o> NRFX_TEMP_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority

// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7

#ifndef NRFX_TEMP_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TEMP_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

// </e>


// <e> NRFX_TIMER_ENABLED - nrfx_timer - TIMER periperal driver
//==========================================================
#ifndef NRFX_TIMER_ENABLED
#define NRFX_TIMER_ENABLED 1
#endif
// <q> NRFX_TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef NRFX_TIMER0_ENABLED
#define NRFX_TIMER0_ENABLED 1
#endif
// <q> NRFX_TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef NRFX_TIMER1_ENABLED
#define NRFX_TIMER1_ENABLED 1
#endif
// <q> NRFX_TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef NRFX_TIMER2_ENABLED
#define NRFX_TIMER2_ENABLED 1
#endif
// <q> NRFX_TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef NRFX_TIMER3_ENABLED
#define NRFX_TIMER3_ENABLED 0
#endif
// <q> NRFX_TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef NRFX_TIMER4_ENABLED
#define NRFX_TIMER4_ENABLED 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY
#define NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_MODE
#define NRFX_TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_TIMER_CONFIG_LOG_ENABLED
#define NRFX_TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_TIMER_CONFIG_LOG_LEVEL
#define NRFX_TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_INFO_COLOR
#define NRFX_TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_DEBUG_COLOR
#define NRFX_TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>
// </e>

// <e> NRFX_I2S_ENABLED - nrfx_i2s - I2S peripheral driver
//==========================================================
#ifndef NRFX_I2S_ENABLED
#define NRFX_I2S_ENABLED 1
#endif

// <o> NRFX_I2S_CONFIG_SCK_PIN - SCK pin  <0-31> 
#ifndef NRFX_I2S_CONFIG_SCK_PIN
#define NRFX_I2S_CONFIG_SCK_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_LRCK_PIN - LRCK pin  <1-31> 
#ifndef NRFX_I2S_CONFIG_LRCK_PIN
#define NRFX_I2S_CONFIG_LRCK_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_MCK_PIN - MCK pin 
#ifndef NRFX_I2S_CONFIG_MCK_PIN
#define NRFX_I2S_CONFIG_MCK_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_SDOUT_PIN - SDOUT pin  <0-31> 
#ifndef NRFX_I2S_CONFIG_SDOUT_PIN
#define NRFX_I2S_CONFIG_SDOUT_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_SDIN_PIN - SDIN pin  <0-31> 
#ifndef NRFX_I2S_CONFIG_SDIN_PIN
#define NRFX_I2S_CONFIG_SDIN_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_MASTER  - Mode
// <0=> Master 
// <1=> Slave 
#ifndef NRFX_I2S_CONFIG_MASTER
#define NRFX_I2S_CONFIG_MASTER 0
#endif

// <o> NRFX_I2S_CONFIG_FORMAT  - Format
// <0=> I2S 
// <1=> Aligned 
#ifndef NRFX_I2S_CONFIG_FORMAT
#define NRFX_I2S_CONFIG_FORMAT 0
#endif

// <o> NRFX_I2S_CONFIG_ALIGN  - Alignment
// <0=> Left 
// <1=> Right 
#ifndef NRFX_I2S_CONFIG_ALIGN
#define NRFX_I2S_CONFIG_ALIGN 0
#endif

// <o> NRFX_I2S_CONFIG_SWIDTH  - Sample width (bits)
// <0=> 8 
// <1=> 16 
// <2=> 24 
#ifndef NRFX_I2S_CONFIG_SWIDTH
#define NRFX_I2S_CONFIG_SWIDTH 1
#endif

// <o> NRFX_I2S_CONFIG_CHANNELS  - Channels
// <0=> Stereo 
// <1=> Left 
// <2=> Right 
#ifndef NRFX_I2S_CONFIG_CHANNELS
#define NRFX_I2S_CONFIG_CHANNELS 0
#endif

// <o> NRFX_I2S_CONFIG_MCK_SETUP  - MCK behavior
 
// <0=> Disabled 
// <2147483648=> 32MHz/2 
// <1342177280=> 32MHz/3 
// <1073741824=> 32MHz/4 
// <805306368=> 32MHz/5 
// <671088640=> 32MHz/6 
// <536870912=> 32MHz/8 
// <402653184=> 32MHz/10 
// <369098752=> 32MHz/11 
// <285212672=> 32MHz/15 
// <268435456=> 32MHz/16 
// <201326592=> 32MHz/21 
// <184549376=> 32MHz/23 
// <142606336=> 32MHz/30 
// <138412032=> 32MHz/31 
// <134217728=> 32MHz/32 
// <100663296=> 32MHz/42 
// <68157440=> 32MHz/63 
// <34340864=> 32MHz/125 

#ifndef NRFX_I2S_CONFIG_MCK_SETUP
#define NRFX_I2S_CONFIG_MCK_SETUP 402653184
#endif

// <o> NRFX_I2S_CONFIG_RATIO  - MCK/LRCK ratio
 
// <0=> 32x 
// <1=> 48x 
// <2=> 64x 
// <3=> 96x 
// <4=> 128x 
// <5=> 192x 
// <6=> 256x 
// <7=> 384x 
// <8=> 512x 

#ifndef NRFX_I2S_CONFIG_RATIO
#define NRFX_I2S_CONFIG_RATIO 2000
#endif

// <o> NRFX_I2S_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_I2S_CONFIG_IRQ_PRIORITY
#define NRFX_I2S_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_I2S_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_I2S_CONFIG_LOG_ENABLED
#define NRFX_I2S_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_I2S_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_I2S_CONFIG_LOG_LEVEL
#define NRFX_I2S_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_I2S_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_I2S_CONFIG_INFO_COLOR
#define NRFX_I2S_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_I2S_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_I2S_CONFIG_DEBUG_COLOR
#define NRFX_I2S_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

//==========================================================

// <h> nRF_Drivers 

//==========================================================
// <e> I2S_ENABLED - nrf_drv_i2s - I2S peripheral driver - legacy layer
//==========================================================
#ifndef I2S_ENABLED
#define I2S_ENABLED 1
#endif

// <o> I2S_CONFIG_SCK_PIN - SCK pin  <0-31> 
#ifndef I2S_CONFIG_SCK_PIN
#define I2S_CONFIG_SCK_PIN 255
#endif

// <o> I2S_CONFIG_LRCK_PIN - LRCK pin  <1-31> 
#ifndef I2S_CONFIG_LRCK_PIN
#define I2S_CONFIG_LRCK_PIN 255
#endif

// <o> I2S_CONFIG_MCK_PIN - MCK pin 
#ifndef I2S_CONFIG_MCK_PIN
#define I2S_CONFIG_MCK_PIN 255
#endif

// <o> I2S_CONFIG_SDOUT_PIN - SDOUT pin  <0-31> 
#ifndef I2S_CONFIG_SDOUT_PIN
#define I2S_CONFIG_SDOUT_PIN 255
#endif

// <o> I2S_CONFIG_SDIN_PIN - SDIN pin  <0-31> 
#ifndef I2S_CONFIG_SDIN_PIN
#define I2S_CONFIG_SDIN_PIN 255
#endif

// <o> I2S_CONFIG_MASTER  - Mode
#ifndef I2S_CONFIG_MASTER
#define I2S_CONFIG_MASTER 0
#endif

// <o> I2S_CONFIG_FORMAT  - Format
#ifndef I2S_CONFIG_FORMAT
#define I2S_CONFIG_FORMAT 0
#endif

// <o> I2S_CONFIG_ALIGN  - Alignment
#ifndef I2S_CONFIG_ALIGN
#define I2S_CONFIG_ALIGN 0
#endif

// <o> I2S_CONFIG_SWIDTH  - Sample width (bits)
#ifndef I2S_CONFIG_SWIDTH
#define I2S_CONFIG_SWIDTH 1
#endif

// <o> I2S_CONFIG_CHANNELS  - Channels
#ifndef I2S_CONFIG_CHANNELS
#define I2S_CONFIG_CHANNELS 0
#endif

// <o> I2S_CONFIG_MCK_SETUP  - MCK behavior
#ifndef I2S_CONFIG_MCK_SETUP
#define I2S_CONFIG_MCK_SETUP  402653184
#endif

// <o> I2S_CONFIG_RATIO  - MCK/LRCK ratio
#ifndef I2S_CONFIG_RATIO
#define I2S_CONFIG_RATIO 2000
#endif

// <o> I2S_CONFIG_IRQ_PRIORITY  - Interrupt priority
#ifndef I2S_CONFIG_IRQ_PRIORITY
#define I2S_CONFIG_IRQ_PRIORITY 6
#endif

// <e> I2S_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef I2S_CONFIG_LOG_ENABLED
#define I2S_CONFIG_LOG_ENABLED 0
#endif

// <o> I2S_CONFIG_LOG_LEVEL  - Default Severity level
#ifndef I2S_CONFIG_LOG_LEVEL
#define I2S_CONFIG_LOG_LEVEL 3
#endif

// <o> I2S_CONFIG_INFO_COLOR  - ANSI escape code prefix.
#ifndef I2S_CONFIG_INFO_COLOR
#define I2S_CONFIG_INFO_COLOR 0
#endif

// <o> I2S_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
#ifndef I2S_CONFIG_DEBUG_COLOR
#define I2S_CONFIG_DEBUG_COLOR 0
#endif

// </e>
// </e>

// <e> NRF_CLOCK_ENABLED - nrf_drv_clock - CLOCK peripheral driver - legacy layer
//==========================================================
#ifndef NRF_CLOCK_ENABLED
#define NRF_CLOCK_ENABLED 1
#endif
// <o> CLOCK_CONFIG_LF_SRC  - LF Clock Source
 
// <0=> RC 
// <1=> XTAL 
// <2=> Synth 
// <131073=> External Low Swing 
// <196609=> External Full Swing 

#ifndef CLOCK_CONFIG_LF_SRC
#define CLOCK_CONFIG_LF_SRC 1
#endif

// <q> CLOCK_CONFIG_LF_CAL_ENABLED  - Calibration enable for LF Clock Source
 

#ifndef CLOCK_CONFIG_LF_CAL_ENABLED
#define CLOCK_CONFIG_LF_CAL_ENABLED 0
#endif

// <o> CLOCK_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef CLOCK_CONFIG_IRQ_PRIORITY
#define CLOCK_CONFIG_IRQ_PRIORITY 6
#endif

// </e>

// <e> POWER_ENABLED - nrf_drv_power - POWER peripheral driver - legacy layer
//==========================================================
#ifndef POWER_ENABLED
#define POWER_ENABLED 1
#endif
// <o> POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef POWER_CONFIG_IRQ_PRIORITY
#define POWER_CONFIG_IRQ_PRIORITY 6
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCEN
#define POWER_CONFIG_DEFAULT_DCDCEN 0
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCENHV
#define POWER_CONFIG_DEFAULT_DCDCENHV 0
#endif

// </e>


// <e> USBD_ENABLED - nrf_drv_usbd - Software Component
//==========================================================
#ifndef USBD_ENABLED
#define USBD_ENABLED 1
#endif
// <o> USBD_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef USBD_CONFIG_IRQ_PRIORITY
#define USBD_CONFIG_IRQ_PRIORITY 6
#endif

// <o> USBD_CONFIG_DMASCHEDULER_MODE  - USBD SMA scheduler working scheme
 
// <0=> Prioritized access 
// <1=> Round Robin 

#ifndef USBD_CONFIG_DMASCHEDULER_MODE
#define USBD_CONFIG_DMASCHEDULER_MODE 0
#endif

// <q> USBD_CONFIG_DMASCHEDULER_ISO_BOOST  - Give priority to isochronous transfers
 

// <i> This option gives priority to isochronous transfers.
// <i> Enabling it assures that isochronous transfers are always processed,
// <i> even if multiple other transfers are pending.
// <i> Isochronous endpoints are prioritized before the usbd_dma_scheduler_algorithm
// <i> function is called, so the option is independent of the algorithm chosen.

#ifndef USBD_CONFIG_DMASCHEDULER_ISO_BOOST
#define USBD_CONFIG_DMASCHEDULER_ISO_BOOST 1
#endif

// <q> USBD_CONFIG_ISO_IN_ZLP  - Respond to an IN token on ISO IN endpoint with ZLP when no data is ready
 

// <i> If set, ISO IN endpoint will respond to an IN token with ZLP when no data is ready to be sent.
// <i> Else, there will be no response.
// <i> NOTE: This option does not work on Engineering A chip.

#ifndef USBD_CONFIG_ISO_IN_ZLP
#define USBD_CONFIG_ISO_IN_ZLP 0
#endif

// </e>

// </h> 
//==========================================================




// <e> NRF_FSTORAGE_ENABLED - nrf_fstorage - Flash abstraction library
//==========================================================
#ifndef NRF_FSTORAGE_ENABLED
#define NRF_FSTORAGE_ENABLED 1
#endif
// <h> nrf_fstorage - Common settings

// <i> Common settings to all fstorage implementations
//==========================================================
// <q> NRF_FSTORAGE_PARAM_CHECK_DISABLED  - Disable user input validation
 
// <i> If selected, use ASSERT to validate user input.
// <i> This effectively removes user input validation in production code.
// <i> Recommended setting: OFF, only enable this setting if size is a major concern.

#ifndef NRF_FSTORAGE_PARAM_CHECK_DISABLED
#define NRF_FSTORAGE_PARAM_CHECK_DISABLED 0
#endif

// </h> 
//==========================================================
// <h> nrf_fstorage_sd - Implementation using the SoftDevice
// <i> Configuration options for the fstorage implementation using the SoftDevice
//==========================================================
// <o> NRF_FSTORAGE_SD_QUEUE_SIZE - Size of the internal queue of operations 
// <i> Increase this value if API calls frequently return the error @ref NRF_ERROR_NO_MEM.

#ifndef NRF_FSTORAGE_SD_QUEUE_SIZE
#define NRF_FSTORAGE_SD_QUEUE_SIZE 2
#endif

// <o> NRF_FSTORAGE_SD_MAX_RETRIES - Maximum number of attempts at executing an operation when the SoftDevice is busy 
// <i> Increase this value if events frequently return the @ref NRF_ERROR_TIMEOUT error.
// <i> The SoftDevice might fail to schedule flash access due to high BLE activity.

#ifndef NRF_FSTORAGE_SD_MAX_RETRIES
#define NRF_FSTORAGE_SD_MAX_RETRIES 0
#endif

// <o> NRF_FSTORAGE_SD_MAX_WRITE_SIZE - Maximum number of bytes to be written to flash in a single operation 
// <i> This value must be a multiple of four.
// <i> Lowering this value can increase the chances of the SoftDevice being able to execute flash operations in between radio activity.
// <i> This value is bound by the maximum number of bytes that can be written to flash in a single call to @ref sd_flash_write.
// <i> That is 1024 bytes for nRF51 ICs and 4096 bytes for nRF52 ICs.

#ifndef NRF_FSTORAGE_SD_MAX_WRITE_SIZE
#define NRF_FSTORAGE_SD_MAX_WRITE_SIZE 1024
#endif

// </h> 

//==========================================================
// <q> CRC16_ENABLED  - crc16 - CRC16 calculation routines

#ifndef CRC16_ENABLED
#define CRC16_ENABLED 1
#endif

// <e> FDS_ENABLED - fds - Flash data storage module
//==========================================================
#ifndef FDS_ENABLED
#define FDS_ENABLED 1
#endif
// <h> Pages - Virtual page settings

// <i> Configure the number of virtual pages to use and their size.
//==========================================================
// <o> FDS_VIRTUAL_PAGES - Number of virtual flash pages to use. 
// <i> One of the virtual pages is reserved by the system for garbage collection.
// <i> Therefore, the minimum is two virtual pages: one page to store data and one page to be used by the system for garbage collection.
// <i> The total amount of flash memory that is used by FDS amounts to @ref FDS_VIRTUAL_PAGES * @ref FDS_VIRTUAL_PAGE_SIZE * 4 bytes.

#ifndef FDS_VIRTUAL_PAGES
#define FDS_VIRTUAL_PAGES 2
#endif

// <o> FDS_VIRTUAL_PAGE_SIZE  - The size of a virtual flash page.
 

// <i> Expressed in number of 4-byte words.
// <i> By default, a virtual page is the same size as a physical page.
// <i> The size of a virtual page must be a multiple of the size of a physical page.
// <1024=> 1024 
// <2048=> 2048 

#ifndef FDS_VIRTUAL_PAGE_SIZE
#define FDS_VIRTUAL_PAGE_SIZE 1024
#endif

// <o> FDS_VIRTUAL_PAGES_RESERVED - The number of virtual flash pages that are used by other modules. 
// <i> FDS module stores its data in the last pages of the flash memory.
// <i> By setting this value, you can move flash end address used by the FDS.
// <i> As a result the reserved space can be used by other modules.

#ifndef FDS_VIRTUAL_PAGES_RESERVED
#define FDS_VIRTUAL_PAGES_RESERVED 0
#endif

// </h> 
//==========================================================

// <h> Backend - Backend configuration

// <i> Configure which nrf_fstorage backend is used by FDS to write to flash.
//==========================================================
// <o> FDS_BACKEND  - FDS flash backend.
 

// <i> NRF_FSTORAGE_SD uses the nrf_fstorage_sd backend implementation using the SoftDevice API. Use this if you have a SoftDevice present.
// <i> NRF_FSTORAGE_NVMC uses the nrf_fstorage_nvmc implementation. Use this setting if you don't use the SoftDevice.
// <1=> NRF_FSTORAGE_NVMC 
// <2=> NRF_FSTORAGE_SD 

#ifndef NRFX_NVMC_ENABLED
#define NRFX_NVMC_ENABLED 1
#endif 

#ifndef FDS_BACKEND
#define FDS_BACKEND 1
#endif

// </h> 
//==========================================================

// <h> Queue - Queue settings

//==========================================================
// <o> FDS_OP_QUEUE_SIZE - Size of the internal queue. 
// <i> Increase this value if you frequently get synchronous FDS_ERR_NO_SPACE_IN_QUEUES errors.

#ifndef FDS_OP_QUEUE_SIZE
#define FDS_OP_QUEUE_SIZE 2
#endif

// </h> 
//==========================================================

// <h> CRC - CRC functionality

//==========================================================
// <e> FDS_CRC_CHECK_ON_READ - Enable CRC checks.

// <i> Save a record's CRC when it is written to flash and check it when the record is opened.
// <i> Records with an incorrect CRC can still be 'seen' by the user using FDS functions, but they cannot be opened.
// <i> Additionally, they will not be garbage collected until they are deleted.
//==========================================================
#ifndef FDS_CRC_CHECK_ON_READ
#define FDS_CRC_CHECK_ON_READ 1
#endif
// <o> FDS_CRC_CHECK_ON_WRITE  - Perform a CRC check on newly written records.
 

// <i> Perform a CRC check on newly written records.
// <i> This setting can be used to make sure that the record data was not altered while being written to flash.
// <1=> Enabled 
// <0=> Disabled 

#ifndef FDS_CRC_CHECK_ON_WRITE
#define FDS_CRC_CHECK_ON_WRITE 1
#endif

// </e>

// </h> 
//==========================================================

// <h> Users - Number of users

//==========================================================
// <o> FDS_MAX_USERS - Maximum number of callbacks that can be registered. 
#ifndef FDS_MAX_USERS
#define FDS_MAX_USERS 1
#endif

// </h> 
//==========================================================


// <h> nRF_Libraries 
//==========================================================
// <e> APP_SCHEDULER_ENABLED - app_scheduler - Events scheduler
//==========================================================
#ifndef APP_SCHEDULER_ENABLED
#define APP_SCHEDULER_ENABLED 0
#endif
// <q> APP_SCHEDULER_WITH_PAUSE  - Enabling pause feature
 

#ifndef APP_SCHEDULER_WITH_PAUSE
#define APP_SCHEDULER_WITH_PAUSE 0
#endif

// <q> APP_SCHEDULER_WITH_PROFILER  - Enabling scheduler profiling
 

#ifndef APP_SCHEDULER_WITH_PROFILER
#define APP_SCHEDULER_WITH_PROFILER 0
#endif

// </e>

// <e> APP_TIMER_ENABLED - app_timer - Application timer functionality
//==========================================================
#ifndef APP_TIMER_ENABLED
#define APP_TIMER_ENABLED 1
#endif
// <o> APP_TIMER_CONFIG_RTC_FREQUENCY  - Configure RTC prescaler.
 
// <0=> 32768 Hz 
// <1=> 16384 Hz 
// <3=> 8192 Hz 
// <7=> 4096 Hz 
// <15=> 2048 Hz 
// <31=> 1024 Hz 

#ifndef APP_TIMER_CONFIG_RTC_FREQUENCY
#define APP_TIMER_CONFIG_RTC_FREQUENCY 0
#endif

// <o> APP_TIMER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef APP_TIMER_CONFIG_IRQ_PRIORITY
#define APP_TIMER_CONFIG_IRQ_PRIORITY 6
#endif

// <o> APP_TIMER_CONFIG_OP_QUEUE_SIZE - Capacity of timer requests queue. 
// <i> Size of the queue depends on how many timers are used
// <i> in the system, how often timers are started and overall
// <i> system latency. If queue size is too small app_timer calls
// <i> will fail.

#ifndef APP_TIMER_CONFIG_OP_QUEUE_SIZE
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE 10
#endif

// <q> APP_TIMER_CONFIG_USE_SCHEDULER  - Enable scheduling app_timer events to app_scheduler
 

#ifndef APP_TIMER_CONFIG_USE_SCHEDULER
#define APP_TIMER_CONFIG_USE_SCHEDULER 0
#endif

// <q> APP_TIMER_KEEPS_RTC_ACTIVE  - Enable RTC always on
 

// <i> If option is enabled RTC is kept running even if there is no active timers.
// <i> This option can be used when app_timer is used for timestamping.

#ifndef APP_TIMER_KEEPS_RTC_ACTIVE
#define APP_TIMER_KEEPS_RTC_ACTIVE 1
#endif

// <o> APP_TIMER_SAFE_WINDOW_MS - Maximum possible latency (in milliseconds) of handling app_timer event. 
// <i> Maximum possible timeout that can be set is reduced by safe window.
// <i> Example: RTC frequency 16384 Hz, maximum possible timeout 1024 seconds - APP_TIMER_SAFE_WINDOW_MS.
// <i> Since RTC is not stopped when processor is halted in debugging session, this value
// <i> must cover it if debugging is needed. It is possible to halt processor for APP_TIMER_SAFE_WINDOW_MS
// <i> without corrupting app_timer behavior.

#ifndef APP_TIMER_SAFE_WINDOW_MS
#define APP_TIMER_SAFE_WINDOW_MS 300000
#endif

// <h> App Timer Legacy configuration - Legacy configuration.

//==========================================================
// <q> APP_TIMER_WITH_PROFILER  - Enable app_timer profiling
 

#ifndef APP_TIMER_WITH_PROFILER
#define APP_TIMER_WITH_PROFILER 0
#endif

// <q> APP_TIMER_CONFIG_SWI_NUMBER  - Configure SWI instance used.
 

#ifndef APP_TIMER_CONFIG_SWI_NUMBER
#define APP_TIMER_CONFIG_SWI_NUMBER 0
#endif

// </h> 
//==========================================================

// </e>

// <q> HARDFAULT_HANDLER_ENABLED  - hardfault_default - HardFault default handler for debugging and release
 

#ifndef HARDFAULT_HANDLER_ENABLED
#define HARDFAULT_HANDLER_ENABLED 1
#endif

// <e> NRF_BALLOC_ENABLED - nrf_balloc - Block allocator module
//==========================================================
#ifndef NRF_BALLOC_ENABLED
#define NRF_BALLOC_ENABLED 1
#endif
// <e> NRF_BALLOC_CONFIG_DEBUG_ENABLED - Enables debug mode in the module.
//==========================================================
#ifndef NRF_BALLOC_CONFIG_DEBUG_ENABLED
#define NRF_BALLOC_CONFIG_DEBUG_ENABLED 0
#endif
// <o> NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS - Number of words used as head guard.  <0-255> 


#ifndef NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS
#define NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS 1
#endif

// <o> NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS - Number of words used as tail guard.  <0-255> 


#ifndef NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS
#define NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS 1
#endif

// <q> NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED  - Enables basic checks in this module.
 

#ifndef NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED
#define NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED 0
#endif

// <q> NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED  - Enables double memory free check in this module.
 

#ifndef NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED
#define NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED 0
#endif

// <q> NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED  - Enables free memory corruption check in this module.
 

#ifndef NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED
#define NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED 0
#endif

// <q> NRF_BALLOC_CLI_CMDS  - Enable CLI commands specific to the module
 

#ifndef NRF_BALLOC_CLI_CMDS
#define NRF_BALLOC_CLI_CMDS 0
#endif

// </e>
// </e>

// <q> NRF_LIBUARTE_ASYNC_WITH_APP_TIMER  - nrf_libuarte_async - libUARTE_async library
 
#ifndef NRF_LIBUARTE_ASYNC_WITH_APP_TIMER
#define NRF_LIBUARTE_ASYNC_WITH_APP_TIMER 1
#endif

// <q> NRF_MEMOBJ_ENABLED  - nrf_memobj - Linked memory allocator module
 

#ifndef NRF_MEMOBJ_ENABLED
#define NRF_MEMOBJ_ENABLED 1
#endif

// <e> NRF_QUEUE_ENABLED - nrf_queue - Queue module
//==========================================================
#ifndef NRF_QUEUE_ENABLED
#define NRF_QUEUE_ENABLED 1
#endif
// <q> NRF_QUEUE_CLI_CMDS  - Enable CLI commands specific to the module
 

#ifndef NRF_QUEUE_CLI_CMDS
#define NRF_QUEUE_CLI_CMDS 0
#endif

// </e>

// <q> NRF_SORTLIST_ENABLED  - nrf_sortlist - Sorted list
 

#ifndef NRF_SORTLIST_ENABLED
#define NRF_SORTLIST_ENABLED 1
#endif

// <q> NRF_STRERROR_ENABLED  - nrf_strerror - Library for converting error code to string.
#ifndef NRF_STRERROR_ENABLED
#define NRF_STRERROR_ENABLED 1
#endif

// <h> app_button - buttons handling module

//==========================================================
// <q> BUTTON_ENABLED  - Enables Button module
#ifndef BUTTON_ENABLED
#define BUTTON_ENABLED 1
#endif

// <q> BUTTON_HIGH_ACCURACY_ENABLED  - Enables GPIOTE high accuracy for buttons
#ifndef BUTTON_HIGH_ACCURACY_ENABLED
#define BUTTON_HIGH_ACCURACY_ENABLED 0
#endif

// </h> 
//==========================================================

// <h> nrf_fprintf - fprintf function.

//==========================================================
// <q> NRF_FPRINTF_ENABLED  - Enable/disable fprintf module.
#ifndef NRF_FPRINTF_ENABLED
#define NRF_FPRINTF_ENABLED 1
#endif

// <q> NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED  - For each printed LF, function will add CR.
#ifndef NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED 0
#endif

// <q> NRF_FPRINTF_DOUBLE_ENABLED  - Enable IEEE-754 double precision formatting.
#ifndef NRF_FPRINTF_DOUBLE_ENABLED
#define NRF_FPRINTF_DOUBLE_ENABLED 0
#endif


// </h> 
//==========================================================

// <h> nrf_libuarte_drv - libUARTE library

//==========================================================
// <q> NRF_LIBUARTE_DRV_HWFC_ENABLED  - Enable HWFC support in the driver
 

#ifndef NRF_LIBUARTE_DRV_HWFC_ENABLED
#define NRF_LIBUARTE_DRV_HWFC_ENABLED 0
#endif

// <q> NRF_LIBUARTE_DRV_UARTE0  - UARTE0 instance
#ifndef NRF_LIBUARTE_DRV_UARTE0
#define NRF_LIBUARTE_DRV_UARTE0 1
#endif

// <q> NRF_LIBUARTE_DRV_UARTE1  - UARTE1 instance
#ifndef NRF_LIBUARTE_DRV_UARTE1
#define NRF_LIBUARTE_DRV_UARTE1 0
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================

//==========================================================
// <e> NRF_LOG_BACKEND_RTT_ENABLED - nrf_log_backend_rtt - Log RTT backend
//==========================================================
#ifndef NRF_LOG_ENABLED
#define NRF_LOG_ENABLED 1
#endif

// <h> Log message backend - Configuration of log message backend handler
//==========================================================

#if USE_RTT_BACKEND
#ifndef NRF_LOG_BACKEND_RTT_ENABLED
#define NRF_LOG_BACKEND_RTT_ENABLED 1
#endif
#endif

#if NRF_LOG_BACKEND_RTT_ENABLED 

#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS    3       // Max. number of up-buffers (T->H) available on this target    (Default: 3)
#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS  3       // Max. number of down-buffers (H->T) available on this target  (Default: 3)

#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_UP
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 2048           // 1024
#endif

#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN 16
#endif

#ifndef SEGGER_RTT_CONFIG_DEFAULT_MODE
#define SEGGER_RTT_CONFIG_DEFAULT_MODE 1
#endif

#endif



// <o> NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE - Size of buffer for partially processed strings. 
// <i> Size of the buffer is a trade-off between RAM usage and processing.
// <i> if buffer is smaller then strings will often be fragmented.
// <i> It is recommended to use size which will fit typical log and only the
// <i> longer one will be fragmented.

#ifndef NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE
#define NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE 32
#endif

// <o> NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS - Period before retrying writing to RTT 
#ifndef NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS
#define NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS 1
#endif

// <o> NRF_LOG_BACKEND_RTT_TX_RETRY_CNT - Writing to RTT retries. 
// <i> If RTT fails to accept any new data after retries
// <i> module assumes that host is not active and on next
// <i> request it will perform only one write attempt.
// <i> On successful writing, module assumes that host is active
// <i> and scheme with retry is applied again.

#ifndef NRF_LOG_BACKEND_RTT_TX_RETRY_CNT
#define NRF_LOG_BACKEND_RTT_TX_RETRY_CNT 3
#endif

// </e>

// <e> NRF_LOG_BACKEND_UART_ENABLED - nrf_log_backend_uart - Log UART backend
//==========================================================

#if USE_USB_BACKEND
#ifndef NRF_LOG_BACKEND_UART_ENABLED
#define NRF_LOG_BACKEND_UART_ENABLED 0
#endif
#endif

#if NRF_LOG_BACKEND_UART_ENABLED
// <o> NRF_LOG_BACKEND_UART_TX_PIN - UART TX pin 
#ifndef NRF_LOG_BACKEND_UART_TX_PIN
#define NRF_LOG_BACKEND_UART_TX_PIN 6
#endif

// <o> NRF_LOG_BACKEND_UART_BAUDRATE  - Default Baudrate
 
// <323584=> 1200 baud 
// <643072=> 2400 baud 
// <1290240=> 4800 baud 
// <2576384=> 9600 baud 
// <3862528=> 14400 baud 
// <5152768=> 19200 baud 
// <7716864=> 28800 baud 
// <10289152=> 38400 baud 
// <15400960=> 57600 baud 
// <20615168=> 76800 baud 
// <30801920=> 115200 baud 
// <61865984=> 230400 baud 
// <67108864=> 250000 baud 
// <121634816=> 460800 baud 
// <251658240=> 921600 baud 
// <268435456=> 1000000 baud 

#ifndef NRF_LOG_BACKEND_UART_BAUDRATE
#define NRF_LOG_BACKEND_UART_BAUDRATE 30801920
#endif

// <o> NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE - Size of buffer for partially processed strings. 
// <i> Size of the buffer is a trade-off between RAM usage and processing.
// <i> if buffer is smaller then strings will often be fragmented.
// <i> It is recommended to use size which will fit typical log and only the
// <i> longer one will be fragmented.

#ifndef NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE
#define NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE 64
#endif

// </e>
#endif

// <h> Log message pool - Configuration of log message pool
//==========================================================
// <o> NRF_LOG_MSGPOOL_ELEMENT_SIZE - Size of a single element in the pool of memory objects. 
// <i> If a small value is set, then performance of logs processing
// <i> is degraded because data is fragmented. Bigger value impacts
// <i> RAM memory utilization. The size is set to fit a message with
// <i> a timestamp and up to 2 arguments in a single memory object.

#ifndef NRF_LOG_MSGPOOL_ELEMENT_SIZE
#define NRF_LOG_MSGPOOL_ELEMENT_SIZE 20
#endif

// <o> NRF_LOG_MSGPOOL_ELEMENT_COUNT - Number of elements in the pool of memory objects 
// <i> If a small value is set, then it may lead to a deadlock
// <i> in certain cases if backend has high latency and holds
// <i> multiple messages for long time. Bigger value impacts
// <i> RAM memory usage.

#ifndef NRF_LOG_MSGPOOL_ELEMENT_COUNT
#define NRF_LOG_MSGPOOL_ELEMENT_COUNT 8
#endif

// </h> 
//==========================================================

// <q> NRF_LOG_ALLOW_OVERFLOW  - Configures behavior when circular buffer is full.
 

// <i> If set then oldest logs are overwritten. Otherwise a 
// <i> marker is injected informing about overflow.

#ifndef NRF_LOG_ALLOW_OVERFLOW
#define NRF_LOG_ALLOW_OVERFLOW 1
#endif

// <o> NRF_LOG_BUFSIZE  - Size of the buffer for storing logs (in bytes).
 

// <i> Must be power of 2 and multiple of 4.
// <i> If NRF_LOG_DEFERRED = 0 then buffer size can be reduced to minimum.
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 
// <2048=> 2048 
// <4096=> 4096 
// <8192=> 8192 
// <16384=> 16384 

#ifndef NRF_LOG_BUFSIZE
#define NRF_LOG_BUFSIZE 128
#endif

// <q> NRF_LOG_CLI_CMDS  - Enable CLI commands for the module.
 

#ifndef NRF_LOG_CLI_CMDS
#define NRF_LOG_CLI_CMDS 0
#endif

// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL 4
#endif

// <q> NRF_LOG_DEFERRED  - Enable deffered logger.
 

// <i> Log data is buffered and can be processed in idle.
#ifndef NRF_LOG_DEFERRED
#define NRF_LOG_DEFERRED 0
#endif

// <q> NRF_LOG_FILTERS_ENABLED  - Enable dynamic filtering of logs.
#ifndef NRF_LOG_FILTERS_ENABLED
#define NRF_LOG_FILTERS_ENABLED 0
#endif

// <q> NRF_LOG_NON_DEFFERED_CRITICAL_REGION_ENABLED  - Enable use of critical region for non deffered mode when flushing logs.
 

// <i> When enabled NRF_LOG_FLUSH is called from critical section when non deffered mode is used.
// <i> Log output will never be corrupted as access to the log backend is exclusive
// <i> but system will spend significant amount of time in critical section

#ifndef NRF_LOG_NON_DEFFERED_CRITICAL_REGION_ENABLED
#define NRF_LOG_NON_DEFFERED_CRITICAL_REGION_ENABLED 0
#endif

// <o> NRF_LOG_STR_PUSH_BUFFER_SIZE  - Size of the buffer dedicated for strings stored using @ref NRF_LOG_PUSH.
 
// <16=> 16 
// <32=> 32 
// <64=> 64 
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 

#ifndef NRF_LOG_STR_PUSH_BUFFER_SIZE
#define NRF_LOG_STR_PUSH_BUFFER_SIZE 128
#endif

// <o> NRF_LOG_STR_PUSH_BUFFER_SIZE  - Size of the buffer dedicated for strings stored using @ref NRF_LOG_PUSH.
 
// <16=> 16 
// <32=> 32 
// <64=> 64 
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 

#ifndef NRF_LOG_STR_PUSH_BUFFER_SIZE
#define NRF_LOG_STR_PUSH_BUFFER_SIZE 128
#endif

// <e> NRF_LOG_USES_COLORS - If enabled then ANSI escape code for colors is prefixed to every string
//==========================================================
#ifndef NRF_LOG_USES_COLORS
#define NRF_LOG_USES_COLORS 0
#endif
// <o> NRF_LOG_COLOR_DEFAULT  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_COLOR_DEFAULT
#define NRF_LOG_COLOR_DEFAULT 0
#endif

// <o> NRF_LOG_ERROR_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_ERROR_COLOR
#define NRF_LOG_ERROR_COLOR 2
#endif

// <o> NRF_LOG_WARNING_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_WARNING_COLOR
#define NRF_LOG_WARNING_COLOR 4
#endif

// </e>

// <e> NRF_LOG_USES_TIMESTAMP - Enable timestamping

// <i> Function for getting the timestamp is provided by the user
//==========================================================
#ifndef NRF_LOG_USES_TIMESTAMP
#define NRF_LOG_USES_TIMESTAMP 0
#endif
// <o> NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY - Default frequency of the timestamp (in Hz) or 0 to use app_timer frequency. 
#ifndef NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY
#define NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY 0
#endif

// </e>

// <h> nrf_log module configuration 

//==========================================================
// <h> nrf_log in nRF_Core 

//==========================================================
// <e> NRF_MPU_LIB_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_MPU_LIB_CONFIG_LOG_ENABLED
#define NRF_MPU_LIB_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_MPU_LIB_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_MPU_LIB_CONFIG_LOG_LEVEL
#define NRF_MPU_LIB_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_MPU_LIB_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MPU_LIB_CONFIG_INFO_COLOR
#define NRF_MPU_LIB_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_MPU_LIB_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MPU_LIB_CONFIG_DEBUG_COLOR
#define NRF_MPU_LIB_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_STACK_GUARD_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_STACK_GUARD_CONFIG_LOG_ENABLED
#define NRF_STACK_GUARD_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_STACK_GUARD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_STACK_GUARD_CONFIG_LOG_LEVEL
#define NRF_STACK_GUARD_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_STACK_GUARD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_STACK_GUARD_CONFIG_INFO_COLOR
#define NRF_STACK_GUARD_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_STACK_GUARD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_STACK_GUARD_CONFIG_DEBUG_COLOR
#define NRF_STACK_GUARD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TASK_MANAGER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TASK_MANAGER_CONFIG_LOG_ENABLED
#define TASK_MANAGER_CONFIG_LOG_ENABLED 0
#endif
// <o> TASK_MANAGER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TASK_MANAGER_CONFIG_LOG_LEVEL
#define TASK_MANAGER_CONFIG_LOG_LEVEL 3
#endif

// <o> TASK_MANAGER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TASK_MANAGER_CONFIG_INFO_COLOR
#define TASK_MANAGER_CONFIG_INFO_COLOR 0
#endif

// <o> TASK_MANAGER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TASK_MANAGER_CONFIG_DEBUG_COLOR
#define TASK_MANAGER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Drivers 

//==========================================================
// <e> CLOCK_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef CLOCK_CONFIG_LOG_ENABLED
#define CLOCK_CONFIG_LOG_ENABLED 0
#endif
// <o> CLOCK_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef CLOCK_CONFIG_LOG_LEVEL
#define CLOCK_CONFIG_LOG_LEVEL 3
#endif

// <o> CLOCK_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef CLOCK_CONFIG_INFO_COLOR
#define CLOCK_CONFIG_INFO_COLOR 0
#endif

// <o> CLOCK_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef CLOCK_CONFIG_DEBUG_COLOR
#define CLOCK_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> COMP_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef COMP_CONFIG_LOG_ENABLED
#define COMP_CONFIG_LOG_ENABLED 0
#endif
// <o> COMP_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef COMP_CONFIG_LOG_LEVEL
#define COMP_CONFIG_LOG_LEVEL 3
#endif

// <o> COMP_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMP_CONFIG_INFO_COLOR
#define COMP_CONFIG_INFO_COLOR 0
#endif

// <o> COMP_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMP_CONFIG_DEBUG_COLOR
#define COMP_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> GPIOTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef GPIOTE_CONFIG_LOG_ENABLED
#define GPIOTE_CONFIG_LOG_ENABLED 0
#endif
// <o> GPIOTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef GPIOTE_CONFIG_LOG_LEVEL
#define GPIOTE_CONFIG_LOG_LEVEL 3
#endif

// <o> GPIOTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef GPIOTE_CONFIG_INFO_COLOR
#define GPIOTE_CONFIG_INFO_COLOR 0
#endif

// <o> GPIOTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef GPIOTE_CONFIG_DEBUG_COLOR
#define GPIOTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> LPCOMP_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef LPCOMP_CONFIG_LOG_ENABLED
#define LPCOMP_CONFIG_LOG_ENABLED 0
#endif
// <o> LPCOMP_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef LPCOMP_CONFIG_LOG_LEVEL
#define LPCOMP_CONFIG_LOG_LEVEL 3
#endif

// <o> LPCOMP_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef LPCOMP_CONFIG_INFO_COLOR
#define LPCOMP_CONFIG_INFO_COLOR 0
#endif

// <o> LPCOMP_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef LPCOMP_CONFIG_DEBUG_COLOR
#define LPCOMP_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> MAX3421E_HOST_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef MAX3421E_HOST_CONFIG_LOG_ENABLED
#define MAX3421E_HOST_CONFIG_LOG_ENABLED 0
#endif
// <o> MAX3421E_HOST_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef MAX3421E_HOST_CONFIG_LOG_LEVEL
#define MAX3421E_HOST_CONFIG_LOG_LEVEL 3
#endif

// <o> MAX3421E_HOST_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef MAX3421E_HOST_CONFIG_INFO_COLOR
#define MAX3421E_HOST_CONFIG_INFO_COLOR 0
#endif

// <o> MAX3421E_HOST_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef MAX3421E_HOST_CONFIG_DEBUG_COLOR
#define MAX3421E_HOST_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRFX_USBD_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef NRFX_USBD_CONFIG_LOG_ENABLED
#define NRFX_USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_USBD_CONFIG_LOG_LEVEL
#define NRFX_USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_USBD_CONFIG_INFO_COLOR
#define NRFX_USBD_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_USBD_CONFIG_DEBUG_COLOR
#define NRFX_USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PDM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PDM_CONFIG_LOG_ENABLED
#define PDM_CONFIG_LOG_ENABLED 0
#endif
// <o> PDM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PDM_CONFIG_LOG_LEVEL
#define PDM_CONFIG_LOG_LEVEL 3
#endif

// <o> PDM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PDM_CONFIG_INFO_COLOR
#define PDM_CONFIG_INFO_COLOR 0
#endif

// <o> PDM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PDM_CONFIG_DEBUG_COLOR
#define PDM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PPI_CONFIG_LOG_ENABLED
#define PPI_CONFIG_LOG_ENABLED 0
#endif
// <o> PPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PPI_CONFIG_LOG_LEVEL
#define PPI_CONFIG_LOG_LEVEL 3
#endif

// <o> PPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_INFO_COLOR
#define PPI_CONFIG_INFO_COLOR 0
#endif

// <o> PPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_DEBUG_COLOR
#define PPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PWM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PWM_CONFIG_LOG_ENABLED
#define PWM_CONFIG_LOG_ENABLED 0
#endif
// <o> PWM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PWM_CONFIG_LOG_LEVEL
#define PWM_CONFIG_LOG_LEVEL 3
#endif

// <o> PWM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PWM_CONFIG_INFO_COLOR
#define PWM_CONFIG_INFO_COLOR 0
#endif

// <o> PWM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PWM_CONFIG_DEBUG_COLOR
#define PWM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> QDEC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef QDEC_CONFIG_LOG_ENABLED
#define QDEC_CONFIG_LOG_ENABLED 0
#endif
// <o> QDEC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef QDEC_CONFIG_LOG_LEVEL
#define QDEC_CONFIG_LOG_LEVEL 3
#endif

// <o> QDEC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef QDEC_CONFIG_INFO_COLOR
#define QDEC_CONFIG_INFO_COLOR 0
#endif

// <o> QDEC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef QDEC_CONFIG_DEBUG_COLOR
#define QDEC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRFX_RNG_ENABLED - nrfx_rng - RNG peripheral driver
//==========================================================
#ifndef NRFX_RNG_ENABLED
#define NRFX_RNG_ENABLED 1
#endif
// <q> NRFX_RNG_CONFIG_ERROR_CORRECTION  - Error correction
 

#ifndef NRFX_RNG_CONFIG_ERROR_CORRECTION
#define NRFX_RNG_CONFIG_ERROR_CORRECTION 1
#endif

// <o> NRFX_RNG_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_RNG_CONFIG_IRQ_PRIORITY
#define NRFX_RNG_CONFIG_IRQ_PRIORITY 6
#endif

// <e> RNG_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef RNG_CONFIG_LOG_ENABLED
#define RNG_CONFIG_LOG_ENABLED 0
#endif
// <o> RNG_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef RNG_CONFIG_LOG_LEVEL
#define RNG_CONFIG_LOG_LEVEL 3
#endif

// <o> RNG_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RNG_CONFIG_INFO_COLOR
#define RNG_CONFIG_INFO_COLOR 0
#endif

// <o> RNG_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RNG_CONFIG_DEBUG_COLOR
#define RNG_CONFIG_DEBUG_COLOR 0
#endif

// <q> RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED  - Enables logging of random numbers.
 

#ifndef RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED
#define RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED 0
#endif

// </e>

// <e> RTC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef RTC_CONFIG_LOG_ENABLED
#define RTC_CONFIG_LOG_ENABLED 0
#endif
// <o> RTC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef RTC_CONFIG_LOG_LEVEL
#define RTC_CONFIG_LOG_LEVEL 3
#endif

// <o> RTC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RTC_CONFIG_INFO_COLOR
#define RTC_CONFIG_INFO_COLOR 0
#endif

// <o> RTC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RTC_CONFIG_DEBUG_COLOR
#define RTC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> SAADC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SAADC_CONFIG_LOG_ENABLED
#define SAADC_CONFIG_LOG_ENABLED 0
#endif
// <o> SAADC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SAADC_CONFIG_LOG_LEVEL
#define SAADC_CONFIG_LOG_LEVEL 3
#endif

// <o> SAADC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SAADC_CONFIG_INFO_COLOR
#define SAADC_CONFIG_INFO_COLOR 0
#endif

// <o> SAADC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SAADC_CONFIG_DEBUG_COLOR
#define SAADC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> SPIS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SPIS_CONFIG_LOG_ENABLED
#define SPIS_CONFIG_LOG_ENABLED 0
#endif
// <o> SPIS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SPIS_CONFIG_LOG_LEVEL
#define SPIS_CONFIG_LOG_LEVEL 3
#endif

// <o> SPIS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPIS_CONFIG_INFO_COLOR
#define SPIS_CONFIG_INFO_COLOR 0
#endif

// <o> SPIS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPIS_CONFIG_DEBUG_COLOR
#define SPIS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> SPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SPI_CONFIG_LOG_ENABLED
#define SPI_CONFIG_LOG_ENABLED 0
#endif
// <o> SPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SPI_CONFIG_LOG_LEVEL
#define SPI_CONFIG_LOG_LEVEL 3
#endif

// <o> SPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_INFO_COLOR
#define SPI_CONFIG_INFO_COLOR 0
#endif

// <o> SPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_DEBUG_COLOR
#define SPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TIMER_CONFIG_LOG_ENABLED
#define TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TIMER_CONFIG_LOG_LEVEL
#define TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_INFO_COLOR
#define TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_DEBUG_COLOR
#define TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TWIS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TWIS_CONFIG_LOG_ENABLED
#define TWIS_CONFIG_LOG_ENABLED 0
#endif
// <o> TWIS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TWIS_CONFIG_LOG_LEVEL
#define TWIS_CONFIG_LOG_LEVEL 3
#endif

// <o> TWIS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWIS_CONFIG_INFO_COLOR
#define TWIS_CONFIG_INFO_COLOR 0
#endif

// <o> TWIS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWIS_CONFIG_DEBUG_COLOR
#define TWIS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TWI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TWI_CONFIG_LOG_ENABLED
#define TWI_CONFIG_LOG_ENABLED 0
#endif
// <o> TWI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TWI_CONFIG_LOG_LEVEL
#define TWI_CONFIG_LOG_LEVEL 3
#endif

// <o> TWI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWI_CONFIG_INFO_COLOR
#define TWI_CONFIG_INFO_COLOR 0
#endif

// <o> TWI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWI_CONFIG_DEBUG_COLOR
#define TWI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef UART_CONFIG_LOG_ENABLED
#define UART_CONFIG_LOG_ENABLED 0
#endif
// <o> UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef UART_CONFIG_LOG_LEVEL
#define UART_CONFIG_LOG_LEVEL 3
#endif

// <o> UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_INFO_COLOR
#define UART_CONFIG_INFO_COLOR 0
#endif

// <o> UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_DEBUG_COLOR
#define UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> USBD_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef USBD_CONFIG_LOG_ENABLED
#define USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef USBD_CONFIG_LOG_LEVEL
#define USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef USBD_CONFIG_INFO_COLOR
#define USBD_CONFIG_INFO_COLOR 0
#endif

// <o> USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef USBD_CONFIG_DEBUG_COLOR
#define USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> WDT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef WDT_CONFIG_LOG_ENABLED
#define WDT_CONFIG_LOG_ENABLED 0
#endif
// <o> WDT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef WDT_CONFIG_LOG_LEVEL
#define WDT_CONFIG_LOG_LEVEL 3
#endif

// <o> WDT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef WDT_CONFIG_INFO_COLOR
#define WDT_CONFIG_INFO_COLOR 0
#endif

// <o> WDT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef WDT_CONFIG_DEBUG_COLOR
#define WDT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Libraries 

//==========================================================
// <e> APP_BUTTON_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_BUTTON_CONFIG_LOG_ENABLED
#define APP_BUTTON_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_BUTTON_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_BUTTON_CONFIG_LOG_LEVEL
#define APP_BUTTON_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_BUTTON_CONFIG_INITIAL_LOG_LEVEL  - Initial severity level if dynamic filtering is enabled.
 

// <i> If module generates a lot of logs, initial log level can
// <i> be decreased to prevent flooding. Severity level can be
// <i> increased on instance basis.
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_BUTTON_CONFIG_INITIAL_LOG_LEVEL
#define APP_BUTTON_CONFIG_INITIAL_LOG_LEVEL 3
#endif

// <o> APP_BUTTON_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_BUTTON_CONFIG_INFO_COLOR
#define APP_BUTTON_CONFIG_INFO_COLOR 0
#endif

// <o> APP_BUTTON_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_BUTTON_CONFIG_DEBUG_COLOR
#define APP_BUTTON_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_TIMER_CONFIG_LOG_ENABLED
#define APP_TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_TIMER_CONFIG_LOG_LEVEL
#define APP_TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_TIMER_CONFIG_INITIAL_LOG_LEVEL  - Initial severity level if dynamic filtering is enabled.
 

// <i> If module generates a lot of logs, initial log level can
// <i> be decreased to prevent flooding. Severity level can be
// <i> increased on instance basis.
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_TIMER_CONFIG_INITIAL_LOG_LEVEL
#define APP_TIMER_CONFIG_INITIAL_LOG_LEVEL 3
#endif

// <o> APP_TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_TIMER_CONFIG_INFO_COLOR
#define APP_TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> APP_TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_TIMER_CONFIG_DEBUG_COLOR
#define APP_TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED
#define APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL
#define APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_CDC_ACM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CDC_ACM_CONFIG_INFO_COLOR
#define APP_USBD_CDC_ACM_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR
#define APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_CONFIG_LOG_ENABLED - Enable logging in the module.
//==========================================================
#ifndef APP_USBD_CONFIG_LOG_ENABLED
#define APP_USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_CONFIG_LOG_LEVEL
#define APP_USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CONFIG_INFO_COLOR
#define APP_USBD_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CONFIG_DEBUG_COLOR
#define APP_USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_DUMMY_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_DUMMY_CONFIG_LOG_ENABLED
#define APP_USBD_DUMMY_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_DUMMY_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_DUMMY_CONFIG_LOG_LEVEL
#define APP_USBD_DUMMY_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_DUMMY_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_DUMMY_CONFIG_INFO_COLOR
#define APP_USBD_DUMMY_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_DUMMY_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_DUMMY_CONFIG_DEBUG_COLOR
#define APP_USBD_DUMMY_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_MSC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_MSC_CONFIG_LOG_ENABLED
#define APP_USBD_MSC_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_MSC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_MSC_CONFIG_LOG_LEVEL
#define APP_USBD_MSC_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_MSC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_MSC_CONFIG_INFO_COLOR
#define APP_USBD_MSC_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_MSC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_MSC_CONFIG_DEBUG_COLOR
#define APP_USBD_MSC_CONFIG_DEBUG_COLOR 0lll
#endif

// </e>

// <e> APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_ATFIFO_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_ATFIFO_CONFIG_LOG_ENABLED
#define NRF_ATFIFO_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_ATFIFO_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_ATFIFO_CONFIG_LOG_LEVEL
#define NRF_ATFIFO_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_ATFIFO_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_ATFIFO_CONFIG_INFO_COLOR
#define NRF_ATFIFO_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_ATFIFO_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_ATFIFO_CONFIG_DEBUG_COLOR
#define NRF_ATFIFO_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BALLOC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BALLOC_CONFIG_LOG_ENABLED
#define NRF_BALLOC_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BALLOC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BALLOC_CONFIG_LOG_LEVEL
#define NRF_BALLOC_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL  - Initial severity level if dynamic filtering is enabled.
 

// <i> If module generates a lot of logs, initial log level can
// <i> be decreased to prevent flooding. Severity level can be
// <i> increased on instance basis.
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL
#define NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL 3
#endif

// <o> NRF_BALLOC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BALLOC_CONFIG_INFO_COLOR
#define NRF_BALLOC_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BALLOC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BALLOC_CONFIG_DEBUG_COLOR
#define NRF_BALLOC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED
#define NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL
#define NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_BLE_UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_BLE_UART_CONFIG_INFO_COLOR
#define NRF_CLI_BLE_UART_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR
#define NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL
#define NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR
#define NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR
#define NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_UART_CONFIG_LOG_ENABLED
#define NRF_CLI_UART_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_UART_CONFIG_LOG_LEVEL
#define NRF_CLI_UART_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_UART_CONFIG_INFO_COLOR
#define NRF_CLI_UART_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_UART_CONFIG_DEBUG_COLOR
#define NRF_CLI_UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_LIBUARTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_LIBUARTE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_LIBUARTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_LIBUARTE_CONFIG_LOG_LEVEL
#define NRF_LIBUARTE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_LIBUARTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LIBUARTE_CONFIG_INFO_COLOR
#define NRF_LIBUARTE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_LIBUARTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LIBUARTE_CONFIG_DEBUG_COLOR
#define NRF_LIBUARTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_MEMOBJ_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_MEMOBJ_CONFIG_LOG_ENABLED
#define NRF_MEMOBJ_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_MEMOBJ_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_MEMOBJ_CONFIG_LOG_LEVEL
#define NRF_MEMOBJ_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_MEMOBJ_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MEMOBJ_CONFIG_INFO_COLOR
#define NRF_MEMOBJ_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_MEMOBJ_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MEMOBJ_CONFIG_DEBUG_COLOR
#define NRF_MEMOBJ_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_PWR_MGMT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_PWR_MGMT_CONFIG_LOG_ENABLED
#define NRF_PWR_MGMT_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_PWR_MGMT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_PWR_MGMT_CONFIG_LOG_LEVEL
#define NRF_PWR_MGMT_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_PWR_MGMT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_PWR_MGMT_CONFIG_INFO_COLOR
#define NRF_PWR_MGMT_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_PWR_MGMT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_PWR_MGMT_CONFIG_DEBUG_COLOR
#define NRF_PWR_MGMT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_QUEUE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_QUEUE_CONFIG_LOG_ENABLED
#define NRF_QUEUE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_QUEUE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_LEVEL
#define NRF_QUEUE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_INFO_COLOR
#define NRF_QUEUE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_QUEUE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_DEBUG_COLOR
#define NRF_QUEUE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_ANT_LOG_ENABLED - Enable logging in SoftDevice handler (ANT) module.
//==========================================================
#ifndef NRF_SDH_ANT_LOG_ENABLED
#define NRF_SDH_ANT_LOG_ENABLED 0
#endif
// <o> NRF_SDH_ANT_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_ANT_LOG_LEVEL
#define NRF_SDH_ANT_LOG_LEVEL 3
#endif

// <o> NRF_SDH_ANT_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_ANT_INFO_COLOR
#define NRF_SDH_ANT_INFO_COLOR 0
#endif

// <o> NRF_SDH_ANT_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_ANT_DEBUG_COLOR
#define NRF_SDH_ANT_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_BLE_LOG_ENABLED - Enable logging in SoftDevice handler (BLE) module.
//==========================================================
#ifndef NRF_SDH_BLE_LOG_ENABLED
#define NRF_SDH_BLE_LOG_ENABLED 0
#endif
// <o> NRF_SDH_BLE_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_BLE_LOG_LEVEL
#define NRF_SDH_BLE_LOG_LEVEL 3
#endif

// <o> NRF_SDH_BLE_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_INFO_COLOR
#define NRF_SDH_BLE_INFO_COLOR 0
#endif

// <o> NRF_SDH_BLE_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_DEBUG_COLOR
#define NRF_SDH_BLE_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_LOG_ENABLED - Enable logging in SoftDevice handler module.
//==========================================================
#ifndef NRF_SDH_LOG_ENABLED
#define NRF_SDH_LOG_ENABLED 0
#endif
// <o> NRF_SDH_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_LOG_LEVEL
#define NRF_SDH_LOG_LEVEL 3
#endif

// <o> NRF_SDH_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_INFO_COLOR
#define NRF_SDH_INFO_COLOR 0
#endif

// <o> NRF_SDH_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_DEBUG_COLOR
#define NRF_SDH_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_SOC_LOG_ENABLED - Enable logging in SoftDevice handler (SoC) module.
//==========================================================
#ifndef NRF_SDH_SOC_LOG_ENABLED
#define NRF_SDH_SOC_LOG_ENABLED 0
#endif
// <o> NRF_SDH_SOC_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_SOC_LOG_LEVEL
#define NRF_SDH_SOC_LOG_LEVEL 3
#endif

// <o> NRF_SDH_SOC_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_SOC_INFO_COLOR
#define NRF_SDH_SOC_INFO_COLOR 0
#endif

// <o> NRF_SDH_SOC_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_SOC_DEBUG_COLOR
#define NRF_SDH_SOC_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SORTLIST_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_SORTLIST_CONFIG_LOG_ENABLED
#define NRF_SORTLIST_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_SORTLIST_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SORTLIST_CONFIG_LOG_LEVEL
#define NRF_SORTLIST_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_SORTLIST_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SORTLIST_CONFIG_INFO_COLOR
#define NRF_SORTLIST_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_SORTLIST_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SORTLIST_CONFIG_DEBUG_COLOR
#define NRF_SORTLIST_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_TWI_SENSOR_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_TWI_SENSOR_CONFIG_LOG_ENABLED
#define NRF_TWI_SENSOR_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_TWI_SENSOR_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_TWI_SENSOR_CONFIG_LOG_LEVEL
#define NRF_TWI_SENSOR_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_TWI_SENSOR_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_TWI_SENSOR_CONFIG_INFO_COLOR
#define NRF_TWI_SENSOR_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR
#define NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PM_LOG_ENABLED - Enable logging in Peer Manager and its submodules.
//==========================================================
#ifndef PM_LOG_ENABLED
#define PM_LOG_ENABLED 0
#endif
// <o> PM_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PM_LOG_LEVEL
#define PM_LOG_LEVEL 3
#endif

// <o> PM_LOG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PM_LOG_INFO_COLOR
#define PM_LOG_INFO_COLOR 0
#endif

// <o> PM_LOG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PM_LOG_DEBUG_COLOR
#define PM_LOG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Serialization 

//==========================================================
// <e> SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED
#define SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL
#define SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL 3
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_INFO_COLOR
#define SER_HAL_TRANSPORT_CONFIG_INFO_COLOR 0
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR
#define SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// </h> 
//==========================================================

// </e>

// <q> NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED  - nrf_log_str_formatter - Log string formatter
 

#ifndef NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED
#define NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED 1
#endif

// </h> 
//==========================================================


// <e> APP_USBD_ENABLED - app_usbd - USB Device library
//==========================================================
#ifndef APP_USBD_ENABLED
#define APP_USBD_ENABLED 1
#endif
// <s> APP_USBD_VID - Vendor ID.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Vendor ID ordered from USB IF: http://www.usb.org/developers/vendor/
#ifndef APP_USBD_VID
#define APP_USBD_VID 0x1915
#endif

// <s> APP_USBD_PID - Product ID.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Selected Product ID
#ifndef APP_USBD_PID
#define APP_USBD_PID 0x520F
#endif

// <o> APP_USBD_DEVICE_VER_MAJOR - Major device version  <0-99> 


// <i> Major device version, will be converted automatically to BCD notation. Use just decimal values.

#ifndef APP_USBD_DEVICE_VER_MAJOR
#define APP_USBD_DEVICE_VER_MAJOR 1
#endif

// <o> APP_USBD_DEVICE_VER_MINOR - Minor device version  <0-9> 


// <i> Minor device version, will be converted automatically to BCD notation. Use just decimal values.

#ifndef APP_USBD_DEVICE_VER_MINOR
#define APP_USBD_DEVICE_VER_MINOR 0
#endif

// <o> APP_USBD_DEVICE_VER_SUB - Sub-minor device version  <0-9> 


// <i> Sub-minor device version, will be converted automatically to BCD notation. Use just decimal values.

#ifndef APP_USBD_DEVICE_VER_SUB
#define APP_USBD_DEVICE_VER_SUB 0
#endif

// <q> APP_USBD_CONFIG_SELF_POWERED  - Self-powered device, as opposed to bus-powered.
 

#ifndef APP_USBD_CONFIG_SELF_POWERED
#define APP_USBD_CONFIG_SELF_POWERED 1
#endif

// <o> APP_USBD_CONFIG_MAX_POWER - MaxPower field in configuration descriptor in milliamps.  <0-500> 


#ifndef APP_USBD_CONFIG_MAX_POWER
#define APP_USBD_CONFIG_MAX_POWER 100
#endif

// <q> APP_USBD_CONFIG_POWER_EVENTS_PROCESS  - Process power events.
 

// <i> Enable processing power events in USB event handler.

#ifndef APP_USBD_CONFIG_POWER_EVENTS_PROCESS
#define APP_USBD_CONFIG_POWER_EVENTS_PROCESS 1
#endif

// <e> APP_USBD_CONFIG_EVENT_QUEUE_ENABLE - Enable event queue.

// <i> This is the default configuration when all the events are placed into internal queue.
// <i> Disable it when an external queue is used like app_scheduler or if you wish to process all events inside interrupts.
// <i> Processing all events from the interrupt level adds requirement not to call any functions that modifies the USBD library state from the context higher than USB interrupt context.
// <i> Functions that modify USBD state are functions for sleep, wakeup, start, stop, enable, and disable.
//==========================================================
#ifndef APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
#define APP_USBD_CONFIG_EVENT_QUEUE_ENABLE 0
#endif
// <o> APP_USBD_CONFIG_EVENT_QUEUE_SIZE - The size of the event queue.  <16-64> 


// <i> The size of the queue for the events that would be processed in the main loop.

#ifndef APP_USBD_CONFIG_EVENT_QUEUE_SIZE
#define APP_USBD_CONFIG_EVENT_QUEUE_SIZE 32
#endif

// <o> APP_USBD_CONFIG_SOF_HANDLING_MODE  - Change SOF events handling mode.
 

// <i> Normal queue   - SOF events are pushed normally into the event queue.
// <i> Compress queue - SOF events are counted and binded with other events or executed when the queue is empty.
// <i>                  This prevents the queue from filling up with SOF events.
// <i> Interrupt      - SOF events are processed in interrupt.
// <0=> Normal queue 
// <1=> Compress queue 
// <2=> Interrupt 

#ifndef APP_USBD_CONFIG_SOF_HANDLING_MODE
#define APP_USBD_CONFIG_SOF_HANDLING_MODE 1
#endif

// </e>

// <q> APP_USBD_CONFIG_SOF_TIMESTAMP_PROVIDE  - Provide a function that generates timestamps for logs based on the current SOF.
 

// <i> The function app_usbd_sof_timestamp_get is implemented if the logger is enabled. 
// <i> Use it when initializing the logger. 
// <i> SOF processing is always enabled when this configuration parameter is active. 
// <i> Note: This option is configured outside of APP_USBD_CONFIG_LOG_ENABLED. 
// <i> This means that it works even if the logging in this very module is disabled. 

#ifndef APP_USBD_CONFIG_SOF_TIMESTAMP_PROVIDE
#define APP_USBD_CONFIG_SOF_TIMESTAMP_PROVIDE 0
#endif

// <o> APP_USBD_CONFIG_DESC_STRING_SIZE - Maximum size of the NULL-terminated string of the string descriptor.  <31-254> 


// <i> 31 characters can be stored in the internal USB buffer used for transfers.
// <i> Any value higher than 31 creates an additional buffer just for descriptor strings.

#ifndef APP_USBD_CONFIG_DESC_STRING_SIZE
#define APP_USBD_CONFIG_DESC_STRING_SIZE 31
#endif

// <q> APP_USBD_CONFIG_DESC_STRING_UTF_ENABLED  - Enable UTF8 conversion.
 

// <i> Enable UTF8-encoded characters. In normal processing, only ASCII characters are available.

#ifndef APP_USBD_CONFIG_DESC_STRING_UTF_ENABLED
#define APP_USBD_CONFIG_DESC_STRING_UTF_ENABLED 0
#endif

// <s> APP_USBD_STRINGS_LANGIDS - Supported languages identifiers.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Comma-separated list of supported languages.
#ifndef APP_USBD_STRINGS_LANGIDS
#define APP_USBD_STRINGS_LANGIDS APP_USBD_LANG_AND_SUBLANG(APP_USBD_LANG_ENGLISH, APP_USBD_SUBLANG_ENGLISH_US)
#endif

// <e> APP_USBD_STRING_ID_MANUFACTURER - Define manufacturer string ID.

// <i> Setting ID to 0 disables the string.
//==========================================================
#ifndef APP_USBD_STRING_ID_MANUFACTURER
#define APP_USBD_STRING_ID_MANUFACTURER 1
#endif
// <q> APP_USBD_STRINGS_MANUFACTURER_EXTERN  - Define whether @ref APP_USBD_STRINGS_MANUFACTURER is created by macro or declared as a global variable.
 

#ifndef APP_USBD_STRINGS_MANUFACTURER_EXTERN
#define APP_USBD_STRINGS_MANUFACTURER_EXTERN 0
#endif

// <s> APP_USBD_STRINGS_MANUFACTURER - String descriptor for the manufacturer name.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Comma-separated list of manufacturer names for each defined language.
// <i> Use @ref APP_USBD_STRING_DESC macro to create string descriptor from a NULL-terminated string.
// <i> Use @ref APP_USBD_STRING_RAW8_DESC macro to create string descriptor from comma-separated uint8_t values.
// <i> Use @ref APP_USBD_STRING_RAW16_DESC macro to create string descriptor from comma-separated uint16_t values.
// <i> Alternatively, configure the macro to point to any internal variable pointer that already contains the descriptor.
// <i> Setting string to NULL disables that string.
// <i> The order of manufacturer names must be the same like in @ref APP_USBD_STRINGS_LANGIDS.
#ifndef APP_USBD_STRINGS_MANUFACTURER
#define APP_USBD_STRINGS_MANUFACTURER APP_USBD_STRING_DESC("Nordic Semiconductor")
#endif

// </e>

// <e> APP_USBD_STRING_ID_PRODUCT - Define product string ID.

// <i> Setting ID to 0 disables the string.
//==========================================================
#ifndef APP_USBD_STRING_ID_PRODUCT
#define APP_USBD_STRING_ID_PRODUCT 2
#endif
// <q> APP_USBD_STRINGS_PRODUCT_EXTERN  - Define whether @ref APP_USBD_STRINGS_PRODUCT is created by macro or declared as a global variable.
 

#ifndef APP_USBD_STRINGS_PRODUCT_EXTERN
#define APP_USBD_STRINGS_PRODUCT_EXTERN 0
#endif

// <s> APP_USBD_STRINGS_PRODUCT - String descriptor for the product name.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> List of product names that is defined the same way like in @ref APP_USBD_STRINGS_MANUFACTURER.
#ifndef APP_USBD_STRINGS_PRODUCT
#define APP_USBD_STRINGS_PRODUCT APP_USBD_STRING_DESC("nRF52 USB CDC Demo")
#endif

// </e>

// <e> APP_USBD_STRING_ID_SERIAL - Define serial number string ID.

// <i> Setting ID to 0 disables the string.
//==========================================================
#ifndef APP_USBD_STRING_ID_SERIAL
#define APP_USBD_STRING_ID_SERIAL 3
#endif
// <q> APP_USBD_STRING_SERIAL_EXTERN  - Define whether @ref APP_USBD_STRING_SERIAL is created by macro or declared as a global variable.
 

#ifndef APP_USBD_STRING_SERIAL_EXTERN
#define APP_USBD_STRING_SERIAL_EXTERN 0
#endif

// <s> APP_USBD_STRING_SERIAL - String descriptor for the serial number.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Serial number that is defined the same way like in @ref APP_USBD_STRINGS_MANUFACTURER.
#ifndef APP_USBD_STRING_SERIAL
#define APP_USBD_STRING_SERIAL APP_USBD_STRING_DESC("000000000000")
#endif

// </e>

// <e> APP_USBD_STRING_ID_CONFIGURATION - Define configuration string ID.

// <i> Setting ID to 0 disables the string.
//==========================================================
#ifndef APP_USBD_STRING_ID_CONFIGURATION
#define APP_USBD_STRING_ID_CONFIGURATION 4
#endif
// <q> APP_USBD_STRING_CONFIGURATION_EXTERN  - Define whether @ref APP_USBD_STRINGS_CONFIGURATION is created by macro or declared as global variable.
 

#ifndef APP_USBD_STRING_CONFIGURATION_EXTERN
#define APP_USBD_STRING_CONFIGURATION_EXTERN 0
#endif

// <s> APP_USBD_STRINGS_CONFIGURATION - String descriptor for the device configuration.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> Configuration string that is defined the same way like in @ref APP_USBD_STRINGS_MANUFACTURER.
#ifndef APP_USBD_STRINGS_CONFIGURATION
#define APP_USBD_STRINGS_CONFIGURATION APP_USBD_STRING_DESC("Default configuration")
#endif

// </e>

// <s> APP_USBD_STRINGS_USER - Default values for user strings.

// <i> Note: This value is not editable in Configuration Wizard.
// <i> This value stores all application specific user strings with the default initialization.
// <i> The setup is done by X-macros.
// <i> Expected macro parameters:
// <i> @code
// <i> X(mnemonic, [=str_idx], ...)
// <i> @endcode
// <i> - @c mnemonic: Mnemonic of the string descriptor that would be added to
// <i>                @ref app_usbd_string_desc_idx_t enumerator.
// <i> - @c str_idx : String index value, can be set or left empty.
// <i>                For example, WinUSB driver requires descriptor to be present on 0xEE index.
// <i>                Then use X(USBD_STRING_WINUSB, =0xEE, (APP_USBD_STRING_DESC(...)))
// <i> - @c ...     : List of string descriptors for each defined language.
#ifndef APP_USBD_STRINGS_USER
#define APP_USBD_STRINGS_USER X(APP_USER_1, , APP_USBD_STRING_DESC("User 1"))
#endif

// </e>

















// <<< end of configuration section >>>
#endif //SDK_CONFIG_H
