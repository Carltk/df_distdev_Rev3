/**
 * Datafuel clone of Nordic nrf_libuarte_drv
 *
 * functionality added to allow better control for the Datafuel DDPC application
 *
 *
 * Copyright (c) 2018 - 2020, Nordic Semiconductor ASA
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
#ifndef DF_LIBUARTE_DRV_H
#define DF_LIBUARTE_DRV_H

#include "sdk_errors.h"
#include "nrfx_uart.h"

#include <stdint.h>
#include <stdbool.h>

#include "../hardware_init.h"

/**
 * @defgroup nrf_libuarte_drv libUARTE driver
 * @ingroup app_common
 *
 * @brief Module for reliable communication over UART.
 *
 * @{
 */

/* Number of bytes available in the buffer when RTS line is set. */
#define NRF_LIBUARTE_DRV_HWFC_BYTE_LIMIT 4

#define STUFF_CHAR      0xD0
#define SOH_CHAR        0xD1
#define EOF_CHAR        0xD2
#define MSG_DELIM       0xD3
#define MSG_MIN_SIZE    5
#define START_LRC_CALC  1  

typedef enum                            // Usage of counter-compare Channels in the Timer/Counter
{   DF_COUNT_CHAN_RXD_CHARS = 0,        
    DF_COUNT_CHAN_RXD_TOTAL,
    //DF_COUNT_CHAN_CHARS_AT_RTS,
    //DF_COUNT_CHAN_UNUSED
} df_timer_chan_t;

typedef enum
{   NRF_LIBUARTE_DRV_EVT_RX_DATA,       ///< Data received.
    NRF_LIBUARTE_DRV_EVT_TX_DONE,       ///< Requested TX transfer completed.
    NRF_LIBUARTE_DRV_EVT_ERROR,         ///< Error reported by the UARTE peripheral.
    NRF_LIBUARTE_DRV_EVT_OVERRUN_ERROR, ///< Error reported by the driver.
    NRF_LIBUARTE_DRV_EVT_GOT_PACKET,    ///< Have received a datafuel Console comms packet
} nrf_libuarte_drv_evt_type_t;

typedef struct {         // generic data structure (pointer to data and size)
    uint8_t  * p_data;  ///< Pointer to the data to be sent or received.
    size_t     size;    ///< Size of the data buffer
} nrf_libuarte_drv_data_t;

typedef struct  {   
    uint32_t overrun_length;
} nrf_libuarte_drv_overrun_err_evt_t;

typedef struct  {   
    nrf_libuarte_drv_evt_type_t type; ///< Event type.
    
    union {
        nrf_libuarte_drv_data_t rxtx;     ///< Data provided for transfer completion events.
        uint8_t                 errorsrc; ///< Error source flags.
        nrf_libuarte_drv_overrun_err_evt_t overrun_err; ///< SW Error structure.
    } data;
} nrf_libuarte_drv_evt_t;

typedef struct {
    uint32_t             tx_pin;        ///< TXD pin number.
    uint32_t             rx_pin;        ///< RXD pin number.
    uint32_t             cts_pin;       ///< CTS pin number.
    uint32_t             rts_pin;       ///< RTS pin number.
    uint32_t             txen_pin;      ///< TxEnable pin
    nrf_uart_parity_t    parity;        ///< Parity configuration.
    nrf_uart_baudrate_t  baudrate;      ///< Baud rate.
    uint8_t              irq_priority;  ///< Interrupt priority.
    bool                 pullup_rx;     ///< Pull up on RX pin.
    nrf_libuarte_drv_data_t rx_in_buf;     ///<buffer for incomming chars
    nrf_libuarte_drv_data_t packet_buf;    ///<buffer for transferring packets
} nrf_libuarte_drv_config_t;

typedef void (*nrf_libuarte_drv_evt_handler_t)(void * p_context, nrf_libuarte_drv_evt_t * p_evt);

typedef struct {
    bool got_SOH;
    bool got_stuff;
    uint8_t in_buf_ptr;
    
    size_t packet_length;
    bool checksum_ok;
    bool tx_reflection;

    // Temporary vaars
    uint32_t SOH_ms;         // TODO Remove this
    uint32_t EOF_ms;         // TODO Remove this

} df_packet_ctl_t;


typedef struct {
    uint32_t total_chars;
    uint32_t SOH_cnt;
    uint32_t EOF_cnt;
    uint32_t err_cnt;
    uint32_t good_packets;
    uint32_t total_packets;
    uint32_t elapsed_ms;     // TODO Remove this
} df_packet_stats_t;

typedef struct {
    uint32_t  txen_pin;                         // The Transmit Enable pin
    
    // Tx buffer management
    nrf_libuarte_drv_data_t tx_out_buf;
    size_t    tx_cur_idx;

    // Rx incomming buffer
    nrf_libuarte_drv_data_t rx_in_buf;          //<buffer for incomming chars

    // Packet buffer management
    nrf_libuarte_drv_data_t packet_buf;         //<buffer for transferring packets

    df_packet_ctl_t packet_ctl;                 // control of packets    
    df_packet_stats_t packet_stats;             // status of packets for external use

    nrf_libuarte_drv_evt_handler_t evt_handler;
    void * p_context;                           // context block that can be passed in a config
    
    bool enabled;
} nrf_libuarte_drv_ctrl_blk_t;

typedef struct {
    nrf_libuarte_drv_ctrl_blk_t * ctrl_blk;
    nrfx_uart_t const uarta;
} nrf_libuarte_drv_t;


/*
#define NRF_LIBUARTE_DRV_DEFINE(_name, _uart_idx, _timer_idx) \
    STATIC_ASSERT(_uart_idx < UART_COUNT, "UART instance not present");\
    STATIC_ASSERT(CONCAT_2(NRF_LIBUARTE_DRV_UART, _uart_idx) == 1, "UARTE instance not enabled");\
    STATIC_ASSERT(CONCAT_3(NRFX_TIMER,_timer_idx, _ENABLED) == 1, "Timer instance not enabled");\
    static nrf_libuarte_drv_ctrl_blk_t CONCAT_2(_name, ctrl_blk); \
    static const nrf_libuarte_drv_t _name = { \
        .ctrl_blk = &CONCAT_2(_name, ctrl_blk), \
        .timer = NRFX_TIMER_INSTANCE(_timer_idx), \
        .uart = CONCAT_2(NRF_UART, _uart_idx),\
    }
*/

/*
#define NRF_LIBUARTE_DRV_DEFINE(_name, _uarte_idx, _timer_idx)              \
    STATIC_ASSERT(_uarte_idx < UARTE_COUNT, "UARTE instance not present");  \
    STATIC_ASSERT(CONCAT_2(NRF_LIBUARTE_DRV_UART,_uarte_idx) == 1, "UARTE instance not enabled");   \
    STATIC_ASSERT(CONCAT_3(NRFX_TIMER,_timer_idx, _ENABLED) == 1, "Timer instance not enabled");    \
    static nrf_libuarte_drv_ctrl_blk_t CONCAT_2(_name, ctrl_blk);           \
    static const nrf_libuarte_drv_t _name = {                               \
        .ctrl_blk = &CONCAT_2(_name, ctrl_blk),                             \
        .uarta = {                                                          \
            .p_reg = &CONCAT_2(NRF_UART, _uarte_idx),                       \
            .drv_inst_idx = _uarte_idx                                      \
        }                                                                   \
    }
*/

#define NRF_LIBUARTE_DRV_DEFINE(_name, _uarte_idx, _timer_idx)              \
    STATIC_ASSERT(_uarte_idx < UARTE_COUNT, "UARTE instance not present");  \
    STATIC_ASSERT(CONCAT_2(NRF_LIBUARTE_DRV_UART,_uarte_idx) == 1, "UARTE instance not enabled");   \
    STATIC_ASSERT(CONCAT_3(NRFX_TIMER,_timer_idx, _ENABLED) == 1, "Timer instance not enabled");    \
    static nrf_libuarte_drv_ctrl_blk_t CONCAT_2(_name, ctrl_blk);           \
    static const nrf_libuarte_drv_t _name = {                               \
        .ctrl_blk = &CONCAT_2(_name, ctrl_blk),                             \
        .uarta = NRFX_UART_INSTANCE(_uarte_idx),                            \
    }




/**
 * @brief Function for initializing the libUARTE library.
 *
 * @param[in] p_libuarte   Pointer to libuarte instance.
 * @param[in] p_config     Pointer to the structure with initial configuration.
 * @param[in] evt_handler  Event handler provided by the user. Must not be NULL.
 * @param[in] context      User context passed in the callback.
 *
 * @return NRF_SUCCESS when properly initialized. NRF_ERROR_INTERNAL otherwise.
 */
ret_code_t nrf_libuarte_drv_init(const nrf_libuarte_drv_t * const p_libuarte,
                                 nrf_libuarte_drv_config_t * p_config,
                                 nrf_libuarte_drv_evt_handler_t evt_handler, void * p_context);

/**
 * @brief Function for uninitializing the libUARTE library.
 *
 * @param[in] p_libuarte     Pointer to libuarte instance.
 */
void nrf_libuarte_drv_uninit(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Function for sending data over UARTE using EasyDMA.
 *
 * @param[in] p_libuarte Pointer to libuarte instance.
 * @param[in] p_data     Pointer to data.
 * @param[in] len        Number of bytes to send.
 *
 * @retval NRF_ERROR_BUSY      Data is transferring.
 * @retval NRF_ERROR_INTERNAL  Error during PPI channel configuration.
 * @retval NRF_SUCCESS         Buffer set for sending.
 */
ret_code_t nrf_libuarte_drv_tx(const nrf_libuarte_drv_t * const p_libuarte,
                               uint8_t * p_data, size_t len);

/**
 * @brief Function for starting receiving data with additional configuration of external
 *        trigger to start receiving.
 *
 * @param p_libuarte      Pointer to libuarte instance.
 *
 * @retval NRF_ERROR_INTERNAL  Error during PPI channel configuration.
 * @retval NRF_SUCCESS         Buffer set for receiving.
 */
ret_code_t nrf_libuarte_drv_rx_start(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Function for stopping receiving data over UARTE.
 *
 * @param p_libuarte       Pointer to libuarte instance.
 */
void nrf_libuarte_drv_rx_stop(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Function for requesting the on/off state of the receiver
 *
 * @param  p_libuarte Pointer to libuarte instance.
 * @retval bool - Enabled state of the receiver
 */
bool nrf_libuarte_drv_rx_enabled(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Get the pointer to the libuarte packet status structure
 *  i.e. number of chars, number of packets, errors etc
 *
 * @param  p_libuarte Pointer to libuarte instance.
 * @retval df_packet_stats_t* - Pointer to the packet status structure
 */
df_packet_stats_t * get_packet_stats(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Clear the contents of the libuarte packet status structure
 *  i.e. number of chars, number of packets, errors etc
 *
 * @param  p_libuarte Pointer to libuarte instance.
 * @retval void
 */
void clear_packet_stats(const nrf_libuarte_drv_t * const p_libuarte);

/**
  * @brief Get the pointer to the libuarte packet control structure
  *  i.e. current packet number of chars, checksum veracity etc
  *
 * @param  p_libuarte Pointer to libuarte instance.
 * @retval df_packet_ctl_t* - Pointer to the packet control structure
 */
df_packet_ctl_t * get_packet_control(const nrf_libuarte_drv_t * const p_libuarte);

/**
 * @brief Clear the contents of the libuarte packet control structure (the part that describes the packet)
 *  i.e. size of packet, checksumOK, TxReflection
 *
 * @param  p_libuarte Pointer to libuarte instance.
 * @retval void
 */
void clr_pkt_ctl(const nrf_libuarte_drv_t * const p_libuarte);


/** @} */

#endif //DF_LIBUARTE_DRV_H
