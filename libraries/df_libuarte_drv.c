/**
* Datafuel clone of Nordic nrf_libuarte_drv
 *
 * functionality added to allow better control for the Datafuel DDPC application
 *
 *
 * Copyright (c) 2019 - 2020, Nordic Semiconductor ASA
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
#include "sdk_config.h"
//#include "nrfx_timer.h"
#include "nrf_gpio.h"
#include <nrfx_gpiote.h>
#include "df_libuarte_drv.h"

#define NRF_LOG_MODULE_NAME libUARTE
#if NRF_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       NRF_LIBUARTE_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  NRF_LIBUARTE_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR NRF_LIBUARTE_CONFIG_DEBUG_COLOR
#else // NRF_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // NRF_LIBUARTE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define INTERRUPTS_MASK  \
     NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_TXDRDY |  \
     NRF_UART_INT_MASK_ERROR | NRF_UART_INT_MASK_RXTO)
// Unused interrupts     NRF_UART_INT_MASK_CTS,  NRF_UART_INT_MASK_NCTS

#if defined(NRF_LIBUARTE_DRV_HWFC_ENABLED)
#define LIBUARTE_DRV_WITH_HWFC NRF_LIBUARTE_DRV_HWFC_ENABLED
#else
#define LIBUARTE_DRV_WITH_HWFC 1
#endif

#define RTS_PIN_DISABLED 0xff

#define IN_CHAR_LEN     1
static uint8_t in_char[IN_CHAR_LEN];

#define IN_CHAR_ALT_LEN     10
static uint8_t in_char_alt[IN_CHAR_ALT_LEN];


// local function protoypes
static void irq_handler(nrfx_uart_event_t const * p_event, void * p_context);
static void clr_buf_ctl(df_packet_ctl_t * pc);

/** @brief Macro executes given function on every allocated channel in the list between provided
 * indexes.
 */
#define PPI_CHANNEL_FOR_M_N(p_libuarte, m, n, func) \
        for (int i = m; i < n; i++) \
        { \
            if (p_libuarte->ctrl_blk->ppi_channels[i] < PPI_CH_NUM) \
            { func(&p_libuarte->ctrl_blk->ppi_channels[i]); } \
        }

/** @brief Macro executes provided function on every allocated PPI channel. */
#define PPI_CHANNEL_FOR_ALL(p_libuarte, func) \
        PPI_CHANNEL_FOR_M_N(p_libuarte, 0, NRF_LIBUARTE_DRV_PPI_CH_MAX, func)

/** @brief Disable and free PPI channel. */
static void ppi_ch_free(nrf_ppi_channel_t * p_ch)
{
    nrfx_err_t err;
    err = nrfx_ppi_channel_disable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
    err = nrfx_ppi_channel_free(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
    *p_ch = (nrf_ppi_channel_t)PPI_CH_NUM;
}

/** @brief Free all channels. */
static void ppi_free(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_ALL(p_libuarte, ppi_ch_free);     }

static ret_code_t ppi_channel_configure(nrf_ppi_channel_t * p_ch, uint32_t evt, uint32_t task, uint32_t fork)
{   nrfx_err_t err;

    err = nrfx_ppi_channel_alloc(p_ch);
    if (err != NRFX_SUCCESS)
    {   return NRF_ERROR_NO_MEM;      }

    err = nrfx_ppi_channel_assign(*p_ch, evt, task);
    if (err != NRFX_SUCCESS)
    {   return NRF_ERROR_INTERNAL;     }

    if (fork)
    {   err = nrfx_ppi_channel_fork_assign(*p_ch, fork);
        if (err != NRFX_SUCCESS)
        {   return NRF_ERROR_INTERNAL;     }
    }

    return NRF_SUCCESS;
}

static ret_code_t ppi_configure(const nrf_libuarte_drv_t * const p_libuarte, nrf_libuarte_drv_config_t * p_config)
{
    ret_code_t ret = NRFX_SUCCESS;

    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_ENDRX_STARTRX],
            nrf_uart_event_address_get(p_libuarte->uart.p_reg, NRF_UART_EVENT_RXDRDY),
            nrf_uart_task_address_get(p_libuarte->uart.p_reg, NRF_UART_TASK_STARTRX),
            NULL);
    if (ret != NRF_SUCCESS)
    {   goto complete_config;   }


    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_RXERR_STARTRX],
            nrf_uart_event_address_get(p_libuarte->uart.p_reg, NRF_UART_EVENT_ERROR),
            nrf_uart_task_address_get(p_libuarte->uart.p_reg, NRF_UART_TASK_STARTRX),
            NULL);
    if (ret != NRF_SUCCESS)
    {   goto complete_config;   }


     ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_RXTO_STARTRX],
            nrf_uart_event_address_get(p_libuarte->uart.p_reg, NRF_UART_EVENT_RXTO),
            nrf_uart_task_address_get(p_libuarte->uart.p_reg, NRF_UART_TASK_STARTRX),
            NULL);

complete_config:
    if (ret == NRF_SUCCESS)
    {   return ret;   }

    ppi_free(p_libuarte);

    return ret;
}

ret_code_t nrf_libuarte_drv_init(const nrf_libuarte_drv_t * const p_libuarte,
                             nrf_libuarte_drv_config_t * p_config,
                             nrf_libuarte_drv_evt_handler_t evt_handler,
                             void * p_context)
{
    ret_code_t ret;

    if (p_libuarte->ctrl_blk->enabled)
    {   return NRF_ERROR_INVALID_STATE;     }

    p_libuarte->ctrl_blk->evt_handler = evt_handler;
    memcpy(&p_libuarte->ctrl_blk->rx_in_buf, &p_config->rx_in_buf, sizeof(nrf_libuarte_drv_data_t));
    memcpy(&p_libuarte->ctrl_blk->packet_buf, &p_config->packet_buf, sizeof(nrf_libuarte_drv_data_t));
    p_libuarte->ctrl_blk->tx_out_buf.p_data = NULL;
    p_libuarte->ctrl_blk->p_context = p_libuarte->ctrl_blk;

    if ((p_config->txen_pin == 0) || (p_config->txen_pin == -1))
    {   p_config->txen_pin = NRF_UART_PSEL_DISCONNECTED; }

    p_libuarte->ctrl_blk->txen_pin = p_config->txen_pin;
    if(p_config->txen_pin != NRF_UART_PSEL_DISCONNECTED)
    {   nrf_gpio_cfg_output(p_libuarte->ctrl_blk->txen_pin);
        nrf_gpio_pin_clear(p_libuarte->ctrl_blk->txen_pin);    
    }

    //UART init
    nrfx_uart_config_t uc = NRFX_UART_DEFAULT_CONFIG;
    uc.pseltxd = p_config->tx_pin;
    uc.pselrxd = p_config->rx_pin;
    uc.p_context = (void *)p_libuarte;
    uc.baudrate = p_config->baudrate;
    
    ret = nrfx_uart_init(&p_libuarte->uart, &uc, irq_handler);
    nrf_libuarte_drv_rx_start(p_libuarte, true);

    ret = ppi_configure(p_libuarte, p_config);
    if (ret != NRF_SUCCESS)
    {   return NRF_ERROR_INTERNAL;  }

    p_libuarte->ctrl_blk->enabled = true;

    return NRF_SUCCESS;
}

void nrf_libuarte_drv_uninit(const nrf_libuarte_drv_t * const p_libuarte)
{
    if (p_libuarte->ctrl_blk->enabled == false)
    {   return;     }

    p_libuarte->ctrl_blk->enabled = false;

    nrfx_uart_uninit(&p_libuarte->uart);
 
    ppi_free(p_libuarte);

    p_libuarte->ctrl_blk->tx_out_buf.p_data = NULL;

}

ret_code_t nrf_libuarte_drv_tx(const nrf_libuarte_drv_t * const p_libuarte,
                               uint8_t * p_data, size_t len)
{   ret_code_t ret = NRFX_SUCCESS;
    
    if (p_libuarte->ctrl_blk->tx_out_buf.p_data)
    {   return NRF_ERROR_BUSY;      }

    p_libuarte->ctrl_blk->tx_out_buf.p_data = p_data;
    p_libuarte->ctrl_blk->tx_out_buf.size = len;
    p_libuarte->ctrl_blk->tx_cur_idx = 0;

    p_libuarte->ctrl_blk->packet_stats.tx_packets += 1;

    if(p_libuarte->ctrl_blk->txen_pin != NRF_UART_PSEL_DISCONNECTED)
    {   nrf_gpio_pin_set(p_libuarte->ctrl_blk->txen_pin);    }          // turn on the TxEnable pin

    NRF_LOG_DEBUG("Started TX total length:%d", len);
    ret = nrfx_uart_tx(&p_libuarte->uart, p_data, len);

    return(ret);
}

ret_code_t nrf_libuarte_drv_rx_start(const nrf_libuarte_drv_t * const p_libuarte, bool clearFlags)
{
    nrfx_uart_rx_enable(&p_libuarte->uart);
    nrfx_uart_rx(&p_libuarte->uart, in_char, IN_CHAR_LEN);
    nrfx_uart_rx(&p_libuarte->uart, in_char_alt, IN_CHAR_ALT_LEN);
    
    if (clearFlags)
    {   clr_buf_ctl(&p_libuarte->ctrl_blk->packet_ctl);  }

    return NRF_SUCCESS;
}

void nrf_libuarte_drv_rx_stop(const nrf_libuarte_drv_t * const p_libuarte)
{
    NRF_LOG_DEBUG("Stopping Rx.");
    nrfx_uart_rx_disable(&p_libuarte->uart);
}

df_packet_stats_t * get_packet_stats(const nrf_libuarte_drv_t * const p_libuarte)
{   return(&p_libuarte->ctrl_blk->packet_stats);    }

void clear_packet_stats(const nrf_libuarte_drv_t * const p_libuarte)
{   memset(&p_libuarte->ctrl_blk->packet_stats, 0x00, sizeof(df_packet_stats_t));   }

df_packet_ctl_t * get_packet_control(const nrf_libuarte_drv_t * const p_libuarte)
{   return(&p_libuarte->ctrl_blk->packet_ctl);      }


static uint8_t make_lrc(char *buf, uint8_t cnt)
{   uint8_t i;
    uint8_t history = 0;

    for (i=0;i<cnt;i++)
    {   history = history ^ buf[i];     // XOR new byte into history
        history = history ^ 0xFF;   	// Complement the result
        if (history >= 0x0080)          // high bit set
        {   history <<= 1;    		// Shift Byte left
            history |= 0x01;         	// High bit was set.. move high bit to low bit
        }
        else
        {   history <<= 1;  }           // Shift Byte left
        history = history + buf[i];
    }

    return(history);
}

static bool is_checksum_ok(char * buf, uint8_t len)
{   uint8_t lrc;
    bool retval = false;                                // default result to "bad"
    
    if (len >= MSG_MIN_SIZE)                            // buf will hold rx_char_count chars .. make sure there is enough for calculation
    {   lrc = make_lrc(&buf[START_LRC_CALC], (len - 3));    // Calc the LRC
        if (lrc == buf[(len - 2)])                      // check the lrc against the message
        {   retval = true;  }                           // .. good? say so
    }
    return(retval);
}

void clr_pkt_ctl(const nrf_libuarte_drv_t * const p_libuarte)
{   p_libuarte->ctrl_blk->packet_ctl.packet_length = 0;
    p_libuarte->ctrl_blk->packet_ctl.checksum_ok = false;
    p_libuarte->ctrl_blk->packet_ctl.tx_reflection = false;
}

static void clr_buf_ctl(df_packet_ctl_t * pc)
{   
    pc->got_SOH = false;
    pc->got_stuff = false;
    pc->in_buf_ptr = 0;
}            


static void irq_handler(nrfx_uart_event_t const * p_event, void * p_context)
{
    nrf_libuarte_drv_t * ctxt = p_context;
    nrf_libuarte_drv_ctrl_blk_t * cb = ctxt->ctrl_blk;

    switch (p_event->type)
    {
        case NRFX_UART_EVT_TX_DONE:         // Tx finished 
               
               cb->tx_out_buf.p_data = NULL;
               cb->tx_out_buf.size = 0;

               nrf_libuarte_drv_evt_t evt = {
                    .type = NRF_LIBUARTE_DRV_EVT_TX_DONE, 
                    .data = {
                        .rxtx = {.p_data = cb->tx_out_buf.p_data, .size = cb->tx_out_buf.size } 
                    } };
               cb->evt_handler(cb, &evt);

               if(cb->txen_pin != NRF_UART_PSEL_DISCONNECTED)
               {   nrf_gpio_pin_clear(cb->txen_pin);    }          // turn off the TxEnable pin
               
               nrf_libuarte_drv_rx_start(ctxt, true);
            break;
        case NRFX_UART_EVT_RX_DONE:
            cb->packet_stats.total_chars += 1;
            
            /* if (p_event->data.rxtx.p_data == in_char_alt)       // if chars have been diverted to the alt-buffer
            {   clr_buf_ctl(&cb->packet_ctl);                   // just dump the packet & wait for another SOH
                break;
            } */  
            
            uint8_t char_in = *p_event->data.rxtx.p_data;

            if (cb->packet_ctl.got_stuff == true)       // Modify the next char if the last was a STUFF_CHAR
            {   char_in = char_in | 0x80;    
                cb->packet_ctl.got_stuff = false;       // .. and de-flag
            }

            if (char_in == STUFF_CHAR)                  // Stuff char received ..
            {   cb->packet_ctl.got_stuff = true;        // .. just mark it
                break;                                  // and abort
            }

            cb->rx_in_buf.p_data[cb->packet_ctl.in_buf_ptr] = char_in;  // Save the char in the in_buf
            cb->packet_ctl.in_buf_ptr += 1;                             // and increment the in_buf pointer

            nrf_libuarte_drv_rx_start(ctxt, false);     // restart comms (but don't clear packet-assembly vars)

            if (char_in == SOH_CHAR)  
            {   cb->packet_stats.SOH_cnt += 1;
                clr_buf_ctl(&cb->packet_ctl);
                cb->packet_ctl.got_SOH = true;

                cb->rx_in_buf.p_data[cb->packet_ctl.in_buf_ptr] = char_in;  // Save the char in the in_buf
                cb->packet_ctl.in_buf_ptr += 1;                             // and increment the in_buf pointer
            }
            else
            {   if (cb->packet_ctl.got_SOH)
                {   if (char_in == EOF_CHAR)                // packet finished .. bundle packet up to application layer
                    {   cb->packet_stats.EOF_cnt += 1;
                        cb->packet_stats.rx_packets += 1;

                        cb->packet_ctl.packet_length = cb->packet_ctl.in_buf_ptr;       // in_buf_ptr is already increment to an effective count
                        memcpy(cb->packet_buf.p_data, cb->rx_in_buf.p_data, cb->packet_ctl.packet_length);   // Move the packet from in_buf to packet_buf

                        if (is_checksum_ok(cb->packet_buf.p_data, cb->packet_ctl.packet_length))
                        {   cb->packet_stats.good_rx_packets += 1;
                            cb->packet_ctl.checksum_ok = true;
                        }

                        nrf_libuarte_drv_evt_t evt = {
                            .type = NRF_LIBUARTE_DRV_EVT_GOT_PACKET, 
                            .data = {
                                .rxtx = {.p_data = cb->packet_buf.p_data, .size = cb->packet_ctl.packet_length } 
                            } };
                        cb->evt_handler(cb, &evt);

                        clr_buf_ctl(&cb->packet_ctl);       // quick reset to state just by saying there is no SOH .. routine will start again
                        break;
                    }
                }
                else    // chars received when there is no SOH .. just drop them
                {   
                    break;       
                }
            }

            if (cb->tx_out_buf.size > 0)                                // Check if there are ongoing Tx comms
            {   cb->packet_ctl.tx_reflection = true;    }               // set the tx_reflection flag

            if (cb->packet_ctl.in_buf_ptr > (cb->rx_in_buf.size - 20))       // Drop the buffer if there are size-20 bytes received without packet delimiters
            {   clr_buf_ctl(&cb->packet_ctl);    
                cb->packet_stats.err_cnt += 1;                               // Log a packet error
            } 

            break;
        case NRFX_UART_EVT_ERROR:
            cb->packet_stats.err_cnt += 1;

            nrf_libuarte_drv_evt_t err_evt = 
                    {   .type = NRF_LIBUARTE_DRV_EVT_ERROR, 
                        .data = { .errorsrc = p_event->data.error.error_mask}    
                    };
            cb->evt_handler(&cb, &err_evt);
            
            nrf_libuarte_drv_rx_start(ctxt, true);

            break;
        default:
            clr_buf_ctl(&cb->packet_ctl);
            nrf_libuarte_drv_rx_start(ctxt, true);

            break;
    }



}

