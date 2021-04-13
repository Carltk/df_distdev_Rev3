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
#include "df_libuarte_drv.h"
#include "nrfx_timer.h"
// #include "nrfx_uart.h"
#include "nrf_gpio.h"
#include <nrfx_gpiote.h>

#include "../../df_distdev_DDPCP_Rev3/hardware_init.h"

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

static void clear_packet_control(const nrf_libuarte_drv_t * const p_libuarte);
static void irq_handler(nrfx_uart_event_t const * p_event, void * p_context);

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

/** @brief Macro executes provided function on every allocated group in the list. */
#define PPI_GROUP_FOR_ALL(p_libuarte, func) \
        for (int i = 0; i < NRF_LIBUARTE_DRV_PPI_GROUP_MAX; i++) \
        { \
            if (p_libuarte->ctrl_blk->ppi_groups[i] < PPI_GROUP_NUM) \
                { func(&p_libuarte->ctrl_blk->ppi_groups[i]); } \
        }

/** @brief Allocate and configure PPI channel. Fork is optional and it's not set if NULL.
 *         Channel parameter is field by the function.
 */
static ret_code_t ppi_channel_configure(nrf_ppi_channel_t * p_ch, uint32_t evt,
                                        uint32_t task, uint32_t fork)
{
    nrfx_err_t err;

    err = nrfx_ppi_channel_alloc(p_ch);
    if (err != NRFX_SUCCESS)
    {
        return NRF_ERROR_NO_MEM;
    }

    err = nrfx_ppi_channel_assign(*p_ch, evt, task);
    if (err != NRFX_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    if (fork)
    {
        err = nrfx_ppi_channel_fork_assign(*p_ch, fork);
        if (err != NRFX_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}

/** @brief Allocate and configure group with one channel. Fetch addresses of enable/disable tasks.*/
static ret_code_t ppi_group_configure(nrf_ppi_channel_group_t * p_ppi_group, nrf_ppi_channel_t ch,
                                      uint32_t * p_en_task, uint32_t * p_dis_task, bool en)
{
    nrfx_err_t err;

    err = nrfx_ppi_group_alloc(p_ppi_group);
    if (err != NRFX_SUCCESS)
    {   return NRF_ERROR_NO_MEM;    }

    err = nrfx_ppi_channel_include_in_group(ch, *p_ppi_group);
    if (err != NRFX_SUCCESS)
    {   return NRF_ERROR_INTERNAL;      }

    if (en)
    {   err = nrfx_ppi_group_enable(*p_ppi_group);
        if (err != NRFX_SUCCESS)
        {   return NRF_ERROR_INTERNAL;          }
    }

    *p_en_task = nrfx_ppi_task_addr_group_enable_get(*p_ppi_group);
    *p_dis_task = nrfx_ppi_task_addr_group_disable_get(*p_ppi_group);

    return NRF_SUCCESS;
}

/** @brief Disable and free PPI channel. */
static void ppi_ch_free(nrf_ppi_channel_t * p_ch)
{   nrfx_err_t err = nrfx_ppi_channel_disable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
    err = nrfx_ppi_channel_free(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
    *p_ch = (nrf_ppi_channel_t)PPI_CH_NUM;
}

/** @brief Disable and free PPI group. */
static void ppi_group_free(nrf_ppi_channel_group_t * p_group)
{   nrfx_err_t err = nrfx_ppi_group_free(*p_group);
    ASSERT(err == NRFX_SUCCESS);
    *p_group = (nrf_ppi_channel_group_t)PPI_GROUP_NUM;
}

/** @brief Free all channels. */
static void ppi_free(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_ALL(p_libuarte, ppi_ch_free);
    PPI_GROUP_FOR_ALL(p_libuarte, ppi_group_free);
}

/** @brief Enable PPI channel. */
static void ppi_ch_enable(nrf_ppi_channel_t * p_ch)
{   nrfx_err_t err = nrfx_ppi_channel_enable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
}

/** @brief Disable PPI channel. */
static void ppi_ch_disable(nrf_ppi_channel_t * p_ch)
{   nrfx_err_t err = nrfx_ppi_channel_disable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
}

/** @brief Enable PPI channels for RX. */
static void rx_ppi_enable(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_M_N(p_libuarte, 0, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, ppi_ch_enable);    }

/** @brief Disable PPI channels for RX. */
static void rx_ppi_disable(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_M_N(p_libuarte, 0, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, ppi_ch_disable);   }

/** @brief Enable PPI channels for TX. */
static void tx_ppi_enable(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_M_N(p_libuarte, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, NRF_LIBUARTE_DRV_PPI_CH_MAX, ppi_ch_enable);  }

/** @brief Disable PPI channels for TX. */
static void tx_ppi_disable(const nrf_libuarte_drv_t * const p_libuarte)
{   PPI_CHANNEL_FOR_M_N(p_libuarte, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, NRF_LIBUARTE_DRV_PPI_CH_MAX, ppi_ch_disable); }

static ret_code_t ppi_configure(const nrf_libuarte_drv_t * const p_libuarte,
                                nrf_libuarte_drv_config_t * p_config)
{   ret_code_t ret;

    for (int i = 0; i < NRF_LIBUARTE_DRV_PPI_CH_MAX; i++)
    {   p_libuarte->ctrl_blk->ppi_channels[i] = (nrf_ppi_channel_t)PPI_CH_NUM;  }    // set to invalid value */

    for (int i = 0; i < NRF_LIBUARTE_DRV_PPI_GROUP_MAX; i++)
    {   p_libuarte->ctrl_blk->ppi_groups[i] = (nrf_ppi_channel_group_t)PPI_GROUP_NUM;  }       /* set to invalid value */

    // PPI -> Link Uarte RxdRdy (char Rxd) Event with Timer Count Task and move total to DF_COUNT_CHAN_RXD_CHARS capture channel

/*
    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_RXRDY_TIMER_COUNT],
            nrf_uart_event_address_get(p_libuarte->uart, NRF_UART_EVENT_RXDRDY),
            nrfx_timer_task_address_get(p_libuarte->timer, NRF_TIMER_TASK_COUNT),
            nrfx_timer_capture_task_address_get(p_libuarte->timer, DF_COUNT_CHAN_RXD_CHARS));
    if (ret != NRF_SUCCESS)
    {   goto complete_config;    }

*/
/*    // PPI -> Link Uarte EndRx (buffer full) Event with Uarte StartRx Task and Fork to move timer Count into Capture/Compare channel 0
    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_ENDRX_STARTRX],
            nrf_uarte_event_address_get(p_libuarte->uart, NRF_UARTE_EVENT_ENDRX),
            nrf_uarte_task_address_get(p_libuarte->uart, NRF_UARTE_TASK_STARTRX),
            nrfx_timer_capture_task_address_get(&p_libuarte->timer, DF_COUNT_CHAN_RXD_TOTAL));
    if (ret != NRF_SUCCESS)
    {   goto complete_config;   }
*/

complete_config:
    if (ret == NRF_SUCCESS)
    {   return ret;    }

    ppi_free(p_libuarte);

    return ret;
}

// Tasks - NRF_UART_TASK_STARTRX, NRF_UART_TASK_STOPRX, NRF_UART_TASK_STARTTX, NRF_UART_TASK_STOPTX, NRF_UART_TASK_SUSPEND
// Events - NRF_UART_EVENT_CTS, NRF_UART_EVENT_NCTS, NRF_UART_EVENT_RXDRDY, NRF_UART_EVENT_TXDRDY,NRF_UART_EVENT_ERROR, NRF_UART_EVENT_RXTO  

void tmr_evt_handler(nrf_timer_event_t event_type, void * p_context)
{   UNUSED_PARAMETER(event_type);
    UNUSED_PARAMETER(p_context);
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

    //UART init
    nrfx_uart_config_t uc = NRFX_UART_DEFAULT_CONFIG;
    uc.pseltxd = p_config->tx_pin;
    uc.pselrxd = p_config->rx_pin;
    uc.p_context = p_libuarte;
    uc.baudrate = p_config->baudrate;
    
    ret = nrfx_uart_init(p_libuarte->uart, &uc, irq_handler);

//    NVIC_SetPriority(irqn, p_config->irq_priority);
//    NVIC_ClearPendingIRQ(irqn);
//    NVIC_EnableIRQ(irqn);

    nrfx_uart_rx_enable(p_libuarte->uart);

    nrfx_timer_config_t tmr_config = NRFX_TIMER_DEFAULT_CONFIG;
    tmr_config.mode = NRF_TIMER_MODE_COUNTER;
    tmr_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret = nrfx_timer_init(p_libuarte->timer, &tmr_config, tmr_evt_handler);
    if (ret != NRFX_SUCCESS)
    {   return NRF_ERROR_INTERNAL;     }

    ret = ppi_configure(p_libuarte, p_config);
    if (ret != NRF_SUCCESS)
    {   return NRF_ERROR_INTERNAL;    }

    p_libuarte->ctrl_blk->enabled = true;
    return NRF_SUCCESS;
}

void nrf_libuarte_drv_uninit(const nrf_libuarte_drv_t * const p_libuarte)
{
    IRQn_Type irqn = nrfx_get_irq_number(p_libuarte->uart);

    if (p_libuarte->ctrl_blk->enabled == false)
    {   return;     }

    p_libuarte->ctrl_blk->enabled = false;

    //NVIC_DisableIRQ(irqn);

    rx_ppi_disable(p_libuarte);
    tx_ppi_disable(p_libuarte);

    nrfx_uart_uninit(p_libuarte->uart);
 
    p_libuarte->ctrl_blk->tx_out_buf.p_data = NULL;

    nrfx_timer_disable(p_libuarte->timer);
    nrfx_timer_uninit(p_libuarte->timer);

    ppi_free(p_libuarte);
}

ret_code_t nrf_libuarte_drv_tx(const nrf_libuarte_drv_t * const p_libuarte,
                               uint8_t * p_data, size_t len)
{   ret_code_t ret = NRFX_SUCCESS;
    
    if (p_libuarte->ctrl_blk->tx_out_buf.p_data)
    {   return NRF_ERROR_BUSY;      }

    p_libuarte->ctrl_blk->tx_out_buf.p_data = p_data;
    p_libuarte->ctrl_blk->tx_out_buf.size = len;
    p_libuarte->ctrl_blk->tx_cur_idx = 0;

    NRF_LOG_WARNING("Started TX total length:%d", len);
    ret = nrfx_uart_tx(p_libuarte->uart, p_data, len);

    return(ret);
}

ret_code_t nrf_libuarte_drv_rx_start(const nrf_libuarte_drv_t * const p_libuarte)
{
    /* Reset byte counting */
    clear_packet_control(p_libuarte);

    if (!nrfx_timer_is_enabled(p_libuarte->timer))
    {   nrfx_timer_enable(p_libuarte->timer);  }
    
    nrfx_timer_clear(p_libuarte->timer);
    rx_ppi_enable(p_libuarte);

    nrfx_uart_rx_enable(p_libuarte->uart);

    return NRF_SUCCESS;
}

void nrf_libuarte_drv_rx_stop(const nrf_libuarte_drv_t * const p_libuarte)
{   rx_ppi_disable(p_libuarte);
    NRF_LOG_DEBUG("Stopping Rx.");
    nrfx_uart_rx_disable(p_libuarte->uart);
    nrfx_timer_disable(p_libuarte->timer);
}

df_packet_stats_t * get_packet_stats(const nrf_libuarte_drv_t * const p_libuarte)
{   return(&p_libuarte->ctrl_blk->packet_stats);    }

void clear_packet_stats(const nrf_libuarte_drv_t * const p_libuarte)
{   memset(&p_libuarte->ctrl_blk->packet_stats, 0x00, sizeof(df_packet_stats_t));   }

df_packet_ctl_t * get_packet_control(const nrf_libuarte_drv_t * const p_libuarte)
{   return(&p_libuarte->ctrl_blk->packet_ctl);      }

static void clear_packet_control(const nrf_libuarte_drv_t * const p_libuarte)
{   memset(&p_libuarte->ctrl_blk->packet_ctl, 0x00, sizeof(df_packet_ctl_t));   }


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


static void irq_handler(nrfx_uart_event_t const * p_event, void * p_context)
{
    nrf_libuarte_drv_t * ctxt = p_context;
    nrf_libuarte_drv_ctrl_blk_t * cb = ctxt->ctrl_blk;

    switch (p_event->type)
    {
        case NRFX_UART_EVT_TX_DONE:         // Tx finished 
                // Do I need to move char by char into the TxBuf or is that handled already?
               cb->tx_out_buf.p_data = NULL;
        
               nrf_libuarte_drv_evt_t evt = {.type = NRF_LIBUARTE_DRV_EVT_TX_DONE, .data = {.rxtx = {.p_data = cb->tx_out_buf.p_data, .size = cb->tx_out_buf.size } } };
               cb->evt_handler(cb, &evt);

/*
                //size_t amount = nrf_uart_tx_amount_get(ctxt->uart);
                NRF_LOG_DEBUG("(evt) TX completed (%d)", amount);
                cb->tx_cur_idx += amount;
                if (cb->tx_cur_idx >= cb->tx_out_buf.size)
                {   
                    nrf_uarte_event_clear(ctxt->uart, NRF_UARTE_EVENT_TXSTOPPED);
                    nrf_uarte_task_trigger(ctxt->uart, NRF_UARTE_TASK_STOPTX);
                }
                else
                {   // size_t rem_len = (cb->tx_out_buf.size - cb->tx_cur_idx);
                    tx_ppi_disable(ctxt);     
                }
*/
            
            break;
        case NRFX_UART_EVT_RX_DONE:
            cb->packet_stats.total_chars += 1;

            //cb->packet_ctl.curr_locn = nrfx_timer_capture_get(&ctxt->timer, DF_COUNT_CHAN_RXD_CHARS);           // get the count from the timer
            uint8_t char_in = *(char *)(cb->rx_in_buf.p_data + cb->packet_ctl.curr_locn - 1);       // get this byte from the rx buffer

            if (cb->packet_ctl.got_SOH)
            {   
                nrf_libuarte_drv_evt_t evt;
            
                if (char_in == EOF_CHAR)   // packet finished .. bundle packet up to application layer
                {   cb->packet_stats.EOF_cnt += 1;
                    cb->packet_stats.total_packets += 1;
                
                    cb->packet_ctl.packet_length = (cb->packet_ctl.curr_locn - cb->packet_ctl.SOH_locn + 1);

                    NRF_LOG_INFO("%ld - libuarte: got EOF", get_sys_ms());
    
                    if (cb->packet_ctl.got_stuff)                         // if there is STUFF_CHAR(s) then we need to cobert the buffer on the way across
                    {   uint8_t pktidx = 0;                                                 // a write-index to the packet buffer
                        uint8_t inofst = (cb->packet_ctl.SOH_locn - 1);   // offset to the start of incomming data

                        for (uint8_t i=1;i<=cb->packet_ctl.packet_length;i++)     // loop around the received buffer
                        {   
                            if (cb->rx_in_buf.p_data[inofst+i] == STUFF_CHAR)     // got a stuff char
                            {   i += 1;     // go to the next char (i.e. skip the STUFF_CHAR)
                                cb->packet_buf.p_data[pktidx] = (cb->rx_in_buf.p_data[inofst+i] | 0x80);
                            }
                            else
                            {   cb->packet_buf.p_data[pktidx] = cb->rx_in_buf.p_data[inofst+i]; }
                        
                            pktidx += 1;            // inc the packetbuf index
                        }
                        cb->packet_ctl.packet_length = pktidx;        // reset the packet length to the new size (without STUFF_CHARs)
                    }
                    else        // no stuff chars .. just use a memcpy to move the packet into the packet buffer
                    {   memcpy(cb->packet_buf.p_data, (cb->rx_in_buf.p_data + cb->packet_ctl.SOH_locn - 1), cb->packet_ctl.packet_length);   }    

                    if (is_checksum_ok(cb->packet_buf.p_data, cb->packet_ctl.packet_length))
                    {   cb->packet_stats.good_packets += 1;
                        cb->packet_ctl.checksum_ok = true;
                    }

                    // Check if the inbuf is greater than 50% full
                    //if (cb->packet_ctl.curr_locn > (cb->rx_in_buf.size >> 1))       
                    //{   nrf_libuarte_drv_rx_stop(ctxt);    }          // stop/restart Rx to reload the buffer
            
                   // Send the GOT_PACKET event up to the application
                    evt.type = NRF_LIBUARTE_DRV_EVT_GOT_PACKET;     // set the event type (i.e. Got a packet)
                    cb->evt_handler(cb, &evt);     // pass interrupt up to application layer

                    cb->packet_ctl.got_SOH = false;       // quick reset to state just by saying there is no SOH .. routine will start again
                }
                else if (char_in == STUFF_CHAR)
                {   cb->packet_ctl.got_stuff = true;    }
                else if (char_in == SOH_CHAR)        // another SOH .. reset and start again
                {   cb->packet_ctl.got_SOH = true;
                    cb->packet_ctl.got_stuff = false;
                    cb->packet_ctl.checksum_ok = false;
                    cb->packet_ctl.SOH_locn = cb->packet_ctl.curr_locn;
                    cb->packet_stats.SOH_cnt += 1;
                }
                // else - nothing to do .. uarte automatically saves characters
            }
            else
            {   if (char_in == SOH_CHAR)  
                {   cb->packet_ctl.got_SOH = true;
                    cb->packet_ctl.SOH_locn = cb->packet_ctl.curr_locn;
                    cb->packet_stats.SOH_cnt += 1;
                    NRF_LOG_INFO("%ld - libuarte: got SOH", get_sys_ms());
                }
            }

            if (cb->packet_ctl.curr_locn > (cb->rx_in_buf.size - 20))       // Check if the inbuf is greater than 20 short of the size
            {   // nrf_libuarte_drv_rx_stop(ctxt);    
                NRF_LOG_INFO("Libuarte: Stop Rx (> size-20)");                        
            } 
            break;
        case NRFX_UART_EVT_ERROR:
            cb->packet_stats.err_cnt += 1;

            nrf_libuarte_drv_evt_t err_evt = 
                    {.type = NRF_LIBUARTE_DRV_EVT_ERROR, 
                     .data = { .errorsrc = nrf_uart_errorsrc_get_and_clear(ctxt->uart) }};
            cb->evt_handler(&cb, &err_evt);

            break;
        default:
            break;
    }



}

