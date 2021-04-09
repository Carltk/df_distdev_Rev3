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
#include "nrf_uarte.h"
#include "nrf_gpio.h"
#include <nrfx_gpiote.h>
#include <../src/prs/nrfx_prs.h>

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

#define MAX_DMA_XFER_LEN    ((1UL << UARTE0_EASYDMA_MAXCNT_SIZE) - 1)


#define INTERRUPTS_MASK  \
    (NRF_UARTE_INT_ENDRX_MASK | NRF_UARTE_INT_RXSTARTED_MASK | NRF_UARTE_INT_ERROR_MASK | \
     NRF_UARTE_INT_ENDTX_MASK | NRF_UARTE_INT_TXSTOPPED_MASK | NRF_UARTE_INT_RXDRDY_MASK)

// ENDRX -> EasyDMA has finished accessing the Rx Buffer in RAM (Receive buffer is filled up)
// RXDRDY -> Character received (but may not yet be in Data RAM) 

static const nrf_libuarte_drv_t * m_libuarte_instance[2];

/* if it is defined it means that PRS for uart is not used. */
#ifdef nrfx_uarte_0_irq_handler
#define libuarte_0_irq_handler UARTE0_UART0_IRQHandler
#endif

#if NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE0)
void libuarte_0_irq_handler(void);
#endif

#if NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE1)
void libuarte_1_irq_handler(void);
#endif

#if defined(NRF_LIBUARTE_DRV_HWFC_ENABLED)
#define LIBUARTE_DRV_WITH_HWFC NRF_LIBUARTE_DRV_HWFC_ENABLED
#else
#define LIBUARTE_DRV_WITH_HWFC 1
#endif

#define RTS_PIN_DISABLED 0xff

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
    {
        return NRF_ERROR_NO_MEM;
    }

    err = nrfx_ppi_channel_include_in_group(ch, *p_ppi_group);
    if (err != NRFX_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    if (en)
    {
        err = nrfx_ppi_group_enable(*p_ppi_group);
        if (err != NRFX_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    *p_en_task = nrfx_ppi_task_addr_group_enable_get(*p_ppi_group);
    *p_dis_task = nrfx_ppi_task_addr_group_disable_get(*p_ppi_group);

    return NRF_SUCCESS;
}

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

/** @brief Disable and free PPI group. */
static void ppi_group_free(nrf_ppi_channel_group_t * p_group)
{
    nrfx_err_t err;
    err = nrfx_ppi_group_free(*p_group);
    ASSERT(err == NRFX_SUCCESS);
    *p_group = (nrf_ppi_channel_group_t)PPI_GROUP_NUM;

}

/** @brief Free all channels. */
static void ppi_free(const nrf_libuarte_drv_t * const p_libuarte)
{
    PPI_CHANNEL_FOR_ALL(p_libuarte, ppi_ch_free);
    PPI_GROUP_FOR_ALL(p_libuarte, ppi_group_free);
}

/** @brief Enable PPI channel. */
static void ppi_ch_enable(nrf_ppi_channel_t * p_ch)
{
    nrfx_err_t err;
    err = nrfx_ppi_channel_enable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
}

/** @brief Disable PPI channel. */
static void ppi_ch_disable(nrf_ppi_channel_t * p_ch)
{
    nrfx_err_t err;
    err = nrfx_ppi_channel_disable(*p_ch);
    ASSERT(err == NRFX_SUCCESS);
}

/** @brief Enable PPI channels for RX. */
static void rx_ppi_enable(const nrf_libuarte_drv_t * const p_libuarte)
{
    PPI_CHANNEL_FOR_M_N(p_libuarte, 0, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, ppi_ch_enable);
}

/** @brief Disable PPI channels for RX. */
static void rx_ppi_disable(const nrf_libuarte_drv_t * const p_libuarte)
{
    PPI_CHANNEL_FOR_M_N(p_libuarte, 0, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX, ppi_ch_disable);
}

/** @brief Enable PPI channels for TX. */
static void tx_ppi_enable(const nrf_libuarte_drv_t * const p_libuarte)
{
    PPI_CHANNEL_FOR_M_N(p_libuarte, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX,
                        NRF_LIBUARTE_DRV_PPI_CH_MAX, ppi_ch_enable);
}

/** @brief Disable PPI channels for TX. */
static void tx_ppi_disable(const nrf_libuarte_drv_t * const p_libuarte)
{
    PPI_CHANNEL_FOR_M_N(p_libuarte, NRF_LIBUARTE_DRV_PPI_CH_RX_GROUP_MAX,
                        NRF_LIBUARTE_DRV_PPI_CH_MAX, ppi_ch_disable);
}

static ret_code_t ppi_configure(const nrf_libuarte_drv_t * const p_libuarte,
                                nrf_libuarte_drv_config_t * p_config)
{
    ret_code_t ret;
    uint32_t gr0_en_task = 0;
    uint32_t gr0_dis_task = 0;
    uint32_t gr1_en_task = 0;
    uint32_t gr1_dis_task = 0;

    for (int i = 0; i < NRF_LIBUARTE_DRV_PPI_CH_MAX; i++)
    {
        /* set to invalid value */
        p_libuarte->ctrl_blk->ppi_channels[i] = (nrf_ppi_channel_t)PPI_CH_NUM;
    }

    for (int i = 0; i < NRF_LIBUARTE_DRV_PPI_GROUP_MAX; i++)
    {
        /* set to invalid value */
        p_libuarte->ctrl_blk->ppi_groups[i] = (nrf_ppi_channel_group_t)PPI_GROUP_NUM;
    }

    if (MAX_DMA_XFER_LEN < UINT16_MAX)
    {
        // PPI -> Link Uarte EndTX Event with Uarte StartTx Task 
        ret = ppi_channel_configure(
                &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_ENDTX_STARTTX],
                nrf_uarte_event_address_get(p_libuarte->uarte, NRF_UARTE_EVENT_ENDTX),
                nrf_uarte_task_address_get(p_libuarte->uarte, NRF_UARTE_TASK_STARTTX),
                0);
        if (ret != NRF_SUCCESS)
        {
            goto complete_config;
        }
    }

    // PPI -> Link Uarte RxdRdy (char Rxd) Event with Timer Count Task and move total to DF_COUNT_CHAN_RXD_CHARS capture channel
    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_RXRDY_TIMER_COUNT],
            nrf_uarte_event_address_get(p_libuarte->uarte, NRF_UARTE_EVENT_RXDRDY),
            nrfx_timer_task_address_get(&p_libuarte->timer, NRF_TIMER_TASK_COUNT),
            nrfx_timer_capture_task_address_get(&p_libuarte->timer, DF_COUNT_CHAN_RXD_CHARS));
    if (ret != NRF_SUCCESS)
    {
        goto complete_config;
    }

    // PPI -> Link Uarte EndRx (buffer full) Event with Uarte StartRx Task and Fork to move timer Count into Capture/Compare channel 0
    ret = ppi_channel_configure(
            &p_libuarte->ctrl_blk->ppi_channels[NRF_LIBUARTE_DRV_PPI_CH_ENDRX_STARTRX],
            nrf_uarte_event_address_get(p_libuarte->uarte, NRF_UARTE_EVENT_ENDRX),
            nrf_uarte_task_address_get(p_libuarte->uarte, NRF_UARTE_TASK_STARTRX),
            nrfx_timer_capture_task_address_get(&p_libuarte->timer, DF_COUNT_CHAN_RXD_TOTAL));
    if (ret != NRF_SUCCESS)
    {
        goto complete_config;
    }

complete_config:
    if (ret == NRF_SUCCESS)
    {
        return ret;
    }

    ppi_free(p_libuarte);

    return ret;
}

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
    IRQn_Type irqn = nrfx_get_irq_number(p_libuarte->uarte);

    if (p_libuarte->ctrl_blk->enabled)
    {   return NRF_ERROR_INVALID_STATE;     }

    p_libuarte->ctrl_blk->evt_handler = evt_handler;
    p_libuarte->ctrl_blk->p_cur_rx = NULL;
    p_libuarte->ctrl_blk->p_next_rx = NULL;
    p_libuarte->ctrl_blk->p_next_next_rx = NULL;
    p_libuarte->ctrl_blk->p_tx = NULL;
    p_libuarte->ctrl_blk->p_context = p_context;
    p_libuarte->ctrl_blk->rts_pin = RTS_PIN_DISABLED;

    m_libuarte_instance[p_libuarte->uarte == NRF_UARTE0 ? 0 : 1] = p_libuarte;

    //UART init
    nrf_gpio_pin_set(p_config->tx_pin);
    nrf_gpio_cfg_output(p_config->tx_pin);
    nrf_gpio_cfg_input(p_config->rx_pin, p_config->pullup_rx ?
                        NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_NOPULL);
    nrf_uarte_baudrate_set(p_libuarte->uarte, p_config->baudrate);
    nrf_uarte_configure(p_libuarte->uarte, p_config->parity, NRF_UARTE_HWFC_DISABLED);
    nrf_uarte_txrx_pins_set(p_libuarte->uarte, p_config->tx_pin, p_config->rx_pin);

#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE0)
    if (irqn == UARTE0_UART0_IRQn)
    {
        if (nrfx_prs_acquire(p_libuarte->uarte, libuarte_0_irq_handler) != NRFX_SUCCESS)
        {   return NRF_ERROR_BUSY;        }
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE0)

    nrf_uarte_int_enable(p_libuarte->uarte, INTERRUPTS_MASK);

    NVIC_SetPriority(irqn, p_config->irq_priority);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);

    nrf_uarte_enable(p_libuarte->uarte);

    nrfx_timer_config_t tmr_config = NRFX_TIMER_DEFAULT_CONFIG;
    tmr_config.mode = NRF_TIMER_MODE_COUNTER;
    tmr_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret = nrfx_timer_init(&p_libuarte->timer, &tmr_config, tmr_evt_handler);
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
    IRQn_Type irqn = nrfx_get_irq_number(p_libuarte->uarte);

    if (p_libuarte->ctrl_blk->enabled == false)
    {   return;     }

    p_libuarte->ctrl_blk->enabled = false;

    NVIC_DisableIRQ(irqn);

    rx_ppi_disable(p_libuarte);
    tx_ppi_disable(p_libuarte);

    nrf_uarte_int_disable(p_libuarte->uarte, 0xFFFFFFFF);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXTO);

    nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STOPTX);
    nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STOPRX);

    while ( (p_libuarte->ctrl_blk->p_tx && !nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED)) ||
           (p_libuarte->ctrl_blk->p_cur_rx && !nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_RXTO)))
    {}
 
    p_libuarte->ctrl_blk->p_tx = NULL;
    p_libuarte->ctrl_blk->p_cur_rx = NULL;

    nrf_uarte_disable(p_libuarte->uarte);

    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ENDTX);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXSTARTED);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXTO);

#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE0)
    if (irqn == UARTE0_UART0_IRQn)
    {
        nrfx_prs_release(p_libuarte->uarte);
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRF_LIBUARTE_DRV_UARTE0)

    nrfx_timer_disable(&p_libuarte->timer);
    nrfx_timer_uninit(&p_libuarte->timer);

    ppi_free(p_libuarte);
}

ret_code_t nrf_libuarte_drv_tx(const nrf_libuarte_drv_t * const p_libuarte,
                               uint8_t * p_data, size_t len)
{
    if (p_libuarte->ctrl_blk->p_tx)
    {   return NRF_ERROR_BUSY;      }

    p_libuarte->ctrl_blk->p_tx = p_data;
    p_libuarte->ctrl_blk->tx_len = len;
    p_libuarte->ctrl_blk->tx_cur_idx = 0;
    uint16_t first_chunk;

    if ((MAX_DMA_XFER_LEN <= UINT16_MAX) && (len <= MAX_DMA_XFER_LEN))
    {   first_chunk = len;
        p_libuarte->ctrl_blk->tx_chunk8 = 0;
    }
    else
    {   uint32_t num_of_chunks = CEIL_DIV(len, MAX_DMA_XFER_LEN);
        p_libuarte->ctrl_blk->tx_chunk8 =  len/num_of_chunks;
        first_chunk = p_libuarte->ctrl_blk->tx_chunk8 + len%p_libuarte->ctrl_blk->tx_chunk8;
    }

    NRF_LOG_WARNING("Started TX total length:%d, first chunk:%d", len, first_chunk);
    nrf_uarte_tx_buffer_set(p_libuarte->uarte, p_data, first_chunk);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED);
    nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STARTTX);

    if ((MAX_DMA_XFER_LEN <= UINT16_MAX) && (len > MAX_DMA_XFER_LEN))
    {
        while(nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED) == 0)
        {           }       // wait for TX to start
        
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED);
        tx_ppi_enable(p_libuarte);

        nrf_uarte_tx_buffer_set(p_libuarte->uarte, &p_libuarte->ctrl_blk->p_tx[first_chunk], p_libuarte->ctrl_blk->tx_chunk8);
    }
    return NRF_SUCCESS;
}

ret_code_t nrf_libuarte_drv_rx_start(const nrf_libuarte_drv_t * const p_libuarte,
                                     uint8_t * p_data, size_t len, uint8_t * pktbuf, size_t pktlen)
{
    ASSERT(len <= MAX_DMA_XFER_LEN);

    if (p_libuarte->ctrl_blk->p_cur_rx)
    {   return NRF_ERROR_BUSY;      }

    p_libuarte->ctrl_blk->chunk_size = len;

    if (p_data)
    {   p_libuarte->ctrl_blk->p_cur_rx = p_data;
        nrf_uarte_rx_buffer_set(p_libuarte->uarte, p_data, len);
    }

    /* Reset byte counting */
    memset(&p_libuarte->ctrl_blk->packet_ctl, 0x00, sizeof(df_packet_ctl_t));
    memset(&p_libuarte->ctrl_blk->packet_stats, 0x00, sizeof(df_packet_stats_t));

    nrfx_timer_enable(&p_libuarte->timer);
    nrfx_timer_clear(&p_libuarte->timer);
    p_libuarte->ctrl_blk->last_rx_byte_cnt = 0;
    p_libuarte->ctrl_blk->last_pin_rx_byte_cnt = 0;

    p_libuarte->ctrl_blk->p_packet_buf = pktbuf;
    p_libuarte->ctrl_blk->packet_len = pktlen;

    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXSTARTED);

    rx_ppi_enable(p_libuarte);

    nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STARTRX);

    NRF_LOG_DEBUG("Start continues RX. Provided buffer:0x%08X", p_data);
    return NRF_SUCCESS;
}

void nrf_libuarte_drv_rx_buf_rsp(const nrf_libuarte_drv_t * const p_libuarte,
                                 uint8_t * p_data, size_t len)
{
    if (p_libuarte->ctrl_blk->p_next_rx == NULL)
    {
        p_libuarte->ctrl_blk->p_next_rx = p_data;
        NRF_LOG_DEBUG("RX buf response (next). Provided buffer:0x%08X", p_data);
        nrf_uarte_rx_buffer_set(p_libuarte->uarte, p_data, len);
    }
    else
    {
        NRF_LOG_DEBUG("RX buf response (mp_next_rx not NULL:0x%08X), Provided buffer:0x%08X", p_libuarte->ctrl_blk->p_next_rx, p_data);
        p_libuarte->ctrl_blk->p_next_next_rx = p_data;
    }
}

void nrf_libuarte_drv_rx_stop(const nrf_libuarte_drv_t * const p_libuarte)
{
    rx_ppi_disable(p_libuarte);

    NRF_LOG_DEBUG("RX stopped.");
    p_libuarte->ctrl_blk->p_cur_rx = NULL;
    nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STOPRX);
    nrfx_timer_disable(&p_libuarte->timer);
}

df_packet_stats_t * get_packet_stats(const nrf_libuarte_drv_t * const p_libuarte)
{   return(&p_libuarte->ctrl_blk->packet_stats);    }

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

static void irq_handler(const nrf_libuarte_drv_t * const p_libuarte)
{
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_RXDRDY))
    {   nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXDRDY);

        p_libuarte->ctrl_blk->packet_stats.total_chars += 1;

        p_libuarte->ctrl_blk->packet_ctl.curr_locn = nrfx_timer_capture_get(&p_libuarte->timer, DF_COUNT_CHAN_RXD_CHARS);    // get the count from the timer
        uint8_t char_in = *(char *)(p_libuarte->ctrl_blk->p_cur_rx + p_libuarte->ctrl_blk->packet_ctl.curr_locn - 1);        // get this byte from the rx buffer

        if (p_libuarte->ctrl_blk->packet_ctl.got_SOH)
        {   
            if (char_in == EOF_CHAR)   // packet finished .. bundle packet up to application layer
            {
                p_libuarte->ctrl_blk->packet_stats.EOF_cnt += 1;
                p_libuarte->ctrl_blk->packet_stats.total_packets += 1;
                p_libuarte->ctrl_blk->packet_ctl.got_SOH = false;
                p_libuarte->ctrl_blk->packet_len = (p_libuarte->ctrl_blk->packet_ctl.curr_locn - p_libuarte->ctrl_blk->packet_ctl.SOH_locn + 1);

                if (is_checksum_ok((p_libuarte->ctrl_blk->p_cur_rx + p_libuarte->ctrl_blk->packet_ctl.SOH_locn - 1) ,p_libuarte->ctrl_blk->packet_len))
                {   p_libuarte->ctrl_blk->packet_stats.good_packets += 1;
                
                    // move the packet into the packet buffer
                    memcpy(p_libuarte->ctrl_blk->p_packet_buf, 
                            (p_libuarte->ctrl_blk->p_cur_rx + p_libuarte->ctrl_blk->packet_ctl.SOH_locn - 1), 
                            p_libuarte->ctrl_blk->packet_len);
                
                    nrf_libuarte_drv_evt_t evt = {.type = NRF_LIBUARTE_DRV_EVT_GOT_PACKET};     // set the event

                    //nrf_libuarte_drv_rx_stop(p_libuarte);       // TODO should be able to remove this stop command

                    p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &evt);     // pass interrupt up to application layer
                }
                
            }
            else if (char_in == SOH_CHAR)        // another SOH .. reset and start again
            {   p_libuarte->ctrl_blk->packet_ctl.got_SOH = true;
                p_libuarte->ctrl_blk->packet_ctl.SOH_locn = p_libuarte->ctrl_blk->packet_ctl.curr_locn;
                p_libuarte->ctrl_blk->packet_stats.SOH_cnt += 1;
            }
            // else - nothing to do
        }
        else
        {   
            if (char_in == SOH_CHAR)
            {   p_libuarte->ctrl_blk->packet_ctl.got_SOH = true;
                p_libuarte->ctrl_blk->packet_ctl.SOH_locn = p_libuarte->ctrl_blk->packet_ctl.curr_locn;
                p_libuarte->ctrl_blk->packet_stats.SOH_cnt += 1;
            }
        }
    }
    
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_ERROR))
    {
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ERROR);
        
        p_libuarte->ctrl_blk->packet_stats.err_cnt += 1;

        nrf_libuarte_drv_evt_t evt = {
                .type = NRF_LIBUARTE_DRV_EVT_ERROR,
                .data = { .errorsrc = nrf_uarte_errorsrc_get_and_clear(p_libuarte->uarte) }
        };
        p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &evt);
    }

    // *** Rx Started
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_RXSTARTED))
    {
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_RXSTARTED);

        nrf_libuarte_drv_evt_t evt = {
                .type = NRF_LIBUARTE_DRV_EVT_RX_BUF_REQ,
        };
        p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &evt);
    }

    // *** Rx End (InBuf is full)  
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_ENDRX))
    {
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ENDRX);

        uint32_t endrx_byte_cnt = nrfx_timer_capture_get(&p_libuarte->timer, DF_COUNT_CHAN_RXD_TOTAL);
        uint32_t stop_byte_cnt = 0;

        uint32_t dma_amount = endrx_byte_cnt - p_libuarte->ctrl_blk->last_rx_byte_cnt;
        uint32_t pin_amount = stop_byte_cnt - p_libuarte->ctrl_blk->last_pin_rx_byte_cnt;
        NRF_LOG_DEBUG("(evt) RX dma_cnt:%d, endrx_cnt:%d, stop_cnt:%d",
                     dma_amount,
                     endrx_byte_cnt,
                     stop_byte_cnt);
        p_libuarte->ctrl_blk->last_rx_byte_cnt = endrx_byte_cnt;
        p_libuarte->ctrl_blk->last_pin_rx_byte_cnt = stop_byte_cnt;

        if (dma_amount || pin_amount)
        {
            uint32_t chunk0 = (dma_amount > p_libuarte->ctrl_blk->chunk_size) ?
                                p_libuarte->ctrl_blk->chunk_size : dma_amount;
            uint32_t chunk1 = dma_amount - chunk0;

            NRF_LOG_DEBUG("RX END chunk0:%d, chunk1:%d, data[0]=%d %d",
                         chunk0,
                         chunk1,
                         p_libuarte->ctrl_blk->p_cur_rx[0],
                         p_libuarte->ctrl_blk->p_cur_rx[1]);
            nrf_libuarte_drv_evt_t evt = {
                    .type = NRF_LIBUARTE_DRV_EVT_RX_DATA,
                    .data = {
                            .rxtx = {
                                .p_data = p_libuarte->ctrl_blk->p_cur_rx,
                                .length = chunk0
                            }
                    }
            };
            p_libuarte->ctrl_blk->p_cur_rx = p_libuarte->ctrl_blk->p_next_rx;
            p_libuarte->ctrl_blk->p_next_rx = NULL;
            if (p_libuarte->ctrl_blk->p_next_next_rx)
            {
                p_libuarte->ctrl_blk->p_next_rx = p_libuarte->ctrl_blk->p_next_next_rx;
                p_libuarte->ctrl_blk->p_next_next_rx = NULL;
                nrf_uarte_rx_buffer_set(p_libuarte->uarte,
                                        p_libuarte->ctrl_blk->p_next_rx,
                                        p_libuarte->ctrl_blk->chunk_size);
            }
            p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &evt);

            if ( chunk1 ||
                ((dma_amount == p_libuarte->ctrl_blk->chunk_size) && (endrx_byte_cnt == stop_byte_cnt)))
            {
                NRF_LOG_WARNING("RX END Chunk1:%d", chunk1);

                nrf_libuarte_drv_evt_t err_evt = {
                    .type = NRF_LIBUARTE_DRV_EVT_OVERRUN_ERROR,
                    .data = {
                            .overrun_err = {
                                    .overrun_length = chunk1
                            }
                    }
                };
                p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &err_evt);

                p_libuarte->ctrl_blk->p_cur_rx  = p_libuarte->ctrl_blk->p_next_rx;
                p_libuarte->ctrl_blk->p_next_rx = NULL;
            }
        }
    }

    // *** Tx Stopped ***
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED))
    {
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED);
        nrf_libuarte_drv_evt_t evt = {
           .type = NRF_LIBUARTE_DRV_EVT_TX_DONE,
           .data = {
               .rxtx = {
                   .p_data = p_libuarte->ctrl_blk->p_tx,
                   .length = p_libuarte->ctrl_blk->tx_len
               }
           }
       };
       p_libuarte->ctrl_blk->p_tx = NULL;
       p_libuarte->ctrl_blk->evt_handler(p_libuarte->ctrl_blk->p_context, &evt);
    }

    // *** End Tx ***
    if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_ENDTX))
    {
        nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_ENDTX);
        size_t amount = nrf_uarte_tx_amount_get(p_libuarte->uarte);

        NRF_LOG_DEBUG("(evt) TX completed (%d)", amount);
        p_libuarte->ctrl_blk->tx_cur_idx += amount;
        if (p_libuarte->ctrl_blk->tx_cur_idx == p_libuarte->ctrl_blk->tx_len)
        {
            nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTOPPED);
            nrf_uarte_task_trigger(p_libuarte->uarte, NRF_UARTE_TASK_STOPTX);
        }
        else
        {
            size_t rem_len = (p_libuarte->ctrl_blk->tx_len - p_libuarte->ctrl_blk->tx_cur_idx);
            if ( rem_len <= MAX_DMA_XFER_LEN)
            {
                tx_ppi_disable(p_libuarte);
            }
            else
            {   uint8_t * p_buffer = &p_libuarte->ctrl_blk->p_tx[p_libuarte->ctrl_blk->tx_cur_idx + p_libuarte->ctrl_blk->tx_chunk8];
                
                if (nrf_uarte_event_check(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED) == 0)
                {
                    NRF_LOG_ERROR("Tx not started yet!");
                    ASSERT(false);
                }
                nrf_uarte_event_clear(p_libuarte->uarte, NRF_UARTE_EVENT_TXSTARTED);
                nrf_uarte_tx_buffer_set(p_libuarte->uarte,
                                        p_buffer,
                                        p_libuarte->ctrl_blk->tx_chunk8);
            }
        }

    }
}

#if NRF_LIBUARTE_DRV_UARTE0
void libuarte_0_irq_handler(void)
{
    irq_handler(m_libuarte_instance[0]);
}
#endif

#if NRF_LIBUARTE_DRV_UARTE1
void UARTE1_IRQHandler(void)
{
    irq_handler(m_libuarte_instance[1]);
}
#endif
