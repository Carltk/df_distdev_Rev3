
#include "analog.h"

#include "nrfx_saadc.h"
#include "nrfx_ppi.h"
#include "nrf_timer.h"
#include "nrfx_timer.h"

#include "nrfx_log.h"

// Include DF headers
#include "hardware_init.h"

#define SAMPLES_IN_BUFFER 5
volatile uint8_t analog_state = 1;

static const nrfx_timer_t analog_timer = NRFX_TIMER_INSTANCE(DF_ANALOG_TIMER_INST);
static const nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t              m_adc_evt_counter;

static nrf_ppi_channel_t analog_ppi;

void analog_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
     NRFX_LOG_INFO("Analog timer tick");

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    // Make a timer to handle the analog triggering
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;      // Make the timer config structure & fill with defaults
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;                   // make changes from default
    
    err_code = nrfx_timer_init(&analog_timer, &timer_cfg, analog_timer_handler);   // Initialise the timer
    APP_ERROR_CHECK(err_code);

    //uint32_t ticks = nrfx_timer_ms_to_ticks(&analog_timer, 400);
    uint32_t ticks = nrfx_timer_ms_to_ticks(&analog_timer, 400);
    nrfx_timer_extended_compare(&analog_timer, NRF_TIMER_CC_CHANNEL2, ticks,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, false);
    nrfx_timer_enable(&analog_timer);
    nrfx_timer_resume(&analog_timer);

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&analog_timer, NRF_TIMER_CC_CHANNEL2);
    uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();

    err_code = nrfx_ppi_channel_alloc(&analog_ppi);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_assign(analog_ppi, timer_compare_event_addr, saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{   ret_code_t err_code = nrfx_ppi_channel_enable(analog_ppi);
    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        uint32_t avg_val = 0;
        ret_code_t err_code;

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRFX_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
            avg_val += p_event->data.done.p_buffer[i];
        }
        avg_val = avg_val / SAMPLES_IN_BUFFER;

        NRFX_LOG_INFO("ADC event number: %d, Avg: %d", (int)m_adc_evt_counter, avg_val);

        m_adc_evt_counter++;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    
    nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    //channel_config.gain = NRF_SAADC_GAIN1;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

/**
 * @brief Initialise the analog subsystem for override sensing
 */
ret_code_t df_or_sense_init(void)
{   ret_code_t ret = NRFX_SUCCESS;

    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    NRF_LOG_INFO("SAADC (analog) process started");

    return (ret);
}

