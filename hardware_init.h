#ifndef DF_HARDWARE_INIT_H__
#define DF_HARDWARE_INIT_H__

#include "nrf_gpio.h"
#include "nrfx_clock.h"
#include "nrfx_timer.h"
#include "nrfx_i2s.h"

extern nrfx_timer_t pulse_counter;

// *** Input Pins ***
#define NOZZLE_PIN      NRF_GPIO_PIN_MAP(0,8)
#define PULSER_PIN      NRF_GPIO_PIN_MAP(0,7)
#define PSENSE_PIN      NRF_GPIO_PIN_MAP(1,13)         // !!! Need to init & an interrupt input processor (will trigger Panic_Save)
#define GPIN_PIN        NRF_GPIO_PIN_MAP(0,6)         // !!! Need to init & handler (polled is OK)

// *** Analog Pins ***
#define OR_SENSE_1      NRF_GPIO_PIN_MAP(0,4)        // !!! Need to Analog init & a handler tied in to the Pump Control state machine
#define OR_SENSE_2      NRF_GPIO_PIN_MAP(0,5)        // !!! Need to Analog init & a handler tied in to the Pump Control state machine

// *** Spare Pins ***
#define SPARE_1_PIN     NRF_GPIO_PIN_MAP(1,15)       // Spare pins .. init as outputs
#define SPARE_2_PIN     NRF_GPIO_PIN_MAP(0,3)
#define SPARE_3_PIN     NRF_GPIO_PIN_MAP(0,29)

// *** Output Pins ***
//#define RELAY_PIN       SPARE_3_PIN
#define RELAY_PIN       NRF_GPIO_PIN_MAP(1,9)   // TRACEDATA(3)
// #define RELAY_PIN       NRF_GPIO_PIN_MAP(0,12)   // TRACEDATA(1)
// #define RELAY_PIN       NRF_GPIO_PIN_MAP(0,11)   // TRACEDATA(2)

#define GPOUT_PIN       NRF_GPIO_PIN_MAP(0,2)         // !!! Need to init & handler (polled is OK)


// *** With Circuit board Mods - Piggyback Mag_Sense onto Mode_PB
//#define MODE_PB_PIN  NRF_GPIO_PIN_MAP(0,20)
//#define MAG_SENSE_PIN NRF_GPIO_PIN_MAP(0,20)
//#define BOOTLOADER_PIN  NRF_GPIO_PIN_MAP(0,22)
// *** No Circuit board Mods shorting under the processor module ()
#define MODE_PB_PIN     NRF_GPIO_PIN_MAP(0,20)
#define MAG_SENSE_PIN   NRF_GPIO_PIN_MAP(0,22)        // !!! Need to init & handler - currently copy MODE_PIN
#define BOOTLOADER_PIN  NRF_GPIO_PIN_MAP(0,19)

#define BSP_SELF_PINRESET_PIN BOOTLOADER_PIN

// *** LED Configuration ***
#define PROC_LED_PIN    NRF_GPIO_PIN_MAP(1,10)         // This is the I2S SDOUT pin

// *** I2S Configuration (underpins the LED control) ***
// i2s required pins
#define I2S_SCK_PIN     NRF_GPIO_PIN_MAP(1,3)         // These are Module Pins 60, 56, 59, 57 i.e. the ones under the middle of the module 
#define I2S_LRCK_PIN    NRF_GPIO_PIN_MAP(1,4)
#define I2S_SDOUT_PIN   PROC_LED_PIN
// i2s optional pins
#define I2S_MCK_PIN     NRFX_I2S_PIN_NOT_USED         // Other i2s pins are unused .. assign them to the unused pins underneath the module
#define I2S_SDIN_PIN    NRFX_I2S_PIN_NOT_USED

// Each LED color is considered a separate LED
#define NUM_LEDS        3

#define HB_PIN          NRF_GPIO_PIN_MAP(1,11) 
#define HB_ERR_PIN      NRF_GPIO_PIN_MAP(1,12) 

#define LED_HB          0       // Array indexes for the LEDs
#define HB_ERR          1  
#define RELAY_OUT       2 

#define LED_PIN_LIST { HB_PIN, HB_ERR_PIN, RELAY_PIN}

// *** Comms (Port 1) ***
#define RX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,30)  
#define TX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,28)
#define TX_ENABLE      NRF_GPIO_PIN_MAP(0,31)       

//#define RX_PIN_NUMBER  SPARE_1_PIN
//#define TX_PIN_NUMBER  SPARE_2_PIN 
//#define TX_ENABLE      SPARE_3_PIN


#define RTS_PIN_NUMBER TX_ENABLE

#define CTS_PIN_NUMBER NRF_GPIO_PIN_MAP(1,01)    // set these to unused pins (flow control is off so the pins won't be used) 1.01 is Module Pin 61
//#define RTS_PIN_NUMBER NRF_GPIO_PIN_MAP(0,15)    // is RTS enough to enable txEnable on rs485?


// *** Comms (Port 2) ***
//#define RX2_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,13)  
// #define TX2_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,15)

#define HWFC           true                    // hardware flow control

#define DF_SOFTDEV_TIMER_INST   0               // Reserved Timer Instance for the SoftDevice
#define DF_PULSER_TIMER_INST    1               // Timer Device Instance for the Pulse Counter
#define DF_ANALOG_TIMER_INST    2               // Timer Device Instance for the SAADC (analog)

#define NUM_NOZZLES     1
#define NUM_PBS         2                       // Mode and MagSense are different but they may do the same thing
#define MODE_PB_IDX     0
#define MAG_PB_IDX      1
#define NUM_PULSES      1
#define NUM_RELAYS      1
#define NUM_GPIS        1
#define NUM_GPOS        1
#define NUM_PSENSE      1

typedef struct
{   // Inputs
    uint8_t nozzle[NUM_NOZZLES];
    uint8_t nozzle_pin[NUM_NOZZLES];

    uint8_t gpin[NUM_GPIS];
    uint8_t gpin_pin[NUM_GPIS];

    uint8_t pushbutton[NUM_PBS];
    uint8_t pushbutton_pin[NUM_PBS];
    uint8_t pushbutton_time[NUM_PBS];

    uint8_t or_sense[NUM_RELAYS];
    uint32_t or_sense_val[NUM_RELAYS];  

    uint32_t pulses[NUM_PULSES];
    uint32_t pulses_last[NUM_PULSES];  

    uint8_t psense[NUM_PSENSE];
    uint8_t psense_pin[NUM_PSENSE];


    // Outputs
    uint8_t relay[NUM_RELAYS];
    uint8_t gpout[NUM_GPOS];

} hardware_t;
extern hardware_t hardware;

extern uint8_t mode_button_LED;

/**
 * @brief Function for initializing the Distributed Device Hardware.
 *
 * After initialization, all outputs are in a safe state.
 *
 * @retval NRF_SUCCESS                          If the procedure was successful.
 * @retval NRF_ERROR_MODULE_ALREADY_INITIALIZED If the driver was already initialized.
 */
ret_code_t df_hardware_init(void);


/**
 * @brief Function to Trigger the Bootloader
 */
void trigger_bootloader(void);

/**
 * @brief Function to Set/Clear the Relay output
 *
 * @param[in] channel - the Relay control channel. Can use 0 for single-channel device
 * @param[in] state - 0 to turn OFF, any other value to turn ON
 */
void df_relay_change(uint32_t channel, uint8_t state);

/**
 * @brief Function to Set/Clear the Output buffer control. 
 *        This is the OE pin of the 3V-5V level shifter and enables most I/O control.
 *
 * @param[in] channel - the OutBuf control channel. Can use 0 for single-channel device
 * @param[in] state - 0 to turn OFF, any other value to turn ON
 */
void df_outbuf_change(uint32_t channel, uint8_t state);

/**
 * @brief System Clock intialisation
 *
 * General-purpose system clock, for LED_Control and NV Storage
 * 
 */
void clock_init(void);

void clock_handler(nrfx_clock_evt_type_t event);

/**
 * @brief System Timer intialisation
 *
 * General-purpose system timer, used for LED_Control and NV Storage
 * 
 */
void timer_init(void);


/**
* @brief Function for requesting a random number
*   It can be called periodically e.g. from the system tick timer
*/
void update_rng(void);


/**
* @brief extends the std nrf_gpio_cfg_output() to allow drive strength to be defined
*/
void df_gpio_cfg_output(uint32_t pin_number, uint32_t drive_type);

#endif // DF_HARDWARE_INIT_H__