
#include <stdlib.h>
#include <string.h>
#include "nrfx_log.h"

#include "df_routines.h"
#include "hardware_init.h"
#include "console_comms.h"
#include "led_control.h"
#include "nv_store.h"
#include "application.h"

// *** general comms defines ***
#define STUFF_CHAR      0xD0
#define SOH_CHAR        0xD1
#define EOF_CHAR        0xD2
#define MSG_DELIM       0xD3

#define MSG_MIN_SIZE    5

#define MSGBUF_TYPE     0  
#define MSGBUF_ADDR     1
#define MSGBUF_COMMAND  3
#define MSGBUF_PAYLOAD  4

#define RXBUF_TYPE      1  
#define RXBUF_ADDR      2
#define RXBUF_COMMAND   4
#define RXBUF_PAYLOAD   5


// *** vars and structures ***
static uint16_t DevAddress = DISCOVERY_DFLT_ADDR;

uint32_t baud_list[NUM_BAUD_RATES] = BAUD_LIST;

// Message received from comms
char rx_buf[RX_BUF_SIZE];    
con_comms_t con_comms = RX_DATA_DEFAULT;

// Processed received message (i.e. no SOH, EOF)
char msg_buf[RX_BUF_SIZE];    
static msg_data_t msg_data = MSG_DATA_DEFAULT;

// Message sent  out
char tx_buf[TX_BUF_SIZE];
uint8_t tx_char_count;

comms_context_t comms_context = COMMS_CONTEXT_DEFAULT;

NRF_LIBUARTE_DRV_DEFINE(intbus, DF_PORT_INTBUS, DF_COMMS_TIMER_INST);


//NRF_QUEUE_DEF(buffer_t, m_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);


// *** function prototypes ***
//void init_rx_struct(void);
//void init_msg_struct(void);

void comms_evt_handler(void * context, nrf_libuarte_drv_evt_t * p_evt);
ret_code_t swap_baud(const nrf_libuarte_drv_t * const p_libuarte);

rx_state_t get_comm_state(con_comms_t *rd);
void con_comms_handler(con_comms_t *rd, char c);
msg_state_t get_msg_data(msg_data_t *md, con_comms_t *rd);
bool is_checksum_ok(con_comms_t *rd);
uint8_t make_lrc(char *buf, uint8_t cnt);
void interpret_msg(msg_data_t *md);
uint8_t handle_action_cmd(msg_data_t *md, char *buf);
uint8_t send_attr_bytes(msg_data_t *md, char *buf, char bytes_reqd);

void clear_rx_bufs(void);

// size_t ConsoleWrite(char *buf, uint8_t count);

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}


#define SERIAL_BUF_TX_SIZE 1           // The number of chars the uart can process before interrupt
#define SERIAL_BUF_RX_SIZE 1

ret_code_t ConsoleSerialPortInit(void)
{   ret_code_t ret = NRFX_SUCCESS;

    con_comms.comms_state = COMMS_INIT;

    nrf_gpio_pin_clear(TX_ENABLE);    

    nrf_libuarte_drv_config_t nrf_libuarte_drv_config = {
        .tx_pin     = TX_PIN_NUMBER,
        .rx_pin     = RX_PIN_NUMBER,
        .rts_pin = RTS_PIN_NUMBER,      
        .cts_pin = CTS_PIN_NUMBER,      
        .hwfc = NRF_UARTE_HWFC_DISABLED,          ///< Flow control configuration.
        .parity = NRF_UARTE_PARITY_EXCLUDED,        ///< Parity configuration.
        .baudrate = NRF_UARTE_BAUDRATE_19200,   ///< Baud rate.
        .irq_priority = APP_IRQ_PRIORITY_LOW,  ///< Interrupt priority.
        .pullup_rx = 1,     ///< Pull up on RX pin.
        //.startrx_evt,   ///< Event to trigger STARTRX task in UARTE.
        //.endrx_evt,     ///< Event to trigger STOPRX task in UARTE.
        //.rxstarted_tsk, ///< Task to be triggered when RXSTARTED UARTE event occurs.
        //.rxdone_tsk,    ///< Task to be triggered when ENDRX UARTE event occurs.
    };

    ret = nrf_libuarte_drv_init(&intbus, &nrf_libuarte_drv_config, &comms_evt_handler, &comms_context);
    
    if (ret != NRFX_SUCCESS)
    {    NRFX_LOG_INFO("Serial init failure [%x]", ret);  }

    NRFX_LOG_INFO("Serial (IntBus) Port - Initialised at baud [%d]", nrf_libuarte_drv_config.baudrate); 

    con_comms.comms_state = COMMS_DISCONNECTED;

    return(ret);
}

void comms_evt_handler(void * context, nrf_libuarte_drv_evt_t * p_evt)
{
    ret_code_t ret;
    char c;
    uint32_t chars_read;
    uint32_t err = 0;

    switch (p_evt->type)
    {
        case NRF_LIBUARTE_DRV_EVT_RX_DATA:    ///< Data received.
NRF_LOG_INFO("Comms: Data Received");                                    

            //ret = nrf_serial_read(p_serial, &c, sizeof(c), &chars_read, NRF_SERIAL_MAX_TIMEOUT);
            //ret = nrf_serial_read(p_serial, &c, sizeof(c), &chars_read, 0);
            if (ret == NRFX_SUCCESS)
            {   
                if ((con_comms.rx_char_count == 0) && (c != SOH_CHAR))
                {   if (inc_is_error(&con_comms.err_count, COMMS_TH_NOT_SOH))
                    {   // NRF_LOG_INFO("Too many Serial errors [%d]", COMMS_TH_NOT_SOH);            
                        swap_baud(&intbus);    }
                    break;
                }
            
                if (nrf_gpio_pin_read(TX_ENABLE) == 0)      // Only receive data when I'm not transmitting
                {   con_comms_handler(&con_comms, c);  }

                if (get_comm_state(&con_comms) == RX_GOT_EOF)
                {   
                    if (con_comms.comms_state < COMMS_VISIBLE) 
                    {   con_comms.comms_state = COMMS_VISIBLE;  }
                
                    con_comms.err_count = 0;
                    get_msg_data(&msg_data, &con_comms);
                }
            }
            else
            {   clear_rx_bufs();    }

            break;        
        case NRF_LIBUARTE_DRV_EVT_RX_BUF_REQ: ///< Requesting new buffer for receiving data.
NRF_LOG_INFO("Comms: Rx buf request");                        
            clear_rx_bufs();
            break;
        case NRF_LIBUARTE_DRV_EVT_TX_DONE:     ///< Requested TX transfer completed.
NRF_LOG_INFO("Comms: Tx done event");                                    
            nrf_gpio_pin_clear(TX_ENABLE);           // turn off the TxEnable pin
            clear_rx_bufs();
            ret = nrf_libuarte_drv_rx_start(&intbus, rx_buf, RX_BUF_SIZE, false);     // re-enable the receiver

            if (ddpc.nv_immediate.dev_address == DISCOVERY_DFLT_ADDR)                           // At the moment only check for collisions in adoption
            {   if (memcmp(&tx_buf, con_comms.rx_buf, tx_char_count - 1) != 0)                  // Check for data collisions
                {   con_comms.tx_collision = 1;   }                                             // Flag if found
                else
                {   ddpc.nv_immediate.dev_address = con_comms.discovery_temp_addr;     }        // No collision .. set the address to the temporary Adoption address
            }
            break;
        case NRF_LIBUARTE_DRV_EVT_ERROR:      ///< Error reported by the UARTE peripheral.
NRF_LOG_INFO("Comms: Error");                        
        case NRF_LIBUARTE_DRV_EVT_OVERRUN_ERROR:    ///< Error reported by the driver.
NRF_LOG_INFO("  - Overrun Error");                        
            con_comms.comms_state = COMMS_ERROR;
            clear_rx_bufs();

            if (inc_is_error(&con_comms.err_count, COMMS_TH_ERR))
            {   swap_baud(&intbus);                }

            break;
        default:
NRF_LOG_INFO("Comms: Unknown event [%x]", p_evt->type);                                               
            break;
    }
}

rx_state_t get_comm_state(con_comms_t *rd)
{   return(rd->rx_state);   }

void con_comms_handler(con_comms_t *rd, char c)
{   char d = c;

    switch (rd->rx_state)
    {   
        case RX_IS_IDLE:
            if (c == SOH_CHAR)
            {   rd->rx_char_count = 0;                  // set intial state of char counter
                rd->rx_state = RX_GOT_SOH;              // advance state
            }
            break;
        case RX_GOT_STUFF:
            d = (c | 0x80);                             // pre-process the incoming character if a stuff char has been received
        case RX_GOT_SOH:
            if (c == STUFF_CHAR)
            {   rd->rx_state = RX_GOT_SOH;  }
            else if (c == EOF_CHAR)
            {   rd->rx_state = RX_GOT_EOF;  }
            // else .. normal char .. ust save it
            break;
        default:
            goto no_save;
            break;
    }
    rd->rx_buf[rd->rx_char_count] = d;
    rd->rx_char_count += 1;
    rd->rx_buf[rd->rx_char_count] = 0;   // clear the next char to keep a null-term buffer 

no_save:
    return;
}

msg_state_t get_msg_data(msg_data_t *md, con_comms_t *rd)
{
    msg_state_t retval = MSG_INCOMPLETE;

    if (rd->rx_state == RX_GOT_EOF)
    {    
        if (rd->rx_char_count >= MSG_MIN_SIZE)
        {
            md->msg_type = rd->rx_buf[RXBUF_TYPE];

            if (isInCharArray(md->msg_type, ddpc.dev_type, NUM_DEV_TYPES))              // See if the message type matches one in my array of acceptable types
            {   md->msg_addr = buf2int(&rd->rx_buf[RXBUF_ADDR]);              // grab the address from the message

                if (md->msg_addr == DISCOVERY_DFLT_ADDR)                        // If it's the Adoption address
                {   
                    if (con_comms.comms_state < COMMS_ADOPTING)
                    {    con_comms.comms_state = COMMS_ADOPTING;    }

                    if (ddpc.nv_immediate.dev_address & 0xFF00)                 // and only if we haven't got a real (adopted) address
                    {   NRF_LOG_INFO("Comms Discovery, Holdoff [%d]", rd->discovery_holdoff);                                  
                        if (rd->rx_buf[RXBUF_COMMAND] == FD_CMD_DISCOVER)       // -> Only respond to Discovery commands
                        {   
                            rd->discovery_holdoff -= 1;                         // -> dec the holdoff counter
                            if (!rd->discovery_holdoff)                         //   .. if we get to 0
                            {   NRF_LOG_INFO("Got Comms Discovery, addr [%x:%x]. Cmd [%x]", md->msg_addr, ddpc.nv_immediate.dev_address, rd->rx_buf[RXBUF_COMMAND]);                                  
                                md->msg_state = retval = MSG_COMPLETE;          //    -> answer the message
                                rd->discovery_holdoff = ((ddpc.my_rnd >> 4) + 1);  // update the holdoff in case we have to go around again
                                update_rng();                                   // and get a new random holdoff for next time
                            }
                            else
                            {   md->msg_state = retval = MSG_DROPPED;   }       // Haven't reached the holdoff count .. drop the message
                        }
                        else
                        {   md->msg_state = retval = MSG_DROPPED;   }           // In discovery & not a discovery command .. drop the message
                    }
                }
                else if (md->msg_addr == ddpc.nv_immediate.dev_address)                      // See if it is my real address. This needs to be after Discovery to stop answering Polls in Discovery mode.
                {   md->msg_state = retval = MSG_COMPLETE;  }               // -> continue to processing of the message
                else if (md->msg_addr == ALL_CALL_ADDR)
                {   md->msg_state = retval = MSG_COMPLETE;  }       // -> good Type & Address
                else
                {   md->msg_state = retval = MSG_BAD_ADDR;  }       // message for another device, output error message
            }
            else            // Bad Type
            {   md->msg_state = retval = MSG_BAD_TYPE;  }           // message for another device, output error message
        }
        // else  -- not enough chars
    }
    
    if (retval > MSG_INCOMPLETE)
    {
        if (retval == MSG_COMPLETE)
        {   
            if (is_checksum_ok(rd) == true)                                     // check the msg checksum
            {   memcpy(md->msg_buf, &rd->rx_buf[1], (rd->rx_char_count - 2));   // All good .. copy the message into the msg_buf
                md->msg_char_count =  rd->rx_char_count - 2;   
                md->msg_state = retval = MSG_OK;                                // and set the return value
                // NRF_LOG_DEBUG("Message for me - [%x:%x]. Data [%x]", md->msg_type, md->msg_addr, *md->msg_buf);      
                interpret_msg(md);
            }
            else
            {   md->msg_state = retval = MSG_BAD_CS;                           // bad message, output error message
                NRF_LOG_DEBUG("Comms Msg bad CS");      

            }
        }

        clear_rx_bufs();
        rd->rx_state = RX_IS_IDLE;         // Clear the rx state flag so we can get new messages
    }

    return(retval);
}

void interpret_msg(msg_data_t *md)
{
    char buf[TX_BUF_SIZE];
    uint8_t char_cnt;
    bool send_resp = false;
    uint8_t comp_type = (md->msg_buf[MSGBUF_TYPE] - DEV_TYPE_PUMP);
    uint8_t a, c;

    #define MSG_PREAMBLE 4    

    char_cnt = MSG_PREAMBLE;              // reflect the first 4 chars from the incomming message back to the output
    memcpy(buf, md->msg_buf, char_cnt);   // preload the output buffer with type.1, addr.2, cmd.1

    // NRF_LOG_DEBUG("Rx %d. Cmd %x", char_cnt, md->msg_buf[MSGBUF_COMMAND]);      

    switch (md->msg_buf[MSGBUF_COMMAND])
    {
        case FD_CMD_RESET:              // reset request
            if (md->msg_char_count == MSG_MIN_SIZE + 1)
            {   if (md->msg_buf[MSGBUF_PAYLOAD] == 0xFF)    // Check for 0xFF payload (Factory Default)
                { do_factory_default(false);     }           // !!!CK Here - "true" here resets Flash .. is this a valid Production operation?
            }    
            send_resp = true;
            do_sys_reset();
            break;
        case FD_CMD_DESC:               // Fetch description of the Controller
            break;
        case FD_CMD_STAT: 		// Fetch controller Status
            send_resp = true;
            buf[char_cnt++] = ((comp_type == 0) ? pump.pump_status : components.comp_status);     // send the correct status for the message devicetype
            buf[char_cnt++] = ((comp_type == 0) ? pump.pump_state : 0);                            // send the correct state for the message devicetype
            
            if (ddpc.nv_immediate.dev_address & 0xFF00)                                         // If it's a temporary address .. set to "FOSTERED"
            {    con_comms.comms_state = COMMS_FOSTERED;     }
            else
            {    con_comms.comms_state = COMMS_ONLINE;   }                                      // Permanent address .. set to ONLINE
            
            break;
        //case FD_CMD_WRATTR_BYTE: 	// Write Attribute byte
        //    break;
        case FD_CMD_RDATTR_BYTE: 	// Read Attribute byte
            send_resp = true;
            memcpy(&buf[char_cnt], &md->msg_buf[char_cnt], 3);   // Move the Attrib Address (2 bytes) and count to the output buffer
            char_cnt += 3;                                       // adjust the count   
            char_cnt += send_attr_bytes(md, &buf[char_cnt], buf[(char_cnt - 1)]);     // load in the required bytes
            break;
        //case FD_CMD_WRATTR_BIT:         // Write Attribute bits
        //case FD_CMD_RDATTR_BIT:         // Read Attribute bits
        //case FD_CMD_WRSTAT_BYTE: 	// Write Status bytes
        //case FD_CMD_RDSTAT_BYTE: 	// Read Status bytes
        //case FD_CMD_WRSTAT_BIT:         // Write Status bits
        //case FD_CMD_RDSTAT_BIT:         // Read Status bits
        //case FD_CMD_RDEVT: 		// Read controller Event
        //case FD_CMD_PRG_S: 		// Program S-Record
        //case FD_CMD_TIME: 		// Set the time
        //case FD_CMD_CLRAPP: 		// Clear the application area and Reset controller to bootloader mode
        //    break;
        case FD_CMD_PING: 		// Ping the controller Controller
            send_resp = true;
            buf[char_cnt++] = 0x01;     // send "application running" response (0=app not running)
            break;
        // case FD_CMD_CS:                 // Fetch the KbdLCD Controller checksum
        case FD_CMD_LDAPP: 		// Load the new application and reset
            NRFX_LOG_INFO("Unknown Command Rx [%x]", md->msg_buf[MSGBUF_COMMAND]);
            break;
        case FD_CMD_DISCOVER:
            con_comms.discovery_temp_addr = buf2int(&md->msg_buf[MSGBUF_PAYLOAD]);    // Message contains the temporary address 0xFF<TempIndex> in payload (also capture the temporary address)
            ddpc.nv_immediate.dev_address = buf2int(&md->msg_buf[MSGBUF_PAYLOAD]);  // Message contains the temporary address 0xFF<TempIndex> in payload (also capture the temporary address)
            flash_control.do_immediate_save = true;

            int2buf(&buf[MSGBUF_ADDR], con_comms.discovery_temp_addr);                // we will respond FROM that message .. swap address payload into output address field
            
            NRFX_LOG_INFO("DDPC Addr changed to [%x]", ddpc.nv_immediate.dev_address);
            
            if ((buf[MSGBUF_TYPE] & 0xFE) == 0xFE)                  // If the message was TO Discovery or AllCall type
            {   buf[MSGBUF_TYPE] = ddpc.dev_type[0];                   // replace type with my primary type for the response message
                NRFX_LOG_INFO("DDPC Type requested [%x]", buf[MSGBUF_TYPE]);
            }

            buf[char_cnt++] = ddpc.my_rnd;                          // add the system random number to the msg (makes collision-detection more obvious)
            send_resp = true;
            break;
        case FD_CMD_IDENT:
            if (buf[MSGBUF_PAYLOAD] & IDENT_LED)
            {   
                wipeAllLEDSlots();
                a = addLEDPattern(PROC_LED_ORANGE, LED_FLASH_MED, 8, 32, 0xFF);
                c = addLEDPattern(PROC_LED_GREEN, LED_FLASH_MED, 8, 32, a);
                c = addLEDPattern(PROC_LED_BLUE, LED_FLASH_MED, 8, 32, c);
                loopLEDPattern(a, c);
            }

            if (buf[MSGBUF_PAYLOAD] & IDENT_RELAY)
            {   if (pump.pump_state == PUMP_STATE_IDLE)         // Only init Relay test if the Pump controller is Idle
                {   hardware.relay_test[0] = 5;                 // relay test will run for for 5 seconds (application second timer will turn it off)
                    df_relay_change(hardware.relay[0], 1);      // trun the relay on
                }
            }

            break;     
        case FD_CMD_ACTION:
            c = handle_action_cmd(md, &buf[char_cnt]);        // act on the command (return val is the size of the reply msg (0 for no reply))
            if (c)
            {   send_resp = true;
                char_cnt += c;
            }
            break;
        default:
            NRFX_LOG_INFO("Unknown Command Rx [%x]", md->msg_buf[MSGBUF_COMMAND]);
            break;
    }

    if (send_resp = true)
    {   
        c = make_lrc(buf, char_cnt);
        buf[char_cnt++] = c;
        char_cnt = ConsoleWrite(NULL, buf, char_cnt);  
        
        if (buf[MSGBUF_COMMAND] != 0x02)
        {   NRFX_LOG_INFO("Tx [%d] chars, Cmd [%x]", char_cnt, buf[MSGBUF_COMMAND]);    }
    }
}

uint8_t handle_action_cmd(msg_data_t *md, char *buf)
{   uint8_t retval = 0;
    uint8_t cmd = md->msg_buf[MSGBUF_PAYLOAD];

    NRFX_LOG_INFO("Action [%x] Received", cmd);

    switch (cmd)
    {   case PUMP_CMD_STATUS:                   // Request for Pump Status
            buf[0] = cmd;
            buf[1] = pump.pump_state;           // Write the status into the buffer
            retval = 2;
            break;
        case PUMP_CMD_EMERG_STOP:
            cmd = PUMP_CMD_STOP;                // Convert EmergencyStop Msg to a Stop message
        case PUMP_CMD_AUTHORISE:                // Authorise a transaction
        case PUMP_CMD_STOP:                     // Go to Paying state 
        case PUMP_CMD_PAID:                     // Clear Paying state & go to Stopped
            pump_action_cmd(&pump, cmd);
            buf[0] = cmd;
            buf[1] = pump.pump_state;           // Write the status into the buffer
            retval = 2;
            break;
        case PUMP_CMD_SEND_TRANS:               // Request of a Transaction Data message
        case PUMP_CMD_TOTALS:                   // Short Trans message (BUSY) or Totals Data Message (Other states)
        case PUMP_CMD_BUSY_TOTALS:              // Send Short Trans Message (while pump is BUSY)
            buf[0] = cmd;            
            buf[1] = pump.pump_state;           // Write the status into the buffer
            long2buf(&buf[2], pump.transaction_count);
            pump.pump_status &= ~PUMP_STATUS_PULSES;    // Clear the "new pulses" flag when count is read
            retval = 6;
            break;
        case PUMP_CMD_DATA_MSG:                 // Sending Data to the device 
            // Do I need to send Grade, Timeouts etc?
            break;
        case PUMP_CMD_ALL_LIGHTS_ON:            // Will there ultimately be GPIOs to control here?    
        case PUMP_CMD_ALL_LIGHTS_OFF:           // Will there ultimately be GPIOs to control here?
        //case PUMP_CMD_EXCH_MSG:               // 2-way data exchange .. not supported
        //case PUMP_CMD_GO_TO_DATA:             // Go to data state .. not suported
        default:
            break;
    }
    
    return(retval);
}

uint8_t send_attr_bytes(msg_data_t *md, char *buf, char bytes_reqd)
{   uint8_t bytes = bytes_reqd;
    uint16_t attr_addr = buf2int(&md->msg_buf[MSGBUF_PAYLOAD]);

    switch (attr_addr)
    {   case DDPC_ATTRIB_UPTIME:
            long2buf(buf, ddpc.nv_panic.uptime_mins);
            break;
        case DDPC_ATTRIB_BOOTS:
            long2buf(buf, ddpc.nv_panic.boot_count);
            break;
        case DDPC_ATTRIB_FLASH_WR:
            long2buf(buf, ddpc.nv_panic.flash_writes);
            break;
        case DDPC_ATTRIB_TEMP:
            long2buf(buf, ddpc.curr_temp);
            break;
        case DDPC_ATTRIB_RND:
            buf[0] = ddpc.my_rnd;
            break;
        case DDPC_ATTRIB_PUMPSTAT:
            buf[0] = pump.pump_status;
            break;
        case DDPC_ATTRIB_NOZZLE:
            buf[0] = pump.nozzle;
            break;
        case DDPC_ATTRIB_RELAY:
            buf[0] = pump.relay;
            break;
        case DDPC_ATTRIB_PULSES:
            long2buf(buf, pump.transaction_count);
            pump.pump_status &= ~PUMP_STATUS_PULSES;        // Clear the pulses flag when count is read
            break;
        case DDPC_ATTRIB_PUMP_STATE:
            buf[0] = pump.pump_state;
            break;
        default:
            bytes = 0;
            break;
    }

    return(bytes);
}


bool is_checksum_ok(con_comms_t *rd)
{
    uint8_t i;
    uint8_t lrc;
    bool retval = false;                                    // default result to "bad"
    
    if (rd->rx_char_count >= MSG_MIN_SIZE)                  // buf will hold rx_char_count chars .. make sure there is enough for calculation
    {   lrc = make_lrc(&rd->rx_buf[RXBUF_TYPE], (rd->rx_char_count - 3));  // Calc the LRC
        if (lrc == rd->rx_buf[(rd->rx_char_count - 2)])       // check the lrc against the message
        {   retval = true;  }                               // .. good? say so
    }
    return(retval);
}

uint8_t make_lrc(char *buf, uint8_t cnt)
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


uint8_t MakeSimpleCRC(uint8_t Byte, uint8_t History)
{
    History = History ^ Byte;  	// XOR history with new byte
    History = History ^ 0xFF;   	// Complement the result
    if (History >= 0x0080)
    {	History <<= 1;    			// Shift Byte left
   	History |= 0x01;         	// High bit was set.. move high bit to low bit
    }
    else
    {	History <<= 1;    			// Shift Byte left
    }
    History = History + Byte;
    return(History);
}


void clear_rx_bufs(void)
{
    //nrf_serial_rx_drain(p_serial);             // clear out received chars
    //nrf_queue_reset(&serial_queues_rxq);
    con_comms.rx_buf[0] = 0;                              // Wipe the SOH character from the buffer
    con_comms.rx_buf[(con_comms.rx_char_count - 1)] = 0;    // Wipe the EOF character from the buffer    
    con_comms.rx_char_count = 0;
    con_comms.rx_state = RX_IS_IDLE;
    
}

uint8_t IsStuffable(char theChar)
{   uint8_t retval = 0;
    
    if ((theChar >=  STUFF_CHAR) && (theChar <=  MSG_DELIM))
    {    retval = 1;    }
    
    return(retval);
}


size_t ConsoleWrite(const nrf_libuarte_drv_t * const p_libuarte, char *buf, uint8_t count)
{   size_t sent = 0;
    tx_char_count = 0;
    struct nrf_serial_s const * tp_serial;


    if (ddpc.nv_immediate.dev_address != DISCOVERY_DFLT_ADDR)           // At the moment only check for collisions in adoption
    {   nrf_libuarte_drv_rx_stop(p_libuarte);   }                         // disable the comms receiver when we're not in adoption mode

    clear_rx_bufs();
    
    tx_buf[tx_char_count++] = SOH_CHAR;

    // add stuff characters into the output buffer if required
    for (uint8_t i=0;i<count;i++)                  
    {   if (IsStuffable(buf[i]))
        {   tx_buf[tx_char_count++] = STUFF_CHAR; 
            tx_buf[tx_char_count++] = buf[i] & 0x7F;
        }
        else
        {   tx_buf[tx_char_count++] = buf[i];   }
    }    

    tx_buf[tx_char_count++] = EOF_CHAR;

    nrf_gpio_pin_set(TX_ENABLE);          // Activate  TxEnable pin

    nrf_libuarte_drv_tx(p_libuarte, buf, count);

    return(sent);
}

ret_code_t swap_baud(const nrf_libuarte_drv_t * const p_libuarte)
{   ret_code_t ret; 

    // TODO - reimplement baud swapping
    // wrapping_inc((uint32_t*)&con_comms.baud_index, 0, NUM_BAUD_RATES);
    NRF_LOG_INFO("Baud rate changed to index %d = %d", con_comms.baud_index, baud_list[con_comms.baud_index]);
    
    con_comms.comms_state = COMMS_AUTOBAUD;

    nrf_libuarte_drv_uninit(p_libuarte);

    nrf_gpio_pin_clear(TX_ENABLE);    

    //ret = nrf_libuarte_drv_init(p_libuarte, p_config, evt_handler, context);
    if (ret != NRFX_SUCCESS)
    {    NRFX_LOG_INFO("Serial 1st init failure [%x]", ret);  }

    nrf_uarte_baudrate_set(p_libuarte->uarte, baud_list[con_comms.baud_index]);


    return(ret);
}

uint8_t makeCommsStatusFlashes(uint8_t idx, uint8_t commsStatus)
{   uint8_t c;

    switch (commsStatus)
    {   case COMMS_INIT:
            c = addLEDPattern(PROC_LED_WHITE, LED_SINGLE_FLASH, 8, 16, idx);  
            break;
        case COMMS_ERROR:
            c = addLEDPattern(PROC_LED_RED, LED_DOUBLE_FLASH, 16, 32, idx);    
            break;
        case COMMS_DISCONNECTED:
            c = addLEDPattern(PROC_LED_MAGENTA, LED_SINGLE_FLASH, 8, 16, idx);
            break;
        case COMMS_AUTOBAUD:
            c = addLEDPattern(PROC_LED_MAGENTA, LED_DOUBLE_FLASH, 16, 32, idx);
            break;
        case COMMS_VISIBLE:
            c = addLEDPattern(PROC_LED_YELLOW, LED_DOUBLE_FLASH, 16, 32, idx);
            break;
        case COMMS_ADOPTING: 
            c = addLEDPattern(PROC_LED_ORANGE, LED_DOUBLE_FLASH, 16, 32, idx);
            break;
        case COMMS_FOSTERED:
            c = addLEDPattern(PROC_LED_GREEN, LED_SINGLE_FLASH, 8, 16, idx);
            break;
        case COMMS_ONLINE:
            c = addLEDPattern(PROC_LED_GREEN, LED_DOUBLE_FLASH, 16, 32, idx);
            break;
        default:
            c = addLEDPattern(PROC_LED_RED, LED_SINGLE_FLASH, 8, 16, idx);
            break;
    }

    c = addLEDPattern(PROC_LED_BLUE, LED_SINGLE_FLASH, 8, 16, c);

    return(c);
}

