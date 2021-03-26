#ifndef DF_CONSOLE_COMMS_H__
#define DF_CONSOLE_COMMS_H__


#include "app_error.h"
#include "df_nrf_serial.h"


#define NUM_BAUD_RATES 5
#define BAUD_LIST { NRF_UART_BAUDRATE_4800, NRF_UART_BAUDRATE_9600, NRF_UART_BAUDRATE_19200, NRF_UART_BAUDRATE_38400, NRF_UART_BAUDRATE_115200 }

#define RX_BUF_SIZE     128
#define TX_BUF_SIZE     128

// #define DISCOVERY_DFLT_ADDR   0xFFFE  - This is defined in sdk_config.h
#define ALL_CALL_ADDR         0xFFFF  

#define DISCOVERY_TYPE        0xFE
#define ALL_CALL_TYPE         0xFF

#define DEV_TYPE_PUMP   0x30    // Pump Device Type
#define DEV_TYPE_COMP   0x31    // Component Device Type

#define FD_CMD_RESET 		 0x00	// Reset the KbdLCD Controller
#define FD_CMD_DESC 		 0x01    // Fetch description of KbdLCD Controller
#define FD_CMD_STAT 		 0x02    // Fetch KbdLCD Status
#define FD_CMD_WRATTR_BYTE 	 0x03    // Write Attribute byte
#define FD_CMD_RDATTR_BYTE 	 0x04    // Read Attribute byte
#define FD_CMD_WRATTR_BIT 	 0x05    // Write Attribute bits
#define FD_CMD_RDATTR_BIT 	 0x06    // Read Attribute bits
#define FD_CMD_WRSTAT_BYTE 	 0x07    // Write Status bytes
#define FD_CMD_RDSTAT_BYTE 	 0x08    // Read Status bytes
#define FD_CMD_WRSTAT_BIT 	 0x09    // Write Status bits
#define FD_CMD_RDSTAT_BIT 	 0x0A    // Read Status bits
#define FD_CMD_RDEVT 		 0x0B    // Read KbdLCD Event
#define FD_CMD_PRG_S 		 0x0C    // Program S-Record
#define FD_CMD_TIME 		 0x0D    // Set the time
#define FD_CMD_CLRAPP 		 0x0E    // Clear the application area and Reset controller to bootloader mode
#define FD_CMD_PING 		 0x0F    // Ping the KbdLCD Controller
#define FD_CMD_CS 		 0x10    // Fetch the KbdLCD Controller checksum
#define FD_CMD_LDAPP 		 0x11    // Load the new application and reset

#define FD_CMD_DISCOVER          0x40    // DDPC Device adoption 
#define FD_CMD_IDENT             0x41    // DDPC Device Identification
    #define IDENT_LED            0x01    // Ident Command Payload flag bit .. Flash the LED
    #define IDENT_RELAY          0x02    // Ident Command Payload flag bit .. Turn on the Relay for 5 Sec

#define FD_CMD_ACTION            0x42    // DDPC Action Command
    #define PUMP_CMD_STATUS 	 0x00    // *** These actions are chosen for compatibility with the the Gilbarco protocol ***
    #define PUMP_CMD_AUTHORISE 	 0x01    // NoResp (32)
    #define PUMP_CMD_GO_TO_DATA  0x02    // (60)NoResp=Unable (32),(60)Resp=1 MSN=D LSN=PumpID (5)
    #define PUMP_CMD_STOP 	 0x03	 // NoResp (32)
    #define PUMP_CMD_SEND_TRANS	 0x04	 // (60)Resp=TransactionDataMsg (5)
    #define PUMP_CMD_TOTALS	 0x05	 // (60)Resp=ShortTransDataMsg or TotalsDataMsg (5)
    #define PUMP_CMD_BUSY_TOTALS 0x15	 // Special Call for ShortTrans (while pump is in Busy mode)
    #define PUMP_CMD_PAID 	 0x0D	 // NoResp (32)
    #define PUMP_CMD_DATA_MSG 	 0xF0	 // NoResp (32)
    #define PUMP_CMD_EXCH_MSG 	 0xF1	 // Exchange data message MB Request, MB Response
    #define PUMP_CMD_EMERG_STOP  0xFC	 // NoResp (32)
    #define PUMP_CMD_ALL_LIGHTS_ON 0xFD	 // NoResp (32)
    #define PUMP_CMD_ALL_LIGHTS_OFF 0xFB // NoResp (32)



typedef enum {
    RX_IS_IDLE = 0,
    RX_GOT_SOH,
    RX_GOT_STUFF,
    RX_GOT_EOF
} rx_state_t;

typedef struct 
{   uint8_t baud_index;             // Index of the baud rate into BAUD_LIST
    uint8_t  discovery_holdoff;     // a (randomised) counter to skip address discovery requests
    uint16_t discovery_temp_addr;   // In discovery mode, holds the new temp address until a succesful response is send (and the real address can be changed to this one)
    rx_state_t rx_state;            // Status of the Comms receiver
    uint8_t rx_char_count;
    char *rx_buf;                   // Pointer to the Rx Buffer
    
    uint8_t tx_collision;          // Flag to show that there was a bus collision
    uint8_t err_count;
} rx_data_t;

extern char rx_buf[RX_BUF_SIZE];    
extern rx_data_t rx_data;    

#define COMMS_TH_ERR        6
#define COMMS_TH_NOT_SOH    50

#define RX_DATA_DEFAULT                         \
{   .baud_index = 1,                            \
    .discovery_holdoff = 0x04,                  \
    .discovery_temp_addr = DISCOVERY_DFLT_ADDR, \
    .rx_state = RX_IS_IDLE,                     \
    .rx_char_count = 0,                         \
    .rx_buf = (char *)&rx_buf,                  \
    .tx_collision = 0,                          \
    .err_count = 0,                             \
}

extern char msg_buf[RX_BUF_SIZE];    

typedef enum {
    MSG_INCOMPLETE = 0,
    MSG_COMPLETE,
    MSG_OK,
    MSG_BAD_TYPE,
    MSG_BAD_ADDR,
    MSG_BAD_CS,
    MSG_DROPPED
} msg_state_t;

typedef struct
{   msg_state_t msg_state;        // Status of the message received
    uint8_t msg_type;
    uint16_t msg_addr;
    char *msg_buf;                // Pointer to the Msg Buffer
    uint8_t msg_char_count;
}  msg_data_t;

#define MSG_DATA_DEFAULT        \
{   .msg_state = MSG_INCOMPLETE, \
    .msg_type = 0,               \
    .msg_addr = 0,               \
    .msg_buf = (char *)&msg_buf, \
    .msg_char_count = 0,         \
}


// *** Function prototypes ***
//ret_code_t ConsoleSerialPortInit(bool first_time);
ret_code_t ConsoleSerialPortInit(struct nrf_serial_s const * p_serial);
size_t ConsoleWrite(struct nrf_serial_s const * p_serial, char *buf, uint8_t count);
// void comms_clear(void);



#endif // DF_CONSOLE_COMMS_H__
