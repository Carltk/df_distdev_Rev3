
#include "nrf.h"
#include "df_routines.h"

void stub(void) {}

void do_sys_reset() {
    // setting up a system reset .. set a flag here 
    // we need to wait long enough that the message response goes out

    NVIC_SystemReset();
}

uint8_t inc_is_error(uint8_t *cnt, uint8_t threshold)
{   uint8_t ret = 0;
    
    *cnt += 1;
    if (*cnt > threshold) 
    {   
        ret = 1;
        *cnt = 0;
    }
    return(ret);
}

void wrapping_inc(uint8_t *val, uint32_t lower, uint32_t upper)
{   *val += 1;         
    if (*val >= upper) *val = lower;
}

uint16_t buf2int(char *buf)
{   return((buf[0] << 8) + buf[1]); }

void int2buf(char *buf, uint16_t val)
{   *buf = (val >> 8) & 0xFF;
    *(buf + 1) = val & 0xFF;
}

void long2buf(char *buf, uint32_t val)
{   *buf = (val >> 24) & 0xFF;
    *(buf + 1) = (val >> 16) & 0xFF;
    *(buf + 2) = (val >> 8) & 0xFF;
    *(buf + 3) = val & 0xFF;
}

char isInCharArray(uint8_t Val, uint8_t *Array, uint8_t ArySize)
{   uint8_t i;
    for (i=0;i<ArySize;i++)
    {   if (Array[i] == Val) {   return(1);    }    }
    return(0);
}

uint8_t slotFromObject(void * thisAddr, void * baseAddr, uint32_t size)
{   return((thisAddr - baseAddr)/size);     }