
#ifndef DF_ROUTINES_H__
#define DF_ROUTINES_H__

#include <stdint.h>

#define IS_IN_RANGE(num, low, high) ((num >= low) && (num <= high))


void do_sys_reset(void);
uint8_t inc_is_error(uint8_t *cnt, uint8_t threshold);
void wrapping_inc(uint8_t *val, uint32_t lower, uint32_t upper);
uint16_t buf2int(char *buf);
void int2buf(char *buf, uint16_t val);
uint32_t buf2long(char *buf);
void long2buf(char *buf, uint32_t val);
char isInCharArray(uint8_t Val, uint8_t *Array, uint8_t ArySize);
uint8_t slotFromObject(void * thisAddr, void * baseAddr, uint32_t size);

/**
 * @brief A general purpose stub function for API calls that need an unused fn_handler
 */
void stub(void);



#endif // DF_ROUTINES_H__
