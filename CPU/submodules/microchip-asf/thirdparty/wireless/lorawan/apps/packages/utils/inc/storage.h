/*  _        _   _ _ _ _
   / \   ___| |_(_) (_) |_ _   _
  / _ \ / __| __| | | | __| | | |
 / ___ \ (__| |_| | | | |_| |_| |
/_/   \_\___|\__|_|_|_|\__|\__, |
                           |___/
    (C)2017 Actility
License: Revised BSD License, see LICENSE.TXT file include in the project
Description: Flash API

*/
#ifndef __EFLASH_H__
#define __EFLASH_H__

#include <asf.h>

#define CRC8_POLYNOMIAL                         (0x31)

#define EFLASH_BLANK_BYTE                       (0xFF)

#define STORAGE_CRITICAL_ENTER()                (__disable_irq())   //TODO: it needs to use another method for enter and exit of critical section. It must be RTOS compatible.
#define STORAGE_CRITICAL_EXIT()                 (__enable_irq())    //TODO: it needs to use another method for enter and exit of critical section. It must be RTOS compatible.

#define STORAGE_RESOURCE_TAKE()     {if(storage_isbusy()) {return STR_BUSY;} else {storage_setbusy(true);}}
#define STORAGE_RESOURCE_GIVE()     {storage_setbusy(false);}

typedef enum storage_status_s {
    STR_OK = 0,
    STR_BAD_LEN,
    STR_BAD_OFFSET,
    STR_HAL_ERR,
    STR_INCONSISTENCY,
    STR_BADALIGN,
    STR_VER_ERR,
    STR_BUSY,
} storage_status_t;


//----

storage_status_t storage_validate_img(uint32_t size, uint8_t expCRC);
void storage_get_own_version(uint32_t *fw, uint32_t *hw);
storage_status_t storage_init(void);
storage_status_t storage_write_block(uint32_t idx, uint32_t M, uint8_t* data, uint32_t len, uint32_t fsize);
storage_status_t storage_read_block(uint32_t idx, uint32_t M, uint8_t* data, uint32_t len, uint32_t fsize);
storage_status_t storage_erase(void); // not to implement
storage_status_t storage_check_blank(void); // not to implement
storage_status_t storage_write_appinfo(uint32_t len, uint8_t crc);
uint8_t storage_calc_crc8(uint8_t *data, uint8_t length, uint8_t initCRC);
bool storage_isbusy(void); // not to implement
void storage_setbusy(bool busy); // not to implement
void print_frame(uint8_t *data, uint32_t len);
uint8_t storage_fragcheck(uint32_t idx, uint32_t M);
void frag_base_set(uint32_t base_idx);
void frag_redund_set(uint32_t redund_idx, uint32_t offset);
void frag_init(void);

#endif /* __EFLASH_H__ */

