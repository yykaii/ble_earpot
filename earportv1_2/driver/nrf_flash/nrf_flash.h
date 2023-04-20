#ifndef __NRF_FLASH_H__
#define	__NRF_FLASH_H__

#include "stdint.h"

#define FLASH_START_ADDR         0x21000
#define FLASH_END_ADDR           0x2ffff
#define FLASH_AVAILABLE_SIZE     (FLASH_END_ADDR-FLASH_START_ADDR)
#define SECTOR_SIZE              (4*1024)
#define FLASH_SECTOR_NUM         (((FLASH_END_ADDR-FLASH_START_ADDR)+1)/(SECTOR_SIZE))
#define FLASH_DEFAULT_VAL        0xFFFFFFFF

typedef enum{
    FLASH_SUCCESS   = 0,
    ARG_ERR         = 1,
    FLASH_WRITE_ERR = 2,
    FLASH_ERASE_ERR = 3,
    FLASH_READ_ERR  = 4,
}FLASH_ERR_EU;


int nrf_flash_init(void);
FLASH_ERR_EU nrf_flash_read(uint32_t addr, void* para, int len);
FLASH_ERR_EU nrf_flash_write(uint32_t addr, void* para, int len);
FLASH_ERR_EU nrf_flash_erase(uint32_t addr);
void nrf_flash_read_test();
void nrf_flash_write_test();
void nrf_flash_erase_test();
void power_manage(void);

#endif /* __NRF_FLASH_H__ */

