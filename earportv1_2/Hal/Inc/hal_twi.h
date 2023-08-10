#ifndef __HAL_TWI_H__
#define __HAL_TWI_H__

#include <stdint.h>

int8_t nrf52_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t nrf52_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void twi_master_init(void);

#endif /*__HAL_TWI_H__*/







