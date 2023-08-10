#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdint.h>

void spi_enable(void);
void spi_disable(void);
int8_t nrf52_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t nrf52_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif /*__HAL_SPI_H__*/
