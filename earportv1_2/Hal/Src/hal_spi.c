#include <stdint.h>
#include <string.h>

#include "sdk_config.h"
#include "app_config.h"

#include "nrf_drv_spi.h"
#include "hal_spi.h"


#if BMI160_INTERFACE_SPI == 1

#define SPI_INSTANCE 0												   /**< SPI instance index. */
static volatile bool spi_xfer_done;									   // SPI数据传输完成标志
static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
static uint8_t spi_tx_buf[256]; /**< TX buffer. */
static uint8_t spi_rx_buf[256]; /**< RX buffer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
					   void *p_context)
{
	spi_xfer_done = true;
}

#define SPI_SS_PIN 0
#define SPI_MISO_PIN 12
// #define SPI_MOSI_PIN 4
#define SPI_MOSI_PIN 14 // csy_0308  seconde version PCB
#define SPI_SCK_PIN 1


void spi_enable(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin = SPI_SCK_PIN;
	// csy_0316 spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
	APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}

void spi_disable(void)
{
	nrf_drv_spi_uninit(&m_spi);
}

int8_t nrf52_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;
	spi_xfer_done = false;
	// printf("%s\r\n",__FUNCTION__);

	spi_tx_buf[0] = reg_addr;

	ret = nrf_drv_spi_transfer(&m_spi, spi_tx_buf, len + 1, spi_rx_buf, len + 1);
	// APP_ERROR_CHECK();
	while (!spi_xfer_done)
		;
	// nrf_delay_ms(1);
	if (reg_addr == 0x80)
	{
		// printf("%x,%x,%x,%x\r\n",spi_rx_buf[0],spi_rx_buf[1],spi_rx_buf[2],spi_rx_buf[3]);
	}
	memcpy(data, &spi_rx_buf[1], len);

	return ret;
}

int8_t nrf52_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;
	// printf("%s\r\n",__FUNCTION__);

	spi_xfer_done = false;

	spi_tx_buf[0] = reg_addr;
	memcpy(&spi_tx_buf[1], data, len);
	ret = nrf_drv_spi_transfer(&m_spi, spi_tx_buf, len + 1, spi_rx_buf, len + 1);
	while (!spi_xfer_done)
		;
	// nrf_delay_ms(1);
	return ret;
}
#endif