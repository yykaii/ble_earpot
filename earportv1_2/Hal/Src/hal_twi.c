#include <stdint.h>

#include "hal_twi.h"
#include "app_config.h"

#if (BMI160_INTERFACE_I2C == 1)
// TWI
#define TWI_INSTANCE_ID 0

static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

int8_t nrf52_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret = 0;
	uint8_t data_out[2];

	data_out[0] = reg_addr;
	memcpy(&data_out[1], data, len);

	ret = nrf_drv_twi_tx(&m_twi, dev_addr, data_out, len + 1, false);

	return ret;
}

int8_t nrf52_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;

	ret = nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, true);
	if (ret != NRF_SUCCESS)
	{
		return ret;
	}
	nrf_delay_ms(2);

	ret = nrf_drv_twi_rx(&m_twi, dev_addr, data, len);
	if (ret != NRF_SUCCESS)
	{
		return ret;
	}
	nrf_delay_ms(2);

	return ret;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event->type)
	{
	case NRF_DRV_TWI_EVT_DONE:
		m_xfer_done = true;
		break;
	default:
		break;
	}
}

void twi_master_init(void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config = {
		.scl = TWI_SCL_M,
		.sda = TWI_SDA_M,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false};

	err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

#endif