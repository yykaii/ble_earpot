#include <stdint.h>
#include "nrf_drv_saadc.h"
#include "hal_adc.h"


void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
}

void saadc_init(void)
{
	ret_code_t err_code;
	nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);
}

void saadc_uninit(void) 
{
    nrfx_saadc_uninit();
}

uint16_t get_adc_voltage(void)
{
	nrf_saadc_value_t saadc_val;
	nrf_drv_saadc_sample_convert(0, &saadc_val);
	return (6 * 3 * saadc_val * 0.6 / 1024 * 1000 * 1.047); // coef = 1.047, test value
}
