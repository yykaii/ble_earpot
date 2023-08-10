#include <stdint.h>

#include "hal_wdt.h"

#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"

/*:ms*/
#define  WDT_RELOAD_TIME 65000

nrf_drv_wdt_channel_id m_channel_id;


void wdt_event_handler(void){

}

void hal_wdt_init(void) {
    int err_code;
	nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    config.reload_value = WDT_RELOAD_TIME;
	err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
	APP_ERROR_CHECK(err_code);
	nrf_drv_wdt_enable();
}

void hal_wdt_feed(void) {
    nrfx_wdt_channel_feed(m_channel_id);
}
