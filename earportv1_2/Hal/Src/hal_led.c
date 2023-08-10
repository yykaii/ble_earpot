#include "nrf_gpio.h"
#include "hal_led.h"
#include "app_config.h"

void hal_led_init(void) {
	nrf_gpio_cfg_output(GPIO_LED_BLUE);
}

void hal_led_on(uint8_t led_idx) {
	switch (led_idx) {
		case LED_BLUE:
			nrf_gpio_pin_set(GPIO_LED_BLUE);
			break;
		default:
			break;
	}
}

void hal_led_off(uint8_t led_idx) {
	switch (led_idx) {
		case LED_BLUE:
			nrf_gpio_pin_clear(GPIO_LED_BLUE);
			break;
		default:
			break;
	}
}

