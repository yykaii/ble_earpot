#ifndef __HAL_LED_H__
#define	__HAL_LED_H__

#include <stdint.h>

enum {
    LED_BLUE,
    LED_MAX,
};

void hal_led_init(void);
void hal_led_on(uint8_t led_idx);
void hal_led_off(uint8_t led_idx);

#endif /* __HAL_LED_H__ */

