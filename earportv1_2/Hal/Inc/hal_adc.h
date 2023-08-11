#ifndef __HAL_ADC_H__
#define	__HAL_ADC_H__

#include <stdint.h>

void saadc_init(void);
void saadc_uninit(void);
uint16_t get_adc_voltage(void);

#endif /* __HAL_ADC_H__ */
