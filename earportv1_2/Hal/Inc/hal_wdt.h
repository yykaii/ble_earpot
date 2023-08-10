#ifndef __HAL_WDT_H__
#define __HAL_WDT_H__

#include <stdint.h>

void hal_wdt_init(void);
void hal_wdt_feed(void);

#endif /*__HAL_WDT_H__*/
