#ifndef __SYS_PLUGIN_H__
#define	__SYS_PLUGIN_H__

#include <stdint.h>

uint8_t calc_checksum(const uint8_t *data, uint8_t len);
void endian_transfer(uint8_t* data, int len);

#endif /* __SYS_PLUGIN_H__ */

