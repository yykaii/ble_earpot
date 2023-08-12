#include <stdint.h>

#include "sys_plugin.h"

uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
	uint8_t i = 0;
	uint8_t checksum = data[0];

	for (i = 1; i < len; i++)
	{
		checksum ^= data[i];
	}

	return checksum;
}

void endian_transfer(uint8_t* data, int len)
{
    uint8_t tmp;
    if(len%2 == 0)
    {
        for(int i=0; i<(len>>1); i++)
        {
            tmp = data[2*i];
            data[2*i] = data[2*i+1];
            data[2*i+1] = tmp;
        }
    }
}
