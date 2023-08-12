#ifndef __VER_H__
#define __VER_H__

#include <stdint.h>

#define SOFT_VER 13

inline uint8_t get_soft_ver(void) {
    return (uint8_t)(SOFT_VER);
}


#endif /*__VER_H__*/

