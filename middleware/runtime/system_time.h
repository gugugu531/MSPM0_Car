#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>

extern uint32_t tick;

static inline uint32_t System_GetTickMs(void){
    return tick;
}

#endif /* SYSTEM_TIME_H */
