/**
 * @file  system_fault.h
 * @brief Middleware system fault handling interface.
 */
#ifndef SYSTEM_FAULT_H
#define SYSTEM_FAULT_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYSTEM_FAULT_MESSAGE_LEN 64U

typedef enum {
    SYSTEM_FAULT_NONE = 0,
    SYSTEM_FAULT_UNKNOWN,
    SYSTEM_FAULT_INVALID_ARG,
    SYSTEM_FAULT_ALLOC_FAILED,
    SYSTEM_FAULT_STATE_ERROR,
    SYSTEM_FAULT_HARDWARE
} SYSTEM_FAULT_CODE;

typedef struct {
    SYSTEM_FAULT_CODE code;
    char message[SYSTEM_FAULT_MESSAGE_LEN];
} SYSTEM_FAULT_INFO;

BSP_STATUS SystemFault_Set(SYSTEM_FAULT_CODE code, const char *message);
BSP_STATUS SystemFault_Get(SYSTEM_FAULT_INFO *out);
SYSTEM_FAULT_CODE SystemFault_GetCode(void);
const char *SystemFault_GetMessage(void);
void SystemFault_Clear(void);

void SystemFault_Handler(SYSTEM_FAULT_CODE code, const char *message);
void SystemFault_Halt(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_FAULT_H */
