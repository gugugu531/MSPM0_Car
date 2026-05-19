/**
 * @file  bsp_time.h
 * @brief BSP 系统时间与阻塞延时服务。
 */
#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Time_Init(void);
void BSP_Time_TickInc(void);
uint32_t BSP_Time_GetMs(void);

void BSP_DelayUs(uint32_t us);
void BSP_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIME_H */
