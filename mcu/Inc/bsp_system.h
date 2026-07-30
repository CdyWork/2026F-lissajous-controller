#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

/*
 * 系统级 BSP 接口。
 *
 * 这里统一声明工程级系统时钟配置和错误处理接口。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 系统时钟配置：25MHz HSE 经 PLL 提升到 168MHz。 */
void SystemClock_Config(void);

/* 统一错误处理入口：初始化失败时会进入该函数并闪烁 LED1。 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
