#ifndef MAIN_H
#define MAIN_H

/*
 * 工程公共主头文件。
 *
 * 所有自写模块通过包含 main.h 获得 STM32 HAL 类型、寄存器定义
 * 和工程级系统接口声明。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "bsp_system.h"

#ifdef __cplusplus
}
#endif

#endif
