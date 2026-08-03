#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

/*
 * STM32F4 HAL 配置文件。
 *
 * HAL 官方头文件会包含本文件，用它决定启用哪些 HAL 模块、
 * HSE/HSI 等时钟常量、HAL tick 优先级、Cache/Prefetch 选项等。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 启用本工程会用到的 HAL 模块。未启用的模块不会展开对应声明。 */
#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED

/* CS07-F407 板载外部晶振为 25MHz。 */
#define HSE_VALUE 25000000U
#define HSE_STARTUP_TIMEOUT 100U
#define HSI_VALUE 16000000U
#define LSI_VALUE 32000U
#define LSE_VALUE 32768U
#define LSE_STARTUP_TIMEOUT 5000U
#define EXTERNAL_CLOCK_VALUE 12288000U

#define VDD_VALUE 3300U
/* HAL tick 中断优先级，数值越大优先级越低。 */
#define TICK_INT_PRIORITY 0x0FU
#define USE_RTOS 0U
/* F4 支持预取和 I/D cache，模板默认开启。 */
#define PREFETCH_ENABLE 1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE 1U

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal_tim.h"

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
