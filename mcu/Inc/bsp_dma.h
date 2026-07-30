#ifndef BSP_DMA_H
#define BSP_DMA_H

/*
 * DMA 基础接口。
 *
 * 当前模板预留：
 * - ADC1: DMA2 Stream0 Channel0
 * - DAC1 CH1: DMA1 Stream5 Channel7
 * - USART1_RX: DMA2 Stream2 Channel4
 * - USART1_TX: DMA2 Stream7 Channel4
 * - USART3_RX: DMA1 Stream1 Channel4
 * - USART3_TX: DMA1 Stream3 Channel4
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

void BSP_DMA_Init(void);
void BSP_DMA_DeInitAll(void);

#ifdef __cplusplus
}
#endif

#endif
