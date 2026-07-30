#include "bsp_dma.h"

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

void BSP_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void BSP_DMA_DeInitAll(void)
{
    (void)HAL_DMA_DeInit(&hdma_adc1);
    (void)HAL_DMA_DeInit(&hdma_dac1_ch1);
    (void)HAL_DMA_DeInit(&hdma_usart1_rx);
    (void)HAL_DMA_DeInit(&hdma_usart1_tx);
    (void)HAL_DMA_DeInit(&hdma_usart2_rx);
    (void)HAL_DMA_DeInit(&hdma_usart2_tx);
    (void)HAL_DMA_DeInit(&hdma_usart3_rx);
    (void)HAL_DMA_DeInit(&hdma_usart3_tx);
}
