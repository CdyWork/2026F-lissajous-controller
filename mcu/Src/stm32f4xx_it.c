#include "main.h"

#include "bsp_dma.h"
#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"

void xPortSysTickHandler(void);

void NMI_Handler(void) {}
void DebugMon_Handler(void) {}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();
}

void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_tx); }
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_dac1_ch1); }
void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_tx); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }
void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
