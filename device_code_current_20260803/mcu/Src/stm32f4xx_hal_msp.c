#include "main.h"

/*
 * HAL MSP 文件。
 *
 * MSP = MCU Support Package。HAL_UART_Init/HAL_SPI_Init/HAL_I2C_Init
 * 会回调这里的函数，用于打开外设时钟和配置引脚复用。
 */

#include "bsp_dma.h"

void HAL_MspInit(void)
{
    /* SYSCFG/PWR 是很多 HAL 外设初始化和系统配置会用到的基础时钟。 */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART1) {
        /* USART1: PA9=TX，PA10=RX，复用功能 AF7。 */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &gpio);

        hdma_usart1_rx.Instance = DMA2_Stream2;
        hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
            Error_Handler();
        }

        hdma_usart1_tx.Instance = DMA2_Stream7;
        hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);
        __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

        HAL_NVIC_SetPriority(USART1_IRQn, 6U, 0U);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART2) {
        /* USART2: PD5=TX，PD6=RX，复用功能 AF7。RX DMA 与 DAC DMA 冲突，模板默认不启用 USART2 RX DMA。 */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &gpio);

        hdma_usart2_tx.Instance = DMA1_Stream6;
        hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);

        HAL_NVIC_SetPriority(USART2_IRQn, 6U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    } else if (huart->Instance == USART3) {
        /* USART3: PB10=TX，PB11=RX，复用功能 AF7，当前用于 RS485 示例。 */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &gpio);

        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
            Error_Handler();
        }

        hdma_usart3_tx.Instance = DMA1_Stream3;
        hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);
        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

        HAL_NVIC_SetPriority(USART3_IRQn, 6U, 0U);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio = {0};

    if (hspi->Instance == SPI1) {
        /* SPI1: PA5=SCK，PA6=MISO，PA7=MOSI，复用功能 AF5。 */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_5;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &gpio);

        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else if (hspi->Instance == SPI2) {
        /* SPI2: PB13=SCK，PB15=MOSI，模板 SPI2 预留。 */
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    if (hi2c->Instance == I2C1) {
        /* I2C1: PB6=SCL，PB7=SDA，开漏复用 AF4。 */
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (hi2c->Instance == I2C2) {
        /* I2C2: PB10=SCL，PB11=SDA，开漏复用 AF4。 */
        __HAL_RCC_I2C2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc->Instance == ADC1) {
        /* ADC1_IN0/IN1: PA0/PA1，DMA2 Stream0 Channel0。 */
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);

        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
    }
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac_handle)
{
    GPIO_InitTypeDef gpio = {0};

    if (hdac_handle->Instance == DAC) {
        /* DAC_OUT1: PA4，DMA1 Stream5 Channel7。PA4 与 SPI1 CS 默认引脚冲突。 */
        __HAL_RCC_DAC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_4;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);

        hdma_dac1_ch1.Instance = DMA1_Stream5;
        hdma_dac1_ch1.Init.Channel = DMA_CHANNEL_7;
        hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
        hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_dac1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hdac_handle, DMA_Handle1, hdma_dac1_ch1);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
    } else if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}
