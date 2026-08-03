#include "bsp_spi.h"

/* SPI1 全局句柄。 */
SPI_HandleTypeDef hspi1;

static uint32_t spi_mode_to_hal(BSP_SPI_Mode mode);
static uint32_t spi_prescaler_for_max_hz(uint32_t pclk_hz, uint32_t max_hz);
static uint32_t spi_prescaler_divisor(uint32_t prescaler);

void BSP_SPI1_CS_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = BSP_SPI1_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BSP_SPI1_CS_GPIO_Port, &gpio);

    BSP_SPI1_CS_Write(false);
}

void BSP_SPI1_CS_Write(bool active)
{
    /* SPI 片选通常低有效：active=true 时拉低 CS。 */
    HAL_GPIO_WritePin(BSP_SPI1_CS_GPIO_Port, BSP_SPI1_CS_Pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BSP_SPI1_Init(void)
{
    BSP_SPI1_CS_Init();
    BSP_SPI1_InitMode(BSP_SPI_MODE_FULL_DUPLEX);
}

void BSP_SPI1_InitMode(BSP_SPI_Mode mode)
{
    BSP_SPI1_InitModeSpeed(mode, BSP_SPI1_DEFAULT_MAX_HZ);
}

void BSP_SPI1_InitModeSpeed(BSP_SPI_Mode mode, uint32_t max_hz)
{
    uint32_t pclk_hz = HAL_RCC_GetPCLK2Freq();

    if (max_hz == 0U) {
        max_hz = BSP_SPI1_DEFAULT_MAX_HZ;
    }

    /* SPI1 主机模式，Mode0，软件 NSS，8 位数据。 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = spi_mode_to_hal(mode);
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = spi_prescaler_for_max_hz(pclk_hz, max_hz);
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    /* HAL_SPI_Init 会回调 HAL_SPI_MspInit 配置 PA5/PA6/PA7。 */
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

uint32_t BSP_SPI1_GetClockHz(void)
{
    uint32_t divisor = spi_prescaler_divisor(hspi1.Init.BaudRatePrescaler);

    if (divisor == 0U) {
        return 0U;
    }
    return HAL_RCC_GetPCLK2Freq() / divisor;
}

HAL_StatusTypeDef BSP_SPI1_Transmit(const uint8_t *tx_data, uint16_t size, uint32_t timeout_ms)
{
    /* 仅发送，不主动控制片选；片选由上层按具体设备协议控制。 */
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)tx_data, size, timeout_ms);
}

HAL_StatusTypeDef BSP_SPI1_Receive(uint8_t *rx_data, uint16_t size, uint32_t timeout_ms)
{
    /* 仅接收；SPI 主机接收时仍需要输出时钟。 */
    return HAL_SPI_Receive(&hspi1, rx_data, size, timeout_ms);
}

HAL_StatusTypeDef BSP_SPI1_TransmitReceive(const uint8_t *tx_data,
                                           uint8_t *rx_data,
                                           uint16_t size,
                                           uint32_t timeout_ms)
{
    /* 全双工收发，常用于寄存器读写或连续交换。 */
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_data, rx_data, size, timeout_ms);
}

HAL_StatusTypeDef BSP_SPI1_WriteThenRead(const uint8_t *tx_data,
                                         uint16_t tx_size,
                                         uint8_t *rx_data,
                                         uint16_t rx_size,
                                         uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;

    /* 常见 SPI 外设读操作：CS 拉低 -> 写命令/地址 -> 读数据 -> CS 拉高。 */
    BSP_SPI1_CS_Write(true);
    ret = BSP_SPI1_Transmit(tx_data, tx_size, timeout_ms);
    if (ret == HAL_OK) {
        ret = BSP_SPI1_Receive(rx_data, rx_size, timeout_ms);
    }
    BSP_SPI1_CS_Write(false);

    return ret;
}

static uint32_t spi_mode_to_hal(BSP_SPI_Mode mode)
{
    switch (mode) {
    case BSP_SPI_MODE_RX_ONLY:
        return SPI_DIRECTION_2LINES_RXONLY;
    case BSP_SPI_MODE_HALF_DUPLEX:
        return SPI_DIRECTION_1LINE;
    case BSP_SPI_MODE_FULL_DUPLEX:
    default:
        return SPI_DIRECTION_2LINES;
    }
}

static uint32_t spi_prescaler_for_max_hz(uint32_t pclk_hz, uint32_t max_hz)
{
    static const uint32_t prescalers[] = {
        SPI_BAUDRATEPRESCALER_2,
        SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,
        SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,
        SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128,
        SPI_BAUDRATEPRESCALER_256
    };

    if (max_hz == 0U) {
        max_hz = BSP_SPI1_DEFAULT_MAX_HZ;
    }

    for (uint8_t i = 0U; i < (sizeof(prescalers) / sizeof(prescalers[0])); i++) {
        uint32_t divisor = spi_prescaler_divisor(prescalers[i]);

        if ((divisor != 0U) && ((pclk_hz / divisor) <= max_hz)) {
            return prescalers[i];
        }
    }

    return SPI_BAUDRATEPRESCALER_256;
}

static uint32_t spi_prescaler_divisor(uint32_t prescaler)
{
    switch (prescaler) {
    case SPI_BAUDRATEPRESCALER_2:
        return 2U;
    case SPI_BAUDRATEPRESCALER_4:
        return 4U;
    case SPI_BAUDRATEPRESCALER_8:
        return 8U;
    case SPI_BAUDRATEPRESCALER_16:
        return 16U;
    case SPI_BAUDRATEPRESCALER_32:
        return 32U;
    case SPI_BAUDRATEPRESCALER_64:
        return 64U;
    case SPI_BAUDRATEPRESCALER_128:
        return 128U;
    case SPI_BAUDRATEPRESCALER_256:
        return 256U;
    default:
        return 0U;
    }
}
