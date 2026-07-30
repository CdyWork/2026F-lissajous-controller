#include "bsp_rs485.h"

void BSP_RS485_DE_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = BSP_RS485_DE_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BSP_RS485_DE_GPIO_Port, &gpio);

    BSP_RS485_DE_Write(false);
}

void BSP_RS485_DE_Write(bool tx_enable)
{
    /* DE=1 发送，DE=0 接收。 */
    HAL_GPIO_WritePin(BSP_RS485_DE_GPIO_Port, BSP_RS485_DE_Pin, tx_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* RS485 半双工发送函数。 */
HAL_StatusTypeDef BSP_RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    HAL_StatusTypeDef ret;

    /* 发送前切换到发送方向。 */
    BSP_RS485_DE_Write(true);
    ret = BSP_UART_TransmitDMA(&huart3, data, len);
    if (ret != HAL_OK) {
        BSP_RS485_DE_Write(false);
        return ret;
    }

    while (huart3.gState != HAL_UART_STATE_READY) {
        if ((timeout_ms != HAL_MAX_DELAY) && ((HAL_GetTick() - start) >= timeout_ms)) {
            (void)HAL_UART_AbortTransmit(&huart3);
            BSP_RS485_DE_Write(false);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef BSP_RS485_SendDMA(const uint8_t *data, uint16_t len)
{
    BSP_RS485_DE_Write(true);

    if (BSP_UART_TransmitDMA(&huart3, data, len) != HAL_OK) {
        BSP_RS485_DE_Write(false);
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BSP_RS485_Receive(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    /* 接收前切换到接收方向。 */
    BSP_RS485_DE_Write(false);
    return BSP_UART_Receive(&huart3, data, len, timeout_ms);
}

HAL_StatusTypeDef BSP_RS485_StartReceiveDMA(void)
{
    BSP_RS485_DE_Write(false);
    return BSP_UART3_StartReceiveDMA();
}

uint16_t BSP_RS485_ReadBuffered(uint8_t *data, uint16_t max_len)
{
    return BSP_UART_ReadBuffered(&huart3, data, max_len);
}

void BSP_RS485_FlushRx(void)
{
    BSP_UART_FlushRx(&huart3);
}
