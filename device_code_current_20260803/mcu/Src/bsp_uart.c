#include "bsp_uart.h"

#include "bsp_rs485.h"

#include <string.h>

/* UART 句柄必须是全局变量，HAL 中断/回调和其他模块会通过它访问串口。 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static uint32_t uart1_hal_mode = UART_MODE_TX_RX;
static uint32_t uart2_hal_mode = UART_MODE_TX_RX;
static uint32_t uart3_hal_mode = UART_MODE_TX_RX;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t dma_rx_buffer[BSP_UART_DMA_RX_BUFFER_SIZE];
    uint8_t rx_buffer[BSP_UART_RX_BUFFER_SIZE];
    volatile uint16_t dma_last_pos;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint8_t rx_overflow;
    volatile uint8_t dma_started;
} BSP_UART_DMA_Context;

static BSP_UART_DMA_Context uart1_dma_ctx = {.huart = &huart1};
static BSP_UART_DMA_Context uart2_dma_ctx = {.huart = &huart2};
static BSP_UART_DMA_Context uart3_dma_ctx = {.huart = &huart3};

static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static uint32_t uart_mode_to_hal(BSP_UART_Mode mode);
static BSP_UART_DMA_Context *uart_context_from_handle(UART_HandleTypeDef *huart);
static void uart_rx_push(BSP_UART_DMA_Context *ctx, uint8_t value);
static uint8_t uart_rx_pop(BSP_UART_DMA_Context *ctx, uint8_t *value);
static uint16_t uart_rx_available(const BSP_UART_DMA_Context *ctx);
static void uart_dma_rx_copy(BSP_UART_DMA_Context *ctx, uint16_t pos);
static uint16_t uart_dma_current_pos(const BSP_UART_DMA_Context *ctx);
static void uart_dma_rx_sync(BSP_UART_DMA_Context *ctx);
static uint8_t uart_timeout_elapsed(uint32_t start, uint32_t timeout_ms);
static HAL_StatusTypeDef uart_wait_tx_complete(UART_HandleTypeDef *huart, uint32_t timeout_ms);

void BSP_UART_Init(void)
{
    /* 初始化两个模板常用串口。 */
    BSP_UART_InitMode(BSP_UART_MODE_TX_RX, BSP_UART_MODE_TX_RX);
}

void BSP_UART_InitMode(BSP_UART_Mode uart1_mode, BSP_UART_Mode uart3_mode)
{
    uart1_hal_mode = uart_mode_to_hal(uart1_mode);
    uart3_hal_mode = uart_mode_to_hal(uart3_mode);
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
}

void BSP_UART2_InitMode(BSP_UART_Mode uart2_mode)
{
    uart2_hal_mode = uart_mode_to_hal(uart2_mode);
    MX_USART2_UART_Init();
}

HAL_StatusTypeDef BSP_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((huart == 0) || (data == 0)) {
        return HAL_ERROR;
    }

    if (size == 0U) {
        return HAL_OK;
    }

    status = BSP_UART_TransmitDMA(huart, data, size);
    if (status != HAL_OK) {
        return status;
    }

    return uart_wait_tx_complete(huart, timeout_ms);
}

HAL_StatusTypeDef BSP_UART_TransmitDMA(UART_HandleTypeDef *huart,
                                       const uint8_t *data,
                                       uint16_t size)
{
    if ((huart == 0) || (data == 0)) {
        return HAL_ERROR;
    }

    if (size == 0U) {
        return HAL_OK;
    }

    return HAL_UART_Transmit_DMA(huart, (uint8_t *)data, size);
}

HAL_StatusTypeDef BSP_UART_Receive(UART_HandleTypeDef *huart,
                                   uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout_ms)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);
    uint32_t start = HAL_GetTick();
    uint16_t read_len = 0U;

    if ((huart == 0) || (data == 0)) {
        return HAL_ERROR;
    }

    if (size == 0U) {
        return HAL_OK;
    }

    if ((ctx == 0) || (ctx->dma_started == 0U)) {
        return HAL_UART_Receive(huart, data, size, timeout_ms);
    }

    while (read_len < size) {
        uart_dma_rx_sync(ctx);
        if (uart_rx_pop(ctx, &data[read_len]) != 0U) {
            read_len++;
        } else if (uart_timeout_elapsed(start, timeout_ms) != 0U) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef BSP_UART_StartReceiveDMA(UART_HandleTypeDef *huart)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);
    HAL_StatusTypeDef status;

    if (ctx == 0) {
        return HAL_ERROR;
    }

    if (ctx->dma_started != 0U) {
        return HAL_OK;
    }

    ctx->dma_last_pos = 0U;
    ctx->rx_head = 0U;
    ctx->rx_tail = 0U;
    ctx->rx_overflow = 0U;

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, ctx->dma_rx_buffer, BSP_UART_DMA_RX_BUFFER_SIZE);
    if (status == HAL_OK) {
        ctx->dma_started = 1U;
        if (huart->hdmarx != 0) {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    return status;
}

HAL_StatusTypeDef BSP_UART1_StartReceiveDMA(void)
{
    return BSP_UART_StartReceiveDMA(&huart1);
}

HAL_StatusTypeDef BSP_UART2_StartReceiveDMA(void)
{
    return BSP_UART_StartReceiveDMA(&huart2);
}

HAL_StatusTypeDef BSP_UART3_StartReceiveDMA(void)
{
    return BSP_UART_StartReceiveDMA(&huart3);
}

uint16_t BSP_UART_ReadAvailable(UART_HandleTypeDef *huart)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);

    if (ctx == 0) {
        return 0U;
    }

    uart_dma_rx_sync(ctx);
    return uart_rx_available(ctx);
}

uint16_t BSP_UART_ReadBuffered(UART_HandleTypeDef *huart, uint8_t *data, uint16_t max_size)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);
    uint16_t read_len = 0U;

    if ((ctx == 0) || (data == 0)) {
        return 0U;
    }

    uart_dma_rx_sync(ctx);
    while ((read_len < max_size) && (uart_rx_pop(ctx, &data[read_len]) != 0U)) {
        read_len++;
    }

    return read_len;
}

void BSP_UART_FlushRx(UART_HandleTypeDef *huart)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);

    if (ctx != 0) {
        if ((ctx->dma_started != 0U) && (ctx->huart->hdmarx != 0)) {
            ctx->dma_last_pos = uart_dma_current_pos(ctx);
        }
        ctx->rx_tail = ctx->rx_head;
        ctx->rx_overflow = 0U;
    }
}

HAL_StatusTypeDef BSP_UART1_Print(const char *text)
{
    /* strlen 只适合发送以 '\0' 结尾的文本，不适合发送任意二进制数据。 */
    return BSP_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

HAL_StatusTypeDef BSP_UART1_PrintDMA(const char *text)
{
    return BSP_UART_TransmitDMA(&huart1, (uint8_t *)text, (uint16_t)strlen(text));
}

HAL_StatusTypeDef BSP_UART1_Read(uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    return BSP_UART_Receive(&huart1, data, size, timeout_ms);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);

    if (ctx != 0) {
        uart_dma_rx_copy(ctx, Size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
        }
        BSP_RS485_DE_Write(false);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_DMA_Context *ctx = uart_context_from_handle(huart);

    if ((ctx != 0) && (ctx->dma_started != 0U)) {
        ctx->dma_started = 0U;
        (void)HAL_UART_AbortReceive(huart);
        (void)BSP_UART_StartReceiveDMA(huart);
    }
}

static void MX_USART1_UART_Init(void)
{
    /* USART1: 115200-8-N-1，用作调试打印或 AT 模块示例。 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = BSP_UART_DEFAULT_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = uart1_hal_mode;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    /* HAL_UART_Init 会回调 HAL_UART_MspInit 配置 PA9/PA10。 */
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    /* USART2: 115200-8-N-1，默认 PD5=TX，PD6=RX。 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = BSP_UART_DEFAULT_BAUDRATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = uart2_hal_mode;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART3_UART_Init(void)
{
    /* USART3: 115200-8-N-1，当前模板用于 RS485/Modbus 示例。 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = BSP_UART_DEFAULT_BAUDRATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = uart3_hal_mode;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    /* HAL_UART_Init 会回调 HAL_UART_MspInit 配置 PB10/PB11。 */
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

static uint32_t uart_mode_to_hal(BSP_UART_Mode mode)
{
    switch (mode) {
    case BSP_UART_MODE_TX_ONLY:
        return UART_MODE_TX;
    case BSP_UART_MODE_RX_ONLY:
        return UART_MODE_RX;
    case BSP_UART_MODE_TX_RX:
    default:
        return UART_MODE_TX_RX;
    }
}

static BSP_UART_DMA_Context *uart_context_from_handle(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 || ((huart != 0) && (huart->Instance == USART1))) {
        return &uart1_dma_ctx;
    }

    if (huart == &huart2 || ((huart != 0) && (huart->Instance == USART2))) {
        return &uart2_dma_ctx;
    }

    if (huart == &huart3 || ((huart != 0) && (huart->Instance == USART3))) {
        return &uart3_dma_ctx;
    }

    return 0;
}

static void uart_rx_push(BSP_UART_DMA_Context *ctx, uint8_t value)
{
    uint16_t next_head = (uint16_t)(ctx->rx_head + 1U);

    if (next_head >= BSP_UART_RX_BUFFER_SIZE) {
        next_head = 0U;
    }

    if (next_head == ctx->rx_tail) {
        ctx->rx_overflow = 1U;
        return;
    }

    ctx->rx_buffer[ctx->rx_head] = value;
    ctx->rx_head = next_head;
}

static uint8_t uart_rx_pop(BSP_UART_DMA_Context *ctx, uint8_t *value)
{
    uint16_t next_tail;

    if (ctx->rx_tail == ctx->rx_head) {
        return 0U;
    }

    *value = ctx->rx_buffer[ctx->rx_tail];
    next_tail = (uint16_t)(ctx->rx_tail + 1U);
    if (next_tail >= BSP_UART_RX_BUFFER_SIZE) {
        next_tail = 0U;
    }
    ctx->rx_tail = next_tail;

    return 1U;
}

static uint16_t uart_rx_available(const BSP_UART_DMA_Context *ctx)
{
    uint16_t head = ctx->rx_head;
    uint16_t tail = ctx->rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BSP_UART_RX_BUFFER_SIZE - tail + head);
}

static void uart_dma_rx_copy(BSP_UART_DMA_Context *ctx, uint16_t pos)
{
    uint16_t current_pos = pos;
    uint16_t last_pos = ctx->dma_last_pos;

    if (current_pos > BSP_UART_DMA_RX_BUFFER_SIZE) {
        return;
    }

    if (current_pos == BSP_UART_DMA_RX_BUFFER_SIZE) {
        current_pos = 0U;
    }

    if (current_pos == last_pos) {
        return;
    }

    if (current_pos > last_pos) {
        for (uint16_t i = last_pos; i < current_pos; i++) {
            uart_rx_push(ctx, ctx->dma_rx_buffer[i]);
        }
    } else {
        for (uint16_t i = last_pos; i < BSP_UART_DMA_RX_BUFFER_SIZE; i++) {
            uart_rx_push(ctx, ctx->dma_rx_buffer[i]);
        }
        for (uint16_t i = 0U; i < current_pos; i++) {
            uart_rx_push(ctx, ctx->dma_rx_buffer[i]);
        }
    }

    ctx->dma_last_pos = current_pos;
}

static uint16_t uart_dma_current_pos(const BSP_UART_DMA_Context *ctx)
{
    uint16_t pos = (uint16_t)(BSP_UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx));

    if (pos >= BSP_UART_DMA_RX_BUFFER_SIZE) {
        pos = 0U;
    }

    return pos;
}

static void uart_dma_rx_sync(BSP_UART_DMA_Context *ctx)
{
    if ((ctx->dma_started != 0U) && (ctx->huart->hdmarx != 0)) {
        uart_dma_rx_copy(ctx, uart_dma_current_pos(ctx));
    }
}

static uint8_t uart_timeout_elapsed(uint32_t start, uint32_t timeout_ms)
{
    if (timeout_ms == HAL_MAX_DELAY) {
        return 0U;
    }

    return ((HAL_GetTick() - start) >= timeout_ms) ? 1U : 0U;
}

static HAL_StatusTypeDef uart_wait_tx_complete(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (huart->gState != HAL_UART_STATE_READY) {
        if (uart_timeout_elapsed(start, timeout_ms) != 0U) {
            (void)HAL_UART_AbortTransmit(huart);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}
