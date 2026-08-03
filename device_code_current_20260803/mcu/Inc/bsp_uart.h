#ifndef BSP_UART_H
#define BSP_UART_H

/* UART 驱动接口。当前模板启用 USART1 和 USART3。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_UART_DMA_RX_BUFFER_SIZE 256U
#define BSP_UART_RX_BUFFER_SIZE     512U
#define BSP_UART_DEFAULT_BAUDRATE   115200U

/* UART 工作模式选择，最终会映射到 HAL 的 UART_MODE_xxx。 */
typedef enum {
    BSP_UART_MODE_TX_RX = 0, /* 同时允许发送和接收，模板默认模式。 */
    BSP_UART_MODE_TX_ONLY,   /* 只发送。 */
    BSP_UART_MODE_RX_ONLY    /* 只接收。 */
} BSP_UART_Mode;

/* USART1 句柄：默认 PA9=TX，PA10=RX，115200-8-N-1。 */
extern UART_HandleTypeDef huart1;

/* USART2 句柄：默认 PD5=TX，PD6=RX，115200-8-N-1。 */
extern UART_HandleTypeDef huart2;

/* USART3 句柄：默认 PB10=TX，PB11=RX，常用于 RS485 示例。 */
extern UART_HandleTypeDef huart3;

/* 初始化 USART1 和 USART3。引脚复用配置在 stm32f4xx_hal_msp.c，默认 TX_RX。 */
void BSP_UART_Init(void);

/* 按指定模式初始化 USART1 和 USART3。 */
void BSP_UART_InitMode(BSP_UART_Mode uart1_mode, BSP_UART_Mode uart3_mode);

/* 按指定模式初始化 USART2，默认引脚 PD5/PD6。模板不启用 USART2 RX DMA，避免与 DAC DMA1 Stream5 冲突。 */
void BSP_UART2_InitMode(BSP_UART_Mode uart2_mode);

/* 通用 UART 发送函数，可传入 huart1/huart3 或其他已初始化句柄。 */
HAL_StatusTypeDef BSP_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout_ms);

/* 通用 UART DMA 发送，非阻塞；完成状态通过 HAL_UART_TxCpltCallback 处理。 */
HAL_StatusTypeDef BSP_UART_TransmitDMA(UART_HandleTypeDef *huart,
                                       const uint8_t *data,
                                       uint16_t size);

/* 通用 UART 接收函数，可传入 huart1/huart3 或其他已初始化句柄。 */
HAL_StatusTypeDef BSP_UART_Receive(UART_HandleTypeDef *huart,
                                   uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout_ms);

/* 启动 ReceiveToIdle DMA 接收。模板默认在 App_Init() 中启动 USART1/USART3。 */
HAL_StatusTypeDef BSP_UART_StartReceiveDMA(UART_HandleTypeDef *huart);
HAL_StatusTypeDef BSP_UART1_StartReceiveDMA(void);
/* 仅在项目没有使用 DAC DMA1 Stream5 时再启用 USART2 RX DMA。 */
HAL_StatusTypeDef BSP_UART2_StartReceiveDMA(void);
HAL_StatusTypeDef BSP_UART3_StartReceiveDMA(void);

/* 查询/读取/清空 DMA 接收软件缓冲区。 */
uint16_t BSP_UART_ReadAvailable(UART_HandleTypeDef *huart);
uint16_t BSP_UART_ReadBuffered(UART_HandleTypeDef *huart, uint8_t *data, uint16_t max_size);
void BSP_UART_FlushRx(UART_HandleTypeDef *huart);

/* 通过 USART1 发送以 '\0' 结尾的字符串。 */
HAL_StatusTypeDef BSP_UART1_Print(const char *text);

/* 通过 USART1 使用 DMA 发送以 '\0' 结尾的字符串，非阻塞。 */
HAL_StatusTypeDef BSP_UART1_PrintDMA(const char *text);

/* 通过 USART1 接收指定长度的数据。 */
HAL_StatusTypeDef BSP_UART1_Read(uint8_t *data, uint16_t size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
