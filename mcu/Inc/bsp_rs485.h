#ifndef BSP_RS485_H
#define BSP_RS485_H

/* RS485 半双工收发接口：使用 USART3 + DE 方向控制脚。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp_board.h"
#include "bsp_uart.h"

/* 初始化 RS485 DE 方向控制脚，默认接收。 */
void BSP_RS485_DE_Init(void);

/* 设置 RS485 方向：tx_enable=true 发送，false 接收。 */
void BSP_RS485_DE_Write(bool tx_enable);

/* 发送一帧 RS485 数据：发送前 DE=1，等待 TC 完成后 DE=0。 */
HAL_StatusTypeDef BSP_RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* DMA 非阻塞发送一帧 RS485 数据，发送完成回调中自动切回接收方向。 */
HAL_StatusTypeDef BSP_RS485_SendDMA(const uint8_t *data, uint16_t len);

/* 接收一帧 RS485 数据：接收前确保 DE=0。 */
HAL_StatusTypeDef BSP_RS485_Receive(uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* 启动/读取/清空 RS485 DMA 接收缓存。 */
HAL_StatusTypeDef BSP_RS485_StartReceiveDMA(void);
uint16_t BSP_RS485_ReadBuffered(uint8_t *data, uint16_t max_len);
void BSP_RS485_FlushRx(void);

#ifdef __cplusplus
}
#endif

#endif
