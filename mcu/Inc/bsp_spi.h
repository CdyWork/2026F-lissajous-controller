#ifndef BSP_SPI_H
#define BSP_SPI_H

/*
 * SPI1 通用传输层。
 *
 * 本文件只负责 SPI 协议收发，不关心具体外设命令。
 * 例如 SPI Flash 的 0x9F 读 ID 命令放在 bsp_spi_flash.c 中。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp_board.h"
#include "main.h"

#define BSP_SPI1_DEFAULT_MAX_HZ 1000000U

/* SPI 方向模式选择，默认使用全双工。 */
typedef enum {
    BSP_SPI_MODE_FULL_DUPLEX = 0, /* 2 线全双工：MOSI/MISO 同时工作。 */
    BSP_SPI_MODE_RX_ONLY,         /* 2 线只接收。 */
    BSP_SPI_MODE_HALF_DUPLEX      /* 1 线半双工。 */
} BSP_SPI_Mode;

/* SPI1 句柄：默认 PA5=SCK，PA6=MISO，PA7=MOSI，软件 NSS。 */
extern SPI_HandleTypeDef hspi1;

/* 初始化 SPI1 软件片选 GPIO，默认释放片选。 */
void BSP_SPI1_CS_Init(void);

/* 控制 SPI1 片选：active=true 拉低选中，active=false 拉高释放。 */
void BSP_SPI1_CS_Write(bool active);

/* 初始化 SPI1 为主机模式、8 位数据、Mode0、软件 NSS，默认全双工。 */
void BSP_SPI1_Init(void);

/* 按指定方向模式初始化 SPI1。 */
void BSP_SPI1_InitMode(BSP_SPI_Mode mode);

/* 按指定方向模式和最高 SCK 频率初始化 SPI1，max_hz=0 时使用默认值。 */
void BSP_SPI1_InitModeSpeed(BSP_SPI_Mode mode, uint32_t max_hz);

/* 返回当前 SPI1 SCK 频率估算值。 */
uint32_t BSP_SPI1_GetClockHz(void);

/* SPI1 发送数据。 */
HAL_StatusTypeDef BSP_SPI1_Transmit(const uint8_t *tx_data, uint16_t size, uint32_t timeout_ms);

/* SPI1 接收数据。主机接收时 HAL 会自动发送 dummy clock。 */
HAL_StatusTypeDef BSP_SPI1_Receive(uint8_t *rx_data, uint16_t size, uint32_t timeout_ms);

/* SPI1 全双工收发，发送和接收长度相同。 */
HAL_StatusTypeDef BSP_SPI1_TransmitReceive(const uint8_t *tx_data,
                                           uint8_t *rx_data,
                                           uint16_t size,
                                           uint32_t timeout_ms);

/* 常见 SPI 设备操作：片选拉低后先写命令，再连续读取响应。 */
HAL_StatusTypeDef BSP_SPI1_WriteThenRead(const uint8_t *tx_data,
                                         uint16_t tx_size,
                                         uint8_t *rx_data,
                                         uint16_t rx_size,
                                         uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
