#ifndef BSP_BOARD_H
#define BSP_BOARD_H

/*
 * 板级总头文件。
 *
 * 这里集中定义 CS07-F407 开发板上已经确认的固定引脚：
 * LCD、LED、蜂鸣器、板载按键、SPI 片选、RS485 方向控制脚。
 *
 * LED、按键、蜂鸣器这类板载基础 IO 直接在 bsp_board.c 中实现。
 * 通信、LCD、矩阵键盘等较复杂模块保留独立文件。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef enum {
    /* 板载 LED0：PC2。 */
    BSP_LED0 = 0,
    /* 板载 LED1：PC3。 */
    BSP_LED1,
    /* LED 数量，用于数组边界检查。 */
    BSP_LED_COUNT
} BSP_LED;

typedef enum {
    /* 板载 KEY0/SW1：PE5，低电平按下。 */
    BSP_KEY0 = 0,
    /* 板载 KEY1/SW2：PE4，低电平按下。 */
    BSP_KEY1,
    /* 板载 KEY2/SW3：PE3，低电平按下。 */
    BSP_KEY2,
    /* 板载 KEY3/SW4：PE2，低电平按下。 */
    BSP_KEY3,
    /* 按键数量，用于数组边界检查。 */
    BSP_KEY_COUNT
} BSP_Key;

typedef enum {
    /* 没有按键事件。 */
    BSP_KEY_EVENT_NONE = 0x0000,
    /* 长按标志预留位，当前简单扫描函数主要输出短按事件。 */
    BSP_KEY_EVENT_LONG = 0x8000,
    /* KEY0 短按事件。 */
    BSP_KEY0_PRESS = 0x0001,
    /* KEY1 短按事件。 */
    BSP_KEY1_PRESS = 0x0002,
    /* KEY2 短按事件。 */
    BSP_KEY2_PRESS = 0x0004,
    /* KEY3 短按事件。 */
    BSP_KEY3_PRESS = 0x0008
} BSP_KeyEvent;

/* LCD 软件 SPI/控制脚定义，来自开发板资料：NV3021，160x128。 */
#define BSP_LCD_CS_Pin GPIO_PIN_0
#define BSP_LCD_CS_GPIO_Port GPIOC
#define BSP_LCD_BL_Pin GPIO_PIN_1
#define BSP_LCD_BL_GPIO_Port GPIOC
#define BSP_LCD_SDA_Pin GPIO_PIN_13
#define BSP_LCD_SDA_GPIO_Port GPIOC
#define BSP_LCD_SCL_Pin GPIO_PIN_14
#define BSP_LCD_SCL_GPIO_Port GPIOC
#define BSP_LCD_RST_Pin GPIO_PIN_15
#define BSP_LCD_RST_GPIO_Port GPIOC
#define BSP_LCD_DC_Pin GPIO_PIN_6
#define BSP_LCD_DC_GPIO_Port GPIOE

/* 板载 LED 和蜂鸣器引脚定义。 */
#define BSP_LED0_Pin GPIO_PIN_2
#define BSP_LED0_GPIO_Port GPIOC
#define BSP_LED1_Pin GPIO_PIN_3
#define BSP_LED1_GPIO_Port GPIOC
#define BSP_BEEP_Pin GPIO_PIN_15
#define BSP_BEEP_GPIO_Port GPIOD

/* 板载独立按键引脚定义，按下为低电平，初始化时使用上拉输入。 */
#define BSP_KEY0_Pin GPIO_PIN_5
#define BSP_KEY1_Pin GPIO_PIN_4
#define BSP_KEY2_Pin GPIO_PIN_3
#define BSP_KEY3_Pin GPIO_PIN_2
#define BSP_KEY_GPIO_Port GPIOE

/* SPI1 软件片选和 RS485 方向控制脚。 */
#define BSP_SPI1_CS_Pin GPIO_PIN_4
#define BSP_SPI1_CS_GPIO_Port GPIOA
#define BSP_RS485_DE_Pin GPIO_PIN_11
#define BSP_RS485_DE_GPIO_Port GPIOA

/* 2026F controller: PB0/PB1 are FPGA IRQ/reset. PE0 remains unused. */
#define F2026_FPGA_IRQ_Pin GPIO_PIN_0
#define F2026_FPGA_IRQ_GPIO_Port GPIOB
#define F2026_FPGA_RESET_Pin GPIO_PIN_1
#define F2026_FPGA_RESET_GPIO_Port GPIOB

/* 初始化板载 GPIO 相关模块：LCD 引脚、LED、蜂鸣器、按键、SPI CS、RS485 DE。 */
void BSP_Board_Init(void);

/* 初始化板载 LED。 */
void BSP_LED_Init(void);

/* 控制板载 LED。on=true 点亮，on=false 熄灭。 */
void BSP_LED_Write(BSP_LED led, bool on);

/* 翻转指定板载 LED。 */
void BSP_LED_Toggle(BSP_LED led);

/* 初始化蜂鸣器。 */
void BSP_Beep_Init(void);

/* 控制蜂鸣器输出。 */
void BSP_Beep_Write(bool on);

/* 初始化板载独立按键。 */
void BSP_Key_Init(void);

/* 读取指定按键当前是否处于按下状态。 */
bool BSP_Key_IsDown(BSP_Key key);

/* 扫描按键事件，返回 BSP_KEYx_PRESS 位掩码。 */
uint16_t BSP_Key_Scan(void);

/* 兼容别名，便于 main/app 中用板级前缀表达用途。 */
#define BSP_Board_LED_Write BSP_LED_Write
#define BSP_Board_LED_Toggle BSP_LED_Toggle
#define BSP_Board_Beep_Write BSP_Beep_Write
#define BSP_Board_Key_IsDown BSP_Key_IsDown
#define BSP_Board_Key_Scan BSP_Key_Scan

#ifdef __cplusplus
}
#endif

#endif
