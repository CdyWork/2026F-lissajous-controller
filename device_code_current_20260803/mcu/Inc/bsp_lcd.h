#ifndef BSP_LCD_H
#define BSP_LCD_H

/*
 * NV3021 SPI LCD 驱动接口。
 *
 * 当前开发板 LCD 为 160x128、RGB565、软件 SPI。
 * LCD 引脚初始化、初始化序列和绘图函数都在 bsp_lcd.c。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t bsp_lcd_last_fill_us;
extern volatile uint32_t bsp_lcd_last_draw_us;
extern volatile uint32_t bsp_lcd_last_pixels;

/* 横屏显示宽度。 */
#define BSP_LCD_WIDTH 160U
/* 横屏显示高度。 */
#define BSP_LCD_HEIGHT 128U

/* 常用 RGB565 颜色定义。 */
#define BSP_LCD_BLACK 0x0000U
#define BSP_LCD_WHITE 0xFFFFU
#define BSP_LCD_BLUE 0x001FU
#define BSP_LCD_CYAN 0x07FFU
#define BSP_LCD_YELLOW 0xFFE0U
#define BSP_LCD_GREEN 0x07E0U
#define BSP_LCD_RED 0xF800U
#define BSP_LCD_GRAY 0x8410U

/* 初始化 LCD 控制器并清屏。调用前必须先初始化 LCD GPIO 引脚。 */
void BSP_LCD_Init(void);

/* 初始化 LCD 的 CS、BL、SDA、SCL、RST、DC 引脚为推挽输出。 */
void BSP_LCD_Pins_Init(void);

/* 独立控制 LCD 背光；关闭背光不会改变显存内容。 */
void BSP_LCD_SetBacklight(bool enabled);

/* 用指定颜色填充整屏。 */
void BSP_LCD_Clear(uint16_t color);

/* 填充矩形区域，超出屏幕边界会自动裁剪。 */
void BSP_LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* 将 RGB565 像素块直接写入 LCD，像素按行优先排列。 */
void BSP_LCD_DrawRGB565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels);

/* 绘制一个 ASCII 字符，使用 5x7 字模扩展成 8x10 像素区域。 */
void BSP_LCD_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg);

/* 显示字符串，当前仅支持 ASCII 字符。 */
void BSP_LCD_ShowString(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg);

/* 显示无符号 32 位整数。 */
void BSP_LCD_ShowU32(uint16_t x, uint16_t y, uint32_t value, uint16_t fg, uint16_t bg);

/* 模板首页：显示芯片、时钟、通信模块和运行时间。 */
void BSP_LCD_TemplateHome(uint32_t uptime_ms);

#ifdef __cplusplus
}
#endif

#endif
