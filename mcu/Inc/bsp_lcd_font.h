#ifndef BSP_LCD_FONT_H
#define BSP_LCD_FONT_H

/* LCD ASCII 字模表：5x7 点阵，当前覆盖空格到大写 Z。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ASCII 字模起始字符：空格 0x20。 */
#define BSP_FONT_ASCII_FIRST 32U
/* ASCII 字模结束字符：大写 Z。小写字母会转换成大写显示。 */
#define BSP_FONT_ASCII_LAST 90U
/* 字模宽度：5 列。 */
#define BSP_FONT_ASCII_WIDTH 5U
/* 字模高度：7 行。 */
#define BSP_FONT_ASCII_HEIGHT 7U

/* 5x7 ASCII 字模数组，每个字节表示一列，bit0-bit6 表示该列像素。 */
extern const uint8_t g_ascii_5x7[BSP_FONT_ASCII_LAST - BSP_FONT_ASCII_FIRST + 1U][BSP_FONT_ASCII_WIDTH];

/* 根据字符获取字模指针；不支持字符会退化为空格。 */
const uint8_t *BSP_LCD_Font_Get5x7(char ch);

#ifdef __cplusplus
}
#endif

#endif
