#include "bsp_lcd.h"

#include "bsp_board.h"
#include "bsp_lcd_font.h"

#define LCD_ENABLE_GPIO_DMA     0U

#if LCD_ENABLE_GPIO_DMA
#define LCD_DMA_PIXELS_PER_CHUNK 8U
#define LCD_DMA_WORDS_PER_BYTE   16U
#define LCD_DMA_REPEAT           8U
#define LCD_DMA_WORDS_PER_PIXEL  (2U * LCD_DMA_WORDS_PER_BYTE * LCD_DMA_REPEAT)
#endif

volatile uint32_t bsp_lcd_last_fill_us;
volatile uint32_t bsp_lcd_last_draw_us;
volatile uint32_t bsp_lcd_last_pixels;

/* 以下为 LCD 内部辅助函数，不对外暴露。 */
static uint32_t lcd_now_us(void);
static void lcd_write8(uint8_t value);
static void lcd_write8_slow(uint8_t value);
static void lcd_write_pixels_cpu(const uint16_t *pixels, uint32_t count);
static void lcd_write_solid_cpu(uint16_t color, uint32_t count);
static void lcd_write_char_pixels(char ch, uint16_t fg, uint16_t bg);
static void lcd_write_string_pixels(const char *text, uint16_t count, uint16_t fg, uint16_t bg);
static uint16_t lcd_string_visible_chars(uint16_t x, const char *text);
#if LCD_ENABLE_GPIO_DMA
static bool lcd_write_pixels_dma(const uint16_t *pixels, uint32_t count);
static bool lcd_write_solid_dma(uint16_t color, uint32_t count);
static void lcd_dma_init(void);
static bool lcd_dma_transfer(const uint32_t *words, uint32_t length);
static uint32_t *lcd_dma_encode_byte(uint32_t *out, uint8_t value);
#endif
static void lcd_cmd(uint8_t cmd);
static void lcd_data8(uint8_t data);
static void lcd_set_address(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static char *append_u32(char *out, uint32_t value);
static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin);
static inline void gpio_reset(GPIO_TypeDef *port, uint16_t pin);

#if LCD_ENABLE_GPIO_DMA
static DMA_HandleTypeDef hdma_lcd_gpio;
static uint32_t lcd_dma_buffer[LCD_DMA_PIXELS_PER_CHUNK * LCD_DMA_WORDS_PER_PIXEL];
static bool lcd_dma_ready;
#endif

void BSP_LCD_Pins_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 先设置默认输出电平，再配置为输出，避免初始化瞬间出现毛刺。 */
    HAL_GPIO_WritePin(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LCD_BL_GPIO_Port, BSP_LCD_BL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LCD_SDA_GPIO_Port, BSP_LCD_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LCD_SCL_GPIO_Port, BSP_LCD_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LCD_RST_GPIO_Port, BSP_LCD_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin, GPIO_PIN_SET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = BSP_LCD_CS_Pin | BSP_LCD_BL_Pin | BSP_LCD_SDA_Pin |
               BSP_LCD_SCL_Pin | BSP_LCD_RST_Pin;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = BSP_LCD_DC_Pin;
    HAL_GPIO_Init(GPIOE, &gpio);
}

void BSP_LCD_SetBacklight(bool enabled)
{
    HAL_GPIO_WritePin(BSP_LCD_BL_GPIO_Port,
                      BSP_LCD_BL_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_LCD_Init(void)
{
    BSP_LCD_Pins_Init();
#if LCD_ENABLE_GPIO_DMA
    lcd_dma_ready = false;
    lcd_dma_init();
#endif

    /* LCD 硬复位：高 -> 低 -> 高。 */
    HAL_GPIO_WritePin(BSP_LCD_RST_GPIO_Port, BSP_LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(BSP_LCD_RST_GPIO_Port, BSP_LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(BSP_LCD_RST_GPIO_Port, BSP_LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);

    /*
     * NV3021 初始化序列。
     * 这些命令来自开发板例程，通常不建议随意改动。
     */
    lcd_cmd(0xB4); lcd_data8(0x07);
    lcd_cmd(0xB1); lcd_data8(0x14);
    lcd_cmd(0xFF); lcd_data8(0xA5);
    lcd_cmd(0xEC); lcd_data8(0x89);
    lcd_cmd(0xED); lcd_data8(0x25);
    lcd_cmd(0xEE); lcd_data8(0x22);
    lcd_cmd(0xF6); lcd_data8(0x10);
    lcd_cmd(0xC4); lcd_data8(0x0D);
    lcd_cmd(0xC5); lcd_data8(0x00);
    lcd_cmd(0xC6); lcd_data8(0x0E);
    lcd_cmd(0x11);
    HAL_Delay(10U);
    lcd_cmd(0x3A); lcd_data8(0x05);  /* RGB565，16bit/pixel。 */
    lcd_cmd(0x36); lcd_data8(0x20);  /* 显示方向：当前模板为横屏 160x128。 */
    lcd_cmd(0x29);

    /* 背光高电平打开。 */
    BSP_LCD_SetBacklight(true);
    BSP_LCD_Clear(BSP_LCD_BLACK);
}

void BSP_LCD_Clear(uint16_t color)
{
    /* 清屏本质上就是填充整个屏幕矩形。 */
    BSP_LCD_FillRect(0U, 0U, BSP_LCD_WIDTH, BSP_LCD_HEIGHT, color);
}

void BSP_LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t start_us;
    uint32_t pixels;

    /* 参数越界或空矩形直接返回。 */
    if ((x >= BSP_LCD_WIDTH) || (y >= BSP_LCD_HEIGHT) || (w == 0U) || (h == 0U)) {
        return;
    }
    if ((x + w) > BSP_LCD_WIDTH) {
        w = BSP_LCD_WIDTH - x;
    }
    if ((y + h) > BSP_LCD_HEIGHT) {
        h = BSP_LCD_HEIGHT - y;
    }

    pixels = (uint32_t)w * h;
    start_us = lcd_now_us();

    /* 设置 GRAM 写入窗口后连续写入 RGB565 像素数据。 */
    lcd_set_address(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_set(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
#if LCD_ENABLE_GPIO_DMA
    if (!lcd_write_solid_dma(color, pixels)) {
        lcd_write_solid_cpu(color, pixels);
    }
#else
    lcd_write_solid_cpu(color, pixels);
#endif
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);

    bsp_lcd_last_fill_us = lcd_now_us() - start_us;
    bsp_lcd_last_pixels = pixels;
}

void BSP_LCD_DrawRGB565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels)
{
    uint32_t start_us;
    uint32_t pixel_count;

    if ((pixels == 0) ||
        (x >= BSP_LCD_WIDTH) ||
        (y >= BSP_LCD_HEIGHT) ||
        (w == 0U) ||
        (h == 0U)) {
        return;
    }
    if ((x + w) > BSP_LCD_WIDTH) {
        w = BSP_LCD_WIDTH - x;
    }
    if ((y + h) > BSP_LCD_HEIGHT) {
        h = BSP_LCD_HEIGHT - y;
    }

    pixel_count = (uint32_t)w * h;
    start_us = lcd_now_us();

    lcd_set_address(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_set(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
#if LCD_ENABLE_GPIO_DMA
    if (!lcd_write_pixels_dma(pixels, pixel_count)) {
        lcd_write_pixels_cpu(pixels, pixel_count);
    }
#else
    lcd_write_pixels_cpu(pixels, pixel_count);
#endif
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);

    bsp_lcd_last_draw_us = lcd_now_us() - start_us;
    bsp_lcd_last_pixels = pixel_count;
}

void BSP_LCD_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    if ((x + 8U) > BSP_LCD_WIDTH || (y + 10U) > BSP_LCD_HEIGHT) {
        return;
    }

    lcd_set_address(x, y, (uint16_t)(x + 7U), (uint16_t)(y + 9U));
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_set(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
    lcd_write_char_pixels(ch, fg, bg);
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
}

void BSP_LCD_ShowString(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg)
{
    uint16_t visible_chars;

    if ((text == 0) || (x >= BSP_LCD_WIDTH) || ((y + 10U) > BSP_LCD_HEIGHT)) {
        return;
    }

    visible_chars = lcd_string_visible_chars(x, text);
    if (visible_chars == 0U) {
        return;
    }

    lcd_set_address(x,
                    y,
                    (uint16_t)(x + (visible_chars * 8U) - 1U),
                    (uint16_t)(y + 9U));
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_set(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
    lcd_write_string_pixels(text, visible_chars, fg, bg);
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
}

void BSP_LCD_ShowU32(uint16_t x, uint16_t y, uint32_t value, uint16_t fg, uint16_t bg)
{
    /* 避免引入 sprintf，手写整数转字符串可减小嵌入式工程体积。 */
    char buf[11];
    char *end = append_u32(buf, value);

    *end = '\0';
    BSP_LCD_ShowString(x, y, buf, fg, bg);
}

void BSP_LCD_TemplateHome(uint32_t uptime_ms)
{
    /* 模板默认首页，方便上电后直观看到 LCD 和 RTOS 任务在运行。 */
    BSP_LCD_Clear(BSP_LCD_BLACK);
    BSP_LCD_FillRect(0U, 0U, BSP_LCD_WIDTH, 18U, BSP_LCD_BLUE);
    BSP_LCD_ShowString(0U, 1U, "HAL TEMPLATE", BSP_LCD_WHITE, BSP_LCD_BLUE);
    BSP_LCD_ShowString(0U, 28U, "STM32F407VET6", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(0U, 46U, "SYS 168MHZ", BSP_LCD_GREEN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(0U, 64U, "UART SPI I2C RS485", BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(0U, 84U, "UPTIME:", BSP_LCD_YELLOW, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(64U, 84U, uptime_ms / 1000U, BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(104U, 84U, "S", BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(0U, 108U, "KEY0 BEEP KEY1 TX", BSP_LCD_GRAY, BSP_LCD_BLACK);
}

static void lcd_write8(uint8_t value)
{
    /* 软件 SPI：从最高位开始，SCL 低电平准备数据，高电平锁存。 */
    for (uint8_t i = 0; i < 8U; i++) {
        uint32_t data_word = ((value & 0x80U) != 0U) ?
                             BSP_LCD_SDA_Pin :
                             ((uint32_t)BSP_LCD_SDA_Pin << 16U);

        BSP_LCD_SCL_GPIO_Port->BSRR = ((uint32_t)BSP_LCD_SCL_Pin << 16U) | data_word;
        BSP_LCD_SCL_GPIO_Port->BSRR = BSP_LCD_SCL_Pin;
        value <<= 1U;
    }
}

static uint32_t lcd_now_us(void)
{
    uint32_t ms = HAL_GetTick();
    uint32_t load = SysTick->LOAD + 1U;
    uint32_t val = SysTick->VAL;
    uint32_t us_in_ms;

    if (load == 0U) {
        return ms * 1000U;
    }
    us_in_ms = ((load - val) * 1000U) / load;
    return (ms * 1000U) + us_in_ms;
}

static void lcd_write8_slow(uint8_t value)
{
    for (uint8_t i = 0; i < 8U; i++) {
        gpio_reset(BSP_LCD_SCL_GPIO_Port, BSP_LCD_SCL_Pin);
        if ((value & 0x80U) != 0U) {
            gpio_set(BSP_LCD_SDA_GPIO_Port, BSP_LCD_SDA_Pin);
        } else {
            gpio_reset(BSP_LCD_SDA_GPIO_Port, BSP_LCD_SDA_Pin);
        }
        value <<= 1U;
        gpio_set(BSP_LCD_SCL_GPIO_Port, BSP_LCD_SCL_Pin);
    }
}

static void lcd_write_pixels_cpu(const uint16_t *pixels, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        uint16_t color = pixels[i];

        lcd_write8((uint8_t)(color >> 8U));
        lcd_write8((uint8_t)color);
    }
}

static void lcd_write_solid_cpu(uint16_t color, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        lcd_write8((uint8_t)(color >> 8U));
        lcd_write8((uint8_t)color);
    }
}

static void lcd_write_char_pixels(char ch, uint16_t fg, uint16_t bg)
{
    const uint8_t *glyph = BSP_LCD_Font_Get5x7(ch);

    for (uint8_t row = 0U; row < 10U; row++) {
        for (uint8_t col = 0U; col < 8U; col++) {
            uint8_t on = (row < BSP_FONT_ASCII_HEIGHT) &&
                         (col < BSP_FONT_ASCII_WIDTH) &&
                         ((glyph[col] & (1U << row)) != 0U);
            uint16_t color = on ? fg : bg;

            lcd_write8((uint8_t)(color >> 8U));
            lcd_write8((uint8_t)color);
        }
    }
}

static void lcd_write_string_pixels(const char *text, uint16_t count, uint16_t fg, uint16_t bg)
{
    for (uint8_t row = 0U; row < 10U; row++) {
        for (uint16_t i = 0U; i < count; i++) {
            const uint8_t *glyph = BSP_LCD_Font_Get5x7(text[i]);

            for (uint8_t col = 0U; col < 8U; col++) {
                uint8_t on = (row < BSP_FONT_ASCII_HEIGHT) &&
                             (col < BSP_FONT_ASCII_WIDTH) &&
                             ((glyph[col] & (1U << row)) != 0U);
                uint16_t color = on ? fg : bg;

                lcd_write8((uint8_t)(color >> 8U));
                lcd_write8((uint8_t)color);
            }
        }
    }
}

static uint16_t lcd_string_visible_chars(uint16_t x, const char *text)
{
    uint16_t visible_chars = 0U;

    while ((text[visible_chars] != '\0') &&
           ((x + ((visible_chars + 1U) * 8U)) <= BSP_LCD_WIDTH)) {
        visible_chars++;
    }

    return visible_chars;
}

#if LCD_ENABLE_GPIO_DMA
static bool lcd_write_pixels_dma(const uint16_t *pixels, uint32_t count)
{
    if (!lcd_dma_ready) {
        return false;
    }

    while (count > 0U) {
        uint32_t chunk = (count > LCD_DMA_PIXELS_PER_CHUNK) ? LCD_DMA_PIXELS_PER_CHUNK : count;
        uint32_t *out = lcd_dma_buffer;

        for (uint32_t i = 0; i < chunk; i++) {
            uint16_t color = pixels[i];

            out = lcd_dma_encode_byte(out, (uint8_t)(color >> 8U));
            out = lcd_dma_encode_byte(out, (uint8_t)color);
        }

        if (!lcd_dma_transfer(lcd_dma_buffer, (uint32_t)(out - lcd_dma_buffer))) {
            return false;
        }

        pixels += chunk;
        count -= chunk;
    }

    return true;
}

static bool lcd_write_solid_dma(uint16_t color, uint32_t count)
{
    if (!lcd_dma_ready) {
        return false;
    }

    while (count > 0U) {
        uint32_t chunk = (count > LCD_DMA_PIXELS_PER_CHUNK) ? LCD_DMA_PIXELS_PER_CHUNK : count;
        uint32_t *out = lcd_dma_buffer;

        for (uint32_t i = 0; i < chunk; i++) {
            out = lcd_dma_encode_byte(out, (uint8_t)(color >> 8U));
            out = lcd_dma_encode_byte(out, (uint8_t)color);
        }

        if (!lcd_dma_transfer(lcd_dma_buffer, (uint32_t)(out - lcd_dma_buffer))) {
            return false;
        }

        count -= chunk;
    }

    return true;
}

static void lcd_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_lcd_gpio.Instance = DMA2_Stream1;
    hdma_lcd_gpio.Init.Channel = DMA_CHANNEL_0;
    hdma_lcd_gpio.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_lcd_gpio.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_lcd_gpio.Init.MemInc = DMA_MINC_ENABLE;
    hdma_lcd_gpio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_lcd_gpio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_lcd_gpio.Init.Mode = DMA_NORMAL;
    hdma_lcd_gpio.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_lcd_gpio.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    lcd_dma_ready = (HAL_DMA_Init(&hdma_lcd_gpio) == HAL_OK);
}

static bool lcd_dma_transfer(const uint32_t *words, uint32_t length)
{
    HAL_StatusTypeDef status;

    if ((words == 0) || (length == 0U) || (length >= 0x10000U)) {
        return false;
    }

    status = HAL_DMA_Start(&hdma_lcd_gpio,
                           (uint32_t)words,
                           (uint32_t)&BSP_LCD_SDA_GPIO_Port->BSRR,
                           length);
    if (status != HAL_OK) {
        return false;
    }

    status = HAL_DMA_PollForTransfer(&hdma_lcd_gpio, HAL_DMA_FULL_TRANSFER, 20U);
    if (status != HAL_OK) {
        (void)HAL_DMA_Abort(&hdma_lcd_gpio);
        return false;
    }

    return true;
}

static uint32_t *lcd_dma_encode_byte(uint32_t *out, uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        uint32_t data_mask = ((value & 0x80U) != 0U) ?
                             BSP_LCD_SDA_Pin :
                             ((uint32_t)BSP_LCD_SDA_Pin << 16U);
        uint32_t low_word = ((uint32_t)BSP_LCD_SCL_Pin << 16U) | data_mask;
        uint32_t high_word = BSP_LCD_SCL_Pin;

        for (uint8_t hold = 0U; hold < LCD_DMA_REPEAT; hold++) {
            *out++ = low_word;
        }
        for (uint8_t hold = 0U; hold < LCD_DMA_REPEAT; hold++) {
            *out++ = high_word;
        }
        value <<= 1U;
    }

    return out;
}
#endif

static void lcd_cmd(uint8_t cmd)
{
    /* DC=0 表示写命令。 */
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_reset(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
    lcd_write8_slow(cmd);
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
}

static void lcd_data8(uint8_t data)
{
    /* DC=1 表示写数据。 */
    gpio_reset(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
    gpio_set(BSP_LCD_DC_GPIO_Port, BSP_LCD_DC_Pin);
    lcd_write8_slow(data);
    gpio_set(BSP_LCD_CS_GPIO_Port, BSP_LCD_CS_Pin);
}

static void lcd_set_address(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    /* 0x2A 设置列地址，0x2B 设置行地址，0x2C 开始写显存。 */
    lcd_cmd(0x2A);
    lcd_data8((uint8_t)(x0 >> 8U));
    lcd_data8((uint8_t)x0);
    lcd_data8((uint8_t)(x1 >> 8U));
    lcd_data8((uint8_t)x1);
    lcd_cmd(0x2B);
    lcd_data8((uint8_t)(y0 >> 8U));
    lcd_data8((uint8_t)y0);
    lcd_data8((uint8_t)(y1 >> 8U));
    lcd_data8((uint8_t)y1);
    lcd_cmd(0x2C);
}

static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = pin;
}

static inline void gpio_reset(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = (uint32_t)pin << 16U;
}

static char *append_u32(char *out, uint32_t value)
{
    /* 先倒序取出十进制数字，再反向写回输出缓冲区。 */
    char tmp[10];
    uint8_t len = 0U;

    do {
        tmp[len++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (len > 0U) {
        *out++ = tmp[--len];
    }

    return out;
}
