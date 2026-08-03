#include "bsp_board.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} BSP_GPIO_Pin;

/* LED0=PC2，LED1=PC3。 */
static const BSP_GPIO_Pin led_table[BSP_LED_COUNT] = {
    {BSP_LED0_GPIO_Port, BSP_LED0_Pin},
    {BSP_LED1_GPIO_Port, BSP_LED1_Pin},
};

/* KEY0-KEY3 均在 GPIOE，按下为低电平。 */
static const BSP_GPIO_Pin key_table[BSP_KEY_COUNT] = {
    {BSP_KEY_GPIO_Port, BSP_KEY0_Pin},
    {BSP_KEY_GPIO_Port, BSP_KEY1_Pin},
    {BSP_KEY_GPIO_Port, BSP_KEY2_Pin},
    {BSP_KEY_GPIO_Port, BSP_KEY3_Pin},
};

void BSP_Board_Init(void)
{
    /* 板级基础 IO 初始化：LED、蜂鸣器、按键。 */
    BSP_LED_Init();
    BSP_Beep_Init();
    BSP_Key_Init();
}

void BSP_LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = BSP_LED0_Pin | BSP_LED1_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    BSP_LED_Write(BSP_LED0, false);
    BSP_LED_Write(BSP_LED1, false);
}

void BSP_LED_Write(BSP_LED led, bool on)
{
    if (led >= BSP_LED_COUNT) {
        return;
    }

    HAL_GPIO_WritePin(led_table[led].port, led_table[led].pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_LED_Toggle(BSP_LED led)
{
    if (led >= BSP_LED_COUNT) {
        return;
    }

    HAL_GPIO_TogglePin(led_table[led].port, led_table[led].pin);
}

void BSP_Beep_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin = BSP_BEEP_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_BEEP_GPIO_Port, &gpio);

    BSP_Beep_Write(false);
}

void BSP_Beep_Write(bool on)
{
    HAL_GPIO_WritePin(BSP_BEEP_GPIO_Port, BSP_BEEP_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_Key_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio.Pin = BSP_KEY0_Pin | BSP_KEY1_Pin | BSP_KEY2_Pin | BSP_KEY3_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_KEY_GPIO_Port, &gpio);
}

bool BSP_Key_IsDown(BSP_Key key)
{
    if (key >= BSP_KEY_COUNT) {
        return false;
    }

    return HAL_GPIO_ReadPin(key_table[key].port, key_table[key].pin) == GPIO_PIN_RESET;
}

uint16_t BSP_Key_Scan(void)
{
    static uint8_t stable_count[BSP_KEY_COUNT];
    static uint8_t was_down[BSP_KEY_COUNT];
    uint16_t event = BSP_KEY_EVENT_NONE;

    for (uint8_t i = 0; i < BSP_KEY_COUNT; i++) {
        bool down = BSP_Key_IsDown((BSP_Key)i);

        if (down) {
            if (stable_count[i] < 3U) {
                stable_count[i]++;
            } else if (was_down[i] == 0U) {
                was_down[i] = 1U;
                event |= (uint16_t)(1U << i);
            }
        } else {
            stable_count[i] = 0U;
            was_down[i] = 0U;
        }
    }

    return event;
}
