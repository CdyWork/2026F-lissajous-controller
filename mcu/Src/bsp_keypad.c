#include "bsp_keypad.h"

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define KEYPAD_NONE 0xFFU
#define KEYPAD_DEBOUNCE_MS 20U

static const uint16_t row_pins[4] = {
    GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
};
static const uint16_t column_pins[4] = {
    GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
};
static bool initialized;

static void set_all_rows(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOD,
                      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                      state);
}

void BSP_Keypad_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);
    set_all_rows(GPIO_PIN_SET);

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
    initialized = true;
}

static uint8_t scan_raw(void)
{
    if (!initialized) {
        return KEYPAD_NONE;
    }

    set_all_rows(GPIO_PIN_SET);
    for (uint8_t row = 0U; row < 4U; row++) {
        HAL_GPIO_WritePin(GPIOD, row_pins[row], GPIO_PIN_RESET);
        for (volatile uint32_t settle = 0U; settle < 80U; settle++) {
        }
        for (uint8_t column = 0U; column < 4U; column++) {
            if (HAL_GPIO_ReadPin(GPIOD, column_pins[column]) == GPIO_PIN_RESET) {
                HAL_GPIO_WritePin(GPIOD, row_pins[row], GPIO_PIN_SET);
                return (uint8_t)(row * 4U + column);
            }
        }
        HAL_GPIO_WritePin(GPIOD, row_pins[row], GPIO_PIN_SET);
    }
    return KEYPAD_NONE;
}

char BSP_Keypad_Scan(void)
{
    static const char key_map[16] = {
        'D', 'C', 'B', 'A',
        '#', '9', '6', '3',
        '0', '8', '5', '2',
        '*', '7', '4', '1',
    };
    static uint8_t previous_raw = KEYPAD_NONE;
    static uint8_t latched_raw = KEYPAD_NONE;
    static uint32_t changed_at_ms;
    uint8_t raw = scan_raw();
    uint32_t now_ms = HAL_GetTick();

    if (raw != previous_raw) {
        previous_raw = raw;
        changed_at_ms = now_ms;
        return '\0';
    }
    if ((uint32_t)(now_ms - changed_at_ms) < KEYPAD_DEBOUNCE_MS) {
        return '\0';
    }
    if (raw == KEYPAD_NONE) {
        latched_raw = KEYPAD_NONE;
        return '\0';
    }
    if (raw == latched_raw) {
        return '\0';
    }
    latched_raw = raw;
    return key_map[raw];
}
