#include "ad9833.h"

#include "bsp_board.h"

#define AD9833_CTRL_B28     0x2000U
#define AD9833_CTRL_FSELECT 0x0800U
#define AD9833_CTRL_RESET   0x0100U
#define AD9833_FREQ0_ADDR   0x4000U

#define AD9833_GPIO_SET(pin) (GPIOE->BSRR = (uint32_t)(pin))
#define AD9833_GPIO_RESET(pin) (GPIOE->BSRR = (uint32_t)(pin) << 16U)

static uint16_t control_word;
static uint32_t programmed_frequency_word;

static void AD9833_ShiftWord(uint16_t word)
{
    for (uint8_t bit = 0U; bit < 16U; ++bit) {
        if ((word & 0x8000U) != 0U) {
            AD9833_GPIO_SET(BSP_AD9833_DATA_Pin);
        } else {
            AD9833_GPIO_RESET(BSP_AD9833_DATA_Pin);
        }
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        AD9833_GPIO_RESET(BSP_AD9833_SCLK_Pin);
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        AD9833_GPIO_SET(BSP_AD9833_SCLK_Pin);
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        word <<= 1U;
    }
}

static void AD9833_WriteWord(uint16_t word)
{
    AD9833_GPIO_RESET(BSP_AD9833_FSYNC_Pin);
    AD9833_ShiftWord(word);
    AD9833_GPIO_SET(BSP_AD9833_FSYNC_Pin);
}

static uint32_t AD9833_FrequencyWord(uint32_t frequency_hz)
{
    return (uint32_t)(((uint64_t)frequency_hz * 268435456ULL +
                       (AD9833_MCLK_HZ / 2U)) /
                      AD9833_MCLK_HZ) & 0x0FFFFFFFUL;
}

static void AD9833_WriteFrequency(uint16_t address, uint32_t frequency_word)
{
    AD9833_WriteWord(address | (uint16_t)(frequency_word & 0x3FFFU));
    AD9833_WriteWord(address | (uint16_t)((frequency_word >> 14U) & 0x3FFFU));
}

void AD9833_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = BSP_AD9833_SCLK_Pin | BSP_AD9833_DATA_Pin |
               BSP_AD9833_FSYNC_Pin | BSP_AD9833_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &gpio);

    AD9833_GPIO_SET(BSP_AD9833_FSYNC_Pin | BSP_AD9833_CS_Pin);
    AD9833_GPIO_SET(BSP_AD9833_SCLK_Pin);
    AD9833_GPIO_RESET(BSP_AD9833_DATA_Pin);

    control_word = AD9833_CTRL_B28;
    programmed_frequency_word = 0U;
    AD9833_WriteWord(control_word | AD9833_CTRL_RESET);
}

void AD9833_SetFrequency(uint32_t frequency_hz)
{
    uint32_t frequency_word;

    if (frequency_hz == 0U) {
        return;
    }

    frequency_word = AD9833_FrequencyWord(frequency_hz);
    control_word = AD9833_CTRL_B28;
    programmed_frequency_word = frequency_word;

    AD9833_WriteWord(control_word | AD9833_CTRL_RESET);
    AD9833_WriteFrequency(AD9833_FREQ0_ADDR, frequency_word);
    AD9833_WriteWord(control_word);
}

void AD9833_Refresh(void)
{
    if (programmed_frequency_word == 0U) {
        return;
    }

    /* Scrub FREQ0 continuously without sending a RESET control word. */
    AD9833_WriteFrequency(AD9833_FREQ0_ADDR, programmed_frequency_word);
}
