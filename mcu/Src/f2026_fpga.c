#include "f2026_fpga.h"

#include "bsp_board.h"
#include "bsp_spi.h"

#include <string.h>

#define F2026_SPI_FRAME_SIZE 16U
#define F2026_CMD_READ_STATUS 0x01U
#define F2026_CMD_SET_CONTROL 0x10U
#define F2026_STATUS_SIGNATURE 0xF6U
#define F2026_FPGA_CLOCK_HZ 50000000ULL

static uint32_t unpack_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void pack_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static bool transfer_frame(const uint8_t *tx, uint8_t *rx)
{
    HAL_StatusTypeDef result;

    BSP_SPI1_CS_Write(true);
    result = BSP_SPI1_TransmitReceive(tx, rx, F2026_SPI_FRAME_SIZE, 20U);
    BSP_SPI1_CS_Write(false);
    return result == HAL_OK;
}

void F2026_FpgaInterfaceInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    F2026_FpgaReset(false);
    BSP_SPI1_InitModeSpeed(BSP_SPI_MODE_FULL_DUPLEX, 5000000U);
}

void F2026_FpgaReset(bool release_reset)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
                      release_reset ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool F2026_FpgaReadStatus(F2026_FpgaStatus *status)
{
    uint8_t tx[F2026_SPI_FRAME_SIZE] = {0};
    uint8_t rx[F2026_SPI_FRAME_SIZE] = {0};
    uint8_t flags;

    if (status == 0) {
        return false;
    }

    tx[0] = F2026_CMD_READ_STATUS;
    if (!transfer_frame(tx, rx) || (rx[1] != F2026_STATUS_SIGNATURE)) {
        return false;
    }

    flags = rx[3];
    status->protocol_version = rx[2];
    status->locked = (flags & 0x01U) != 0U;
    status->otr_seen = (flags & 0x02U) != 0U;
    status->output_enabled = (flags & 0x04U) != 0U;
    status->free_run = (flags & 0x08U) != 0U;
    status->period_ticks = unpack_u32_le(&rx[4]);
    status->edge_count = unpack_u32_le(&rx[8]);
    status->sample_min = rx[12];
    status->sample_max = rx[13];
    status->mode = (F2026_FpgaMode)(rx[14] & 0x07U);
    status->probe_ramp_mode = (uint8_t)(rx[14] >> 3U);
    status->amplitude_code = rx[15];
    return true;
}

bool F2026_FpgaWriteControl(const F2026_FpgaControl *control)
{
    uint8_t tx[F2026_SPI_FRAME_SIZE] = {0};
    uint8_t rx[F2026_SPI_FRAME_SIZE] = {0};

    if (control == 0) {
        return false;
    }

    tx[0] = F2026_CMD_SET_CONTROL;
    tx[1] = (uint8_t)control->mode;
    tx[2] = control->amplitude_code;
    tx[3] = (control->output_enable ? 0x01U : 0U) |
            (control->free_run ? 0x02U : 0U);
    pack_u32_le(&tx[4], control->phase_increment);
    pack_u32_le(&tx[8], control->phase_offset);
    tx[12] = control->dac_mid;
    tx[13] = control->threshold_hysteresis;
    tx[14] = control->probe_ramp_mode;
    return transfer_frame(tx, rx);
}

uint32_t F2026_PhaseIncrementFromHz(uint32_t frequency_hz)
{
    if ((frequency_hz == 0U) || (frequency_hz > 200000U)) {
        return 0U;
    }
    return (uint32_t)((((uint64_t)frequency_hz << 32U) +
                       (F2026_FPGA_CLOCK_HZ / 2ULL)) /
                      F2026_FPGA_CLOCK_HZ);
}

uint32_t F2026_PhaseWordFromDegrees(uint32_t degrees)
{
    degrees %= 360U;
    return (uint32_t)(((uint64_t)degrees << 32U) / 360ULL);
}
