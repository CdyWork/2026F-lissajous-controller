#include "f2026_app.h"

#include "f2026_fpga.h"
#include "f2026_pi.h"

#include "bsp_board.h"
#include "bsp_dma.h"
#include "bsp_lcd.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define F2026_TASK_STACK_WORDS 768U
#define F2026_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define F2026_DEFAULT_PHASE_DEGREES 14U
#define F2026_DEFAULT_PHASE_WORD 0x09F49F49U
#define F2026_DEFAULT_DAC_MID 0x80U
#define F2026_DOUBLE_PHASE_DEGREES 31U
#define F2026_DOUBLE_PHASE_WORD 0x160B60B6U
#define F2026_MIN_TRACK_FREQUENCY_HZ 1000U
#define F2026_MAX_TRACK_FREQUENCY_HZ 100000U
#define F2026_FREQUENCY_STEP_HZ 100U
#define F2026_FREQ_CAL_MID_START_HZ 50100U
#define F2026_FREQ_CAL_HIGH_START_HZ 87200U
#define F2026_TRACKING_PHASE_CALIBRATION_WORD 0xFE93E93FU
#define F2026_LOW_FREQUENCY_PHASE_BASE_WORD 0xFE38E38EU
#define F2026_LOW_FREQUENCY_INCREMENT_LIMIT_WORD 1065152U
#define F2026_TRACKING_LATENCY_CYCLES 53U
typedef struct {
    F2026_FpgaMode mode;
    bool free_run;
    uint8_t amplitude_index;
    uint8_t amplitude_codes[4];
    uint32_t free_frequency_hz;
    uint32_t user_phase_word;
    F2026_FpgaStatus fpga_status;
    F2026_FpgaControl last_control;
    bool last_control_valid;
    bool communication_ok;
    bool output_active;
} F2026_State;

static const uint8_t amplitude_divisions[4] = {2U, 4U, 6U, 8U};
static F2026_State state = {
    .mode = F2026_FPGA_MODE_IDLE,
    .free_run = false,
    .amplitude_index = 3U,
    .amplitude_codes = {13U, 26U, 38U, 51U},
    .free_frequency_hz = 1000U,
    .user_phase_word = F2026_DEFAULT_PHASE_WORD,
    .last_control_valid = false,
    .communication_ok = false,
    .output_active = false
};

static void F2026_Task(void *argument);
static void F2026_HardwareInit(void);
static void F2026_ApplyControl(void);
static void F2026_ServiceKeys(void);
static void F2026_ServicePi(void);
static void F2026_DrawStatus(void);
static void F2026_SendPiStatus(void);
static const char *F2026_ModeName(F2026_FpgaMode mode);
static int F2026_AmplitudeIndex(uint32_t divisions);
static uint32_t F2026_CurrentFrequencyHz(void);
static uint32_t F2026_QuantizeFrequencyHz(uint32_t frequency_hz);
static uint32_t F2026_PhaseOffsetForMode(F2026_FpgaMode mode,
                                         uint32_t frequency_hz,
                                         bool tracking);
static uint32_t F2026_TrackingCompensationWord(F2026_FpgaMode mode,
                                               uint32_t frequency_hz);
static uint16_t F2026_TrackingPhaseCorrectionTenthDegrees(uint32_t frequency_hz);
static uint32_t F2026_PhaseWordFromTenthDegrees(uint32_t tenth_degrees);

void F2026_AppStart(void)
{
    HAL_Init();
    SystemClock_Config();
    F2026_HardwareInit();

    if (xTaskCreate(F2026_Task,
                    "f2026",
                    F2026_TASK_STACK_WORDS,
                    0,
                    F2026_TASK_PRIORITY,
                    0) != pdPASS) {
        Error_Handler();
    }

    vTaskStartScheduler();
    Error_Handler();
}

static void F2026_HardwareInit(void)
{
    BSP_Board_Init();
    BSP_DMA_Init();

    F2026_FpgaInterfaceInit();
    F2026_PiInit();
    BSP_LCD_Init();

    HAL_Delay(10U);
    F2026_FpgaReset(true);
    HAL_Delay(20U);
    BSP_LCD_Clear(BSP_LCD_BLACK);
}

static void F2026_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t draw_divider = 0U;

    (void)argument;
    F2026_ApplyControl();

    for (;;) {
        F2026_FpgaStatus latest_status;

        if (F2026_FpgaReadStatus(&latest_status) &&
            (latest_status.protocol_version == F2026_FPGA_PROTOCOL_VERSION)) {
            state.fpga_status = latest_status;
            state.communication_ok = true;
            state.output_active = latest_status.output_enabled;

            if (state.last_control_valid &&
                ((latest_status.mode != state.last_control.mode) ||
                 (latest_status.amplitude_code != state.last_control.amplitude_code) ||
                 (latest_status.free_run != state.last_control.free_run))) {
                state.last_control_valid = false;
            }
        } else {
            state.communication_ok = false;
            state.output_active = false;
        }

        F2026_ServiceKeys();
        F2026_ServicePi();
        F2026_ApplyControl();

        if (++draw_divider >= 10U) {
            draw_divider = 0U;
            F2026_DrawStatus();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20U));
    }
}

static void F2026_ApplyControl(void)
{
    F2026_FpgaControl control;
    uint32_t frequency_hz = F2026_CurrentFrequencyHz();
    memset(&control, 0, sizeof(control));
    control.mode = state.mode;
    control.amplitude_code = state.amplitude_codes[state.amplitude_index];
    control.free_run = state.free_run;
    control.phase_offset =
        F2026_PhaseOffsetForMode(state.mode, frequency_hz, !state.free_run);
    control.dac_mid = F2026_DEFAULT_DAC_MID;
    control.threshold_hysteresis = 3U;

    if (state.mode != F2026_FPGA_MODE_IDLE) {
        if (state.free_run) {
            control.phase_increment =
                F2026_PhaseIncrementFromHz(state.free_frequency_hz);
            control.output_enable = control.phase_increment != 0U;
        } else {
            // The FPGA still locks the DDS on every input edge. Frequency-
            // dependent phase compensation is kept deterministic in the MCU,
            // so wait for a locked averaged period before enabling output.
            control.output_enable = (frequency_hz != 0U);
        }
    }

    if (!state.last_control_valid ||
        (memcmp(&control, &state.last_control, sizeof(control)) != 0)) {
        if (F2026_FpgaWriteControl(&control)) {
            state.last_control = control;
            state.last_control_valid = true;
        } else {
            state.communication_ok = false;
            state.output_active = false;
        }
    }
}

static void F2026_ServiceKeys(void)
{
    uint16_t event = BSP_Key_Scan();

    if ((event & BSP_KEY0_PRESS) != 0U) {
        state.mode = F2026_FPGA_MODE_DIAGONAL;
        state.free_run = false;
    }
    if ((event & BSP_KEY1_PRESS) != 0U) {
        state.mode = F2026_FPGA_MODE_CIRCLE;
        state.free_run = false;
    }
    if ((event & BSP_KEY2_PRESS) != 0U) {
        state.mode = F2026_FPGA_MODE_DOUBLE;
        state.free_run = false;
    }
    if ((event & BSP_KEY3_PRESS) != 0U) {
        state.amplitude_index = (uint8_t)((state.amplitude_index + 1U) % 4U);
    }

    if (event != BSP_KEY_EVENT_NONE) {
        BSP_Beep_Write(true);
        vTaskDelay(pdMS_TO_TICKS(15U));
        BSP_Beep_Write(false);
        state.last_control_valid = false;
    }
}

static void F2026_ServicePi(void)
{
    F2026_PiCommand command;

    while (F2026_PiPoll(&command)) {
        int amplitude_index;

        switch (command.type) {
        case F2026_PI_COMMAND_STATUS:
            F2026_SendPiStatus();
            break;
        case F2026_PI_COMMAND_BYPASS:
            state.mode = F2026_FPGA_MODE_DIAGONAL;
            state.free_run = false;
            F2026_PiReply("OK TRACK DIAG\r\n");
            break;
        case F2026_PI_COMMAND_TRACK:
            state.mode = command.mode;
            state.free_run = false;
            F2026_PiReply("OK TRACK\r\n");
            break;
        case F2026_PI_COMMAND_AUTO:
            state.mode = command.mode;
            state.free_run = true;
            F2026_PiReply("OK AUTO\r\n");
            break;
        case F2026_PI_COMMAND_AMPLITUDE:
            amplitude_index = F2026_AmplitudeIndex(command.value);
            if (amplitude_index >= 0) {
                state.amplitude_index = (uint8_t)amplitude_index;
                F2026_PiReply("OK AMP\r\n");
            } else {
                F2026_PiReply("ERR AMP\r\n");
            }
            break;
        case F2026_PI_COMMAND_FREQUENCY:
            if ((command.value >= 1000U) && (command.value <= 100000U)) {
                state.free_frequency_hz = command.value;
                F2026_PiReply("OK FREQ\r\n");
            } else {
                F2026_PiReply("ERR FREQ\r\n");
            }
            break;
        case F2026_PI_COMMAND_PHASE:
            state.user_phase_word = F2026_PhaseWordFromDegrees(command.value);
            F2026_PiReply("OK PHASE\r\n");
            break;
        case F2026_PI_COMMAND_CALIBRATE:
            amplitude_index = F2026_AmplitudeIndex(command.value);
            if ((amplitude_index >= 0) &&
                (command.value2 >= 1U) && (command.value2 <= 127U)) {
                state.amplitude_codes[amplitude_index] = (uint8_t)command.value2;
                F2026_PiReply("OK CAL\r\n");
            } else {
                F2026_PiReply("ERR CAL\r\n");
            }
            break;
        default:
            F2026_PiReply("ERR COMMAND\r\n");
            break;
        }

        state.last_control_valid = false;
    }
}

static void F2026_DrawStatus(void)
{
    uint32_t frequency_hz = F2026_CurrentFrequencyHz();
    uint16_t status_color = state.communication_ok ? BSP_LCD_GREEN : BSP_LCD_RED;

    BSP_LCD_Clear(BSP_LCD_BLACK);
    BSP_LCD_FillRect(0U, 0U, BSP_LCD_WIDTH, 14U, BSP_LCD_BLUE);
    BSP_LCD_ShowString(4U, 2U, "F2026 LISSAJOUS", BSP_LCD_WHITE, BSP_LCD_BLUE);

    BSP_LCD_ShowString(0U, 20U, "MODE", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(48U, 20U, F2026_ModeName(state.mode), BSP_LCD_WHITE, BSP_LCD_BLACK);
    if (state.free_run)
        BSP_LCD_ShowString(120U, 20U, "AUTO", BSP_LCD_YELLOW, BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 36U, "Y DIV", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(48U, 36U, amplitude_divisions[state.amplitude_index],
                    BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(80U, 36U, "CODE", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(120U, 36U,
                    state.last_control_valid ? state.last_control.amplitude_code :
                                               state.amplitude_codes[state.amplitude_index],
                    BSP_LCD_WHITE, BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 52U, "FREQ", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(48U, 52U, frequency_hz, BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(120U, 52U, "HZ", BSP_LCD_GRAY, BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 68U, "LOCK", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(48U, 68U,
                       state.fpga_status.locked ? "YES" : "NO",
                       state.fpga_status.locked ? BSP_LCD_GREEN : BSP_LCD_RED,
                       BSP_LCD_BLACK);
    BSP_LCD_ShowString(88U, 68U, "OTR", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(120U, 68U,
                       state.fpga_status.otr_seen ? "YES" : "NO",
                       state.fpga_status.otr_seen ? BSP_LCD_RED : BSP_LCD_GREEN,
                       BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 84U, "ADC", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(40U, 84U, state.fpga_status.sample_min, BSP_LCD_WHITE, BSP_LCD_BLACK);
    BSP_LCD_ShowString(72U, 84U, "-", BSP_LCD_GRAY, BSP_LCD_BLACK);
    BSP_LCD_ShowU32(88U, 84U, state.fpga_status.sample_max, BSP_LCD_WHITE, BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 100U, "FPGA", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(48U, 100U,
                       state.communication_ok ? "ONLINE" : "OFFLINE",
                       status_color,
                       BSP_LCD_BLACK);
    BSP_LCD_ShowString(112U, 100U,
                       state.output_active ? "RUN" : "MUTE",
                       state.output_active ? BSP_LCD_GREEN : BSP_LCD_YELLOW,
                       BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 116U, "0:1X 1:90 2:2X 3:A", BSP_LCD_GRAY, BSP_LCD_BLACK);
}

static void F2026_SendPiStatus(void)
{
    char response[192];
    uint32_t frequency_hz = F2026_CurrentFrequencyHz();

    (void)snprintf(response,
                   sizeof(response),
                   "STATUS MODE=%s AUTO=%u AMP=%u CODE=%u FREQ=%lu LOCK=%u ADC=%u,%u OTR=%u OUTPUT=%u COMM=%u FOUT=%u VER=%u\r\n",
                   F2026_ModeName(state.mode),
                   state.free_run ? 1U : 0U,
                   amplitude_divisions[state.amplitude_index],
                   state.last_control_valid ? state.last_control.amplitude_code :
                                              state.amplitude_codes[state.amplitude_index],
                   (unsigned long)frequency_hz,
                   state.fpga_status.locked ? 1U : 0U,
                   state.fpga_status.sample_min,
                   state.fpga_status.sample_max,
                   state.fpga_status.otr_seen ? 1U : 0U,
                   state.output_active ? 1U : 0U,
                   state.communication_ok ? 1U : 0U,
                   state.fpga_status.output_enabled ? 1U : 0U,
                   state.fpga_status.protocol_version);
    F2026_PiReply(response);
}

static const char *F2026_ModeName(F2026_FpgaMode mode)
{
    switch (mode) {
    case F2026_FPGA_MODE_DIAGONAL:
        return "DIAG";
    case F2026_FPGA_MODE_CIRCLE:
        return "CIRCLE";
    case F2026_FPGA_MODE_DOUBLE:
        return "DOUBLE";
    case F2026_FPGA_MODE_IDLE:
    default:
        return "IDLE";
    }
}

static int F2026_AmplitudeIndex(uint32_t divisions)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        if (amplitude_divisions[i] == divisions)
            return (int)i;
    }
    return -1;
}

static uint32_t F2026_CurrentFrequencyHz(void)
{
    uint32_t frequency_hz = 0U;

    if (state.free_run) {
        frequency_hz = state.free_frequency_hz;
    } else if (!state.fpga_status.locked) {
        return 0U;
    } else if (state.fpga_status.average_period_q8 != 0U) {
        frequency_hz =
            (uint32_t)((50000000ULL * 256ULL +
                        (state.fpga_status.average_period_q8 / 2ULL)) /
                       state.fpga_status.average_period_q8);
    }

    return F2026_QuantizeFrequencyHz(frequency_hz);
}

static uint32_t F2026_QuantizeFrequencyHz(uint32_t frequency_hz)
{
    if (frequency_hz == 0U) {
        return 0U;
    }

    frequency_hz =
        ((frequency_hz + (F2026_FREQUENCY_STEP_HZ / 2U)) /
         F2026_FREQUENCY_STEP_HZ) *
        F2026_FREQUENCY_STEP_HZ;

    if (frequency_hz >= F2026_FREQ_CAL_HIGH_START_HZ) {
        frequency_hz -= 200U;
    } else if (frequency_hz >= F2026_FREQ_CAL_MID_START_HZ) {
        frequency_hz -= 100U;
    }

    if (frequency_hz < F2026_MIN_TRACK_FREQUENCY_HZ) {
        return F2026_MIN_TRACK_FREQUENCY_HZ;
    }
    if (frequency_hz > F2026_MAX_TRACK_FREQUENCY_HZ) {
        return F2026_MAX_TRACK_FREQUENCY_HZ;
    }
    return frequency_hz;
}

static uint32_t F2026_PhaseOffsetForMode(F2026_FpgaMode mode,
                                         uint32_t frequency_hz,
                                         bool tracking)
{
    uint32_t phase_offset;

    if (mode == F2026_FPGA_MODE_DOUBLE) {
        phase_offset =
            F2026_DOUBLE_PHASE_WORD -
            F2026_PhaseWordFromTenthDegrees(
                2U * F2026_TrackingPhaseCorrectionTenthDegrees(frequency_hz));
    } else {
        phase_offset =
            state.user_phase_word -
            F2026_PhaseWordFromTenthDegrees(
                F2026_TrackingPhaseCorrectionTenthDegrees(frequency_hz));
    }

    if (tracking) {
        phase_offset += F2026_TrackingCompensationWord(mode, frequency_hz);
    }

    return phase_offset;
}

static uint32_t F2026_TrackingCompensationWord(F2026_FpgaMode mode,
                                               uint32_t frequency_hz)
{
    uint32_t phase_increment;
    uint32_t latency_phase;
    uint32_t low_frequency_phase = 0U;

    if (frequency_hz == 0U) {
        return 0U;
    }

    phase_increment = F2026_PhaseIncrementFromHz(frequency_hz);
    latency_phase = phase_increment * F2026_TRACKING_LATENCY_CYCLES;

    if (phase_increment <= F2026_LOW_FREQUENCY_INCREMENT_LIMIT_WORD) {
        low_frequency_phase =
            F2026_LOW_FREQUENCY_PHASE_BASE_WORD + (phase_increment * 28U);
    }

    return F2026_TRACKING_PHASE_CALIBRATION_WORD +
           low_frequency_phase +
           ((mode == F2026_FPGA_MODE_DOUBLE) ? (latency_phase * 2U) :
                                               latency_phase);
}

static uint16_t F2026_TrackingPhaseCorrectionTenthDegrees(uint32_t frequency_hz)
{
    uint32_t frequency_khz;
    int64_t correction_scaled;

    if (frequency_hz == 0U) {
        return 0U;
    }
    if (frequency_hz < 5000U) {
        frequency_hz = 5000U;
    } else if (frequency_hz > 100000U) {
        frequency_hz = 100000U;
    }

    frequency_khz = (frequency_hz + 500U) / 1000U;

    // Smooth curve fit for the latest single-frequency bench data.
    // x = frequency in kHz, y = correction in tenth-degrees.
    // y ~= 1.398501e-6*x^5 - 3.390005e-4*x^4 + 2.921774e-2*x^3
    //      - 1.067705*x^2 + 17.7423866*x - 65.3999567
    // Coefficients are scaled by 1e12 and evaluated with Horner's method.
    correction_scaled = 1398501LL;
    correction_scaled = correction_scaled * (int64_t)frequency_khz - 339000463LL;
    correction_scaled = correction_scaled * (int64_t)frequency_khz + 29217737569LL;
    correction_scaled = correction_scaled * (int64_t)frequency_khz - 1067704804504LL;
    correction_scaled = correction_scaled * (int64_t)frequency_khz + 17742386603967LL;
    correction_scaled = correction_scaled * (int64_t)frequency_khz - 65399956657636LL;

    if (correction_scaled <= 0LL) {
        return 0U;
    }
    correction_scaled = (correction_scaled + 500000000000LL) / 1000000000000LL;
    if (correction_scaled > 3600LL) {
        correction_scaled = 3600LL;
    }
    return (uint16_t)correction_scaled;
}

static uint32_t F2026_PhaseWordFromTenthDegrees(uint32_t tenth_degrees)
{
    tenth_degrees %= 3600U;
    return (uint32_t)((((uint64_t)tenth_degrees << 32U) + 1800ULL) / 3600ULL);
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    Error_Handler();
}

void vApplicationIdleHook(void)
{
}
