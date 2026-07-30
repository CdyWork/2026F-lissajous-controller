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
#include <stdio.h>
#include <string.h>

#define F2026_TASK_STACK_WORDS 768U
#define F2026_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
typedef struct {
    F2026_FpgaMode mode;
    bool free_run;
    uint8_t amplitude_index;
    uint8_t amplitude_codes[4];
    uint32_t free_frequency_hz;
    uint32_t user_phase_word;
    uint8_t probe_ramp_mode;
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
    .user_phase_word = 0U,
    .probe_ramp_mode = 1U,
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
static int F2026_ProbeRampMode(uint32_t ramp_us);
static uint32_t F2026_ProbeRampUs(uint8_t ramp_mode);

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
    memset(&control, 0, sizeof(control));
    control.mode = state.mode;
    control.amplitude_code = state.amplitude_codes[state.amplitude_index];
    control.free_run = state.free_run;
    control.phase_offset = state.user_phase_word;
    control.dac_mid = 0x80U;
    control.threshold_hysteresis = 3U;
    control.probe_ramp_mode = state.probe_ramp_mode;

    if (state.mode != F2026_FPGA_MODE_IDLE) {
        if (state.mode == F2026_FPGA_MODE_PROBE) {
            control.output_enable = true;
        } else if (state.free_run) {
            control.phase_increment =
                F2026_PhaseIncrementFromHz(state.free_frequency_hz);
            control.output_enable = control.phase_increment != 0U;
        } else {
            // Tracking frequency, phase correction and loss-of-lock muting are
            // autonomous in the FPGA. The MCU only arms the selected mode.
            control.output_enable = true;
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
        case F2026_PI_COMMAND_PROBE:
            amplitude_index = F2026_ProbeRampMode(command.value);
            if (amplitude_index >= 0) {
                state.mode = F2026_FPGA_MODE_PROBE;
                state.free_run = true;
                state.probe_ramp_mode = (uint8_t)amplitude_index;
                F2026_PiReply("OK PROBE\r\n");
            } else {
                F2026_PiReply("ERR PROBE\r\n");
            }
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
    uint32_t frequency_hz = 0U;
    uint16_t status_color = state.communication_ok ? BSP_LCD_GREEN : BSP_LCD_RED;

    if (state.free_run) {
        frequency_hz = state.free_frequency_hz;
    } else if (state.fpga_status.period_ticks != 0U) {
        frequency_hz = 50000000U / state.fpga_status.period_ticks;
    }

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
    BSP_LCD_ShowU32(120U, 36U, state.amplitude_codes[state.amplitude_index],
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
    uint32_t frequency_hz = state.free_run ? state.free_frequency_hz :
        ((state.fpga_status.period_ticks == 0U)
             ? 0U : (50000000U / state.fpga_status.period_ticks));

    (void)snprintf(response,
                   sizeof(response),
                   "STATUS MODE=%s AUTO=%u AMP=%u FREQ=%lu RAMP=%lu LOCK=%u ADC=%u,%u OTR=%u OUTPUT=%u COMM=%u FOUT=%u VER=%u\r\n",
                   F2026_ModeName(state.mode),
                   state.free_run ? 1U : 0U,
                   amplitude_divisions[state.amplitude_index],
                   (unsigned long)frequency_hz,
                   (unsigned long)F2026_ProbeRampUs(state.fpga_status.probe_ramp_mode),
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
    case F2026_FPGA_MODE_PROBE:
        return "PROBE";
    case F2026_FPGA_MODE_IDLE:
    default:
        return "IDLE";
    }
}

static int F2026_ProbeRampMode(uint32_t ramp_us)
{
    switch (ramp_us) {
    case 100U:
        return 4;
    case 200U:
        return 5;
    case 500U:
        return 0;
    case 1000U:
        return 1;
    case 2000U:
        return 2;
    case 5000U:
        return 3;
    default:
        return -1;
    }
}

static uint32_t F2026_ProbeRampUs(uint8_t ramp_mode)
{
    switch (ramp_mode) {
    case 0U:
        return 500U;
    case 2U:
        return 2000U;
    case 3U:
        return 5000U;
    case 4U:
        return 100U;
    case 5U:
        return 200U;
    case 1U:
    default:
        return 1000U;
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
