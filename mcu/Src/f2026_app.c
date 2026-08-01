#include "f2026_app.h"

#include "f2026_fpga.h"
#include "f2026_pi.h"

#include "ad9833.h"

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
#define F2026_AD9833_TASK_STACK_WORDS 192U
#define F2026_AD9833_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define F2026_AD9833_OUTPUT_HZ 4000U
#define F2026_AD9833_WRITE_PERIOD_MS 16U
#define F2026_DEFAULT_PHASE_DEGREES 14U
#define F2026_DEFAULT_PHASE_WORD 0x09F49F49U
#define F2026_PROBE_DEFAULT_FRAME_US 2000U
#define F2026_PROBE_DEFAULT_RAMP_US 50U
#define F2026_PROBE_MIN_RAMP_US 10U
#define F2026_PROBE_MAX_RAMP_US 1800U
#define F2026_PROBE_FRAME_QUANTUM_US 200U
#define F2026_PROBE_MIN_FRAME_US F2026_PROBE_FRAME_QUANTUM_US
#define F2026_PROBE_MAX_FRAME_US 2000U
#define F2026_FPGA_TICKS_PER_US 50U
#define F2026_PROBE_SWEEP_FRAME_US 2000U
#define F2026_PROBE_SWEEP_FIRST_RAMP_US 10U
#define F2026_PROBE_TABLE_MID_INDEX 16U
#define F2026_VISION_MEASUREMENT_TIMEOUT_MS 12000U
typedef struct {
    F2026_FpgaMode mode;
    bool free_run;
    uint8_t amplitude_index;
    uint8_t amplitude_codes[4];
    uint32_t free_frequency_hz;
    uint32_t probe_ramp_us;
    uint32_t probe_frame_us;
    uint32_t user_phase_word;
    F2026_FpgaStatus fpga_status;
    F2026_FpgaControl last_control;
    bool last_control_valid;
    bool communication_ok;
    bool output_active;
    uint32_t vision_frequency_hz;
    bool vision_frequency_valid;
    bool vision_measurement_pending;
    uint32_t vision_measurement_started_ms;
} F2026_State;

static const uint8_t amplitude_divisions[4] = {2U, 4U, 6U, 8U};
static F2026_State state = {
    .mode = F2026_FPGA_MODE_IDLE,
    .free_run = false,
    .amplitude_index = 3U,
    .amplitude_codes = {13U, 26U, 38U, 51U},
    .free_frequency_hz = 1000U,
    .probe_ramp_us = F2026_PROBE_DEFAULT_RAMP_US,
    .probe_frame_us = F2026_PROBE_DEFAULT_FRAME_US,
    .user_phase_word = F2026_DEFAULT_PHASE_WORD,
    .last_control_valid = false,
    .communication_ok = false,
    .output_active = false,
    .vision_frequency_hz = 0U,
    .vision_frequency_valid = false,
    .vision_measurement_pending = false,
    .vision_measurement_started_ms = 0U
};

static void F2026_Task(void *argument);
static void F2026_AD9833WriteTask(void *argument);
static void F2026_HardwareInit(void);
static void F2026_ApplyControl(void);
static void F2026_ServiceKeys(void);
static void F2026_ServicePi(void);
static void F2026_DrawStatus(void);
static void F2026_SendPiStatus(void);
static const char *F2026_ModeName(F2026_FpgaMode mode);
static int F2026_AmplitudeIndex(uint32_t divisions);

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

    if (xTaskCreate(F2026_AD9833WriteTask,
                    "ad9833",
                    F2026_AD9833_TASK_STACK_WORDS,
                    0,
                    F2026_AD9833_TASK_PRIORITY,
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

    AD9833_Init();
    AD9833_SetFrequency(F2026_AD9833_OUTPUT_HZ);

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

static void F2026_AD9833WriteTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;) {
        AD9833_Refresh();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(F2026_AD9833_WRITE_PERIOD_MS));
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

    if (state.mode != F2026_FPGA_MODE_IDLE) {
        if (state.mode == F2026_FPGA_MODE_PROBE) {
            control.free_run = true;
            control.phase_increment = state.probe_ramp_us * F2026_FPGA_TICKS_PER_US;
            control.phase_offset = state.probe_frame_us * F2026_FPGA_TICKS_PER_US;
            control.output_enable = true;
        } else if (state.mode == F2026_FPGA_MODE_PROBE_SWEEP) {
            // The FPGA owns all eight ramp widths and switches only at the
            // 2 ms frame boundary. These values merely arm the free-run path.
            control.free_run = true;
            control.phase_increment =
                F2026_PROBE_SWEEP_FIRST_RAMP_US * F2026_FPGA_TICKS_PER_US;
            control.phase_offset =
                F2026_PROBE_SWEEP_FRAME_US * F2026_FPGA_TICKS_PER_US;
            control.output_enable = true;
        } else if (state.mode == F2026_FPGA_MODE_PROBE_TABLE) {
            // phase_increment carries the 0..31 table index. The FPGA latches
            // it on the next 2 ms frame boundary.
            control.free_run = true;
            control.phase_increment = state.probe_ramp_us;
            control.phase_offset =
                F2026_PROBE_SWEEP_FRAME_US * F2026_FPGA_TICKS_PER_US;
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

    if (state.vision_measurement_pending &&
        ((uint32_t)(HAL_GetTick() - state.vision_measurement_started_ms) >=
         F2026_VISION_MEASUREMENT_TIMEOUT_MS)) {
        state.vision_measurement_pending = false;
    }

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
    if (((event & BSP_KEY3_PRESS) != 0U) && !state.vision_measurement_pending) {
        state.vision_frequency_hz = 0U;
        state.vision_frequency_valid = false;
        state.vision_measurement_pending = true;
        state.vision_measurement_started_ms = HAL_GetTick();
        F2026_PiNotifyMeasureRequest();
    }

    // KEY3 delegates all FPGA transitions to the Orange Pi. Re-applying an
    // MCU control word here can race its first IDLE/STEP command.
    if ((event & (uint16_t)~BSP_KEY3_PRESS) != BSP_KEY_EVENT_NONE) {
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
        case F2026_PI_COMMAND_IDLE:
            state.mode = F2026_FPGA_MODE_IDLE;
            state.free_run = false;
            F2026_PiReply("OK IDLE\r\n");
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
        case F2026_PI_COMMAND_PROBE: {
            uint32_t frame_us = command.value2;
            if (frame_us == 0U) {
                frame_us = F2026_PROBE_DEFAULT_FRAME_US;
            }
            if ((command.value >= F2026_PROBE_MIN_RAMP_US) &&
                (command.value <= F2026_PROBE_MAX_RAMP_US) &&
                (frame_us >= F2026_PROBE_MIN_FRAME_US) &&
                (frame_us <= F2026_PROBE_MAX_FRAME_US) &&
                ((frame_us % F2026_PROBE_FRAME_QUANTUM_US) == 0U) &&
                (command.value < frame_us)) {
                state.mode = F2026_FPGA_MODE_PROBE;
                state.free_run = true;
                state.probe_ramp_us = command.value;
                state.probe_frame_us = frame_us;
                F2026_PiReply("OK PROBE\r\n");
            } else {
                F2026_PiReply("ERR PROBE\r\n");
            }
            break;
        }
        case F2026_PI_COMMAND_STEP:
            if (command.value < 34U) {
                state.mode = F2026_FPGA_MODE_PROBE_TABLE;
                state.free_run = true;
                state.probe_ramp_us = command.value;
                state.probe_frame_us = F2026_PROBE_SWEEP_FRAME_US;
                state.last_control_valid = false;
                F2026_ApplyControl();
                F2026_PiReply("OK STEP\r\n");
            } else {
                F2026_PiReply("ERR STEP\r\n");
            }
            break;
        case F2026_PI_COMMAND_SWEEP:
            state.mode = F2026_FPGA_MODE_PROBE_SWEEP;
            state.free_run = true;
            state.probe_ramp_us = F2026_PROBE_SWEEP_FIRST_RAMP_US;
            state.probe_frame_us = F2026_PROBE_SWEEP_FRAME_US;
            state.last_control_valid = false;
            // Start the hardware before replying so the Pi timestamp is tied
            // to the beginning of the FPGA-controlled schedule.
            F2026_ApplyControl();
            F2026_PiReply("OK SWEEP\r\n");
            break;
        case F2026_PI_COMMAND_RESULT:
            state.vision_frequency_hz = command.value;
            state.vision_frequency_valid = command.value != 0U;
            state.vision_measurement_pending = false;
            F2026_PiReply("OK RESULT\r\n");
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
    bool vision_frequency = state.vision_frequency_valid;
    uint16_t status_color = state.communication_ok ? BSP_LCD_GREEN : BSP_LCD_RED;

    if (state.mode == F2026_FPGA_MODE_PROBE) {
        frequency_hz = state.probe_ramp_us;
    } else if (state.mode == F2026_FPGA_MODE_PROBE_TABLE) {
        frequency_hz = state.probe_ramp_us;
    } else if (state.mode == F2026_FPGA_MODE_PROBE_SWEEP) {
        frequency_hz = 0U;
    } else if (state.free_run) {
        frequency_hz = state.free_frequency_hz;
    } else if (state.fpga_status.period_ticks != 0U) {
        frequency_hz = 50000000U / state.fpga_status.period_ticks;
    }
    if (vision_frequency) {
        frequency_hz = state.vision_frequency_hz;
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
    BSP_LCD_ShowString(100U, 52U, vision_frequency ? "PI" : "", BSP_LCD_YELLOW, BSP_LCD_BLACK);
    BSP_LCD_ShowString(120U, 52U, vision_frequency ? "HZ" :
                       ((state.mode == F2026_FPGA_MODE_PROBE) ||
                        (state.mode == F2026_FPGA_MODE_PROBE_SWEEP)) ? "US" :
                       (state.mode == F2026_FPGA_MODE_PROBE_TABLE) ? "IDX" : "HZ",
                       BSP_LCD_GRAY, BSP_LCD_BLACK);

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

    BSP_LCD_ShowString(0U, 116U, "0:1X 1:90 2:2X 3:MEAS", BSP_LCD_GRAY, BSP_LCD_BLACK);
}

static void F2026_SendPiStatus(void)
{
    char response[192];
    uint32_t frequency_hz = (state.mode == F2026_FPGA_MODE_PROBE) ? state.probe_ramp_us :
        ((state.mode == F2026_FPGA_MODE_PROBE_TABLE) ? state.probe_ramp_us :
        ((state.mode == F2026_FPGA_MODE_PROBE_SWEEP) ? 0U :
        (state.free_run ? state.free_frequency_hz :
        ((state.fpga_status.period_ticks == 0U)
             ? 0U : (50000000U / state.fpga_status.period_ticks)))));

    (void)snprintf(response,
                   sizeof(response),
                   "STATUS MODE=%s AUTO=%u AMP=%u FREQ=%lu RAMP_US=%lu FRAME_US=%lu LOCK=%u ADC=%u,%u OTR=%u OUTPUT=%u COMM=%u FOUT=%u VER=%u\r\n",
                   F2026_ModeName(state.mode),
                   state.free_run ? 1U : 0U,
                   amplitude_divisions[state.amplitude_index],
                   (unsigned long)frequency_hz,
                   (unsigned long)state.probe_ramp_us,
                   (unsigned long)state.probe_frame_us,
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
    case F2026_FPGA_MODE_PROBE_SWEEP:
        return "SWEEP";
    case F2026_FPGA_MODE_PROBE_TABLE:
        return "TABLE";
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
