#include "f2026_app.h"

#include "f2026_fpga.h"
#include "f2026_pi.h"

#include "bsp_board.h"
#include "bsp_dma.h"
#include "bsp_keypad.h"
#include "bsp_lcd.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define F2026_TASK_STACK_WORDS 768U
#define F2026_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define F2026_Q5_FREQUENCY_1_HZ 1100U
#define F2026_Q5_FREQUENCY_2_HZ 49900U
#define F2026_Q5_FREQUENCY_3_HZ 90900U
#define F2026_REFERENCE_CALIBRATION_HZ 100000U
#define F2026_REFERENCE_CALIBRATION_PERIODS 50000ULL
#define F2026_REFERENCE_CALIBRATION_EXPECTED_TICKS \
    (F2026_REFERENCE_CALIBRATION_PERIODS * \
     (50000000ULL / F2026_REFERENCE_CALIBRATION_HZ))
#define F2026_REFERENCE_CALIBRATION_SAVED_TICKS 24999942U
#define F2026_REFERENCE_CALIBRATION_TIMEOUT_MS 3000U
#define F2026_DEFAULT_PHASE_DEGREES 14U
#define F2026_DEFAULT_PHASE_WORD 0x09F49F49U
#define F2026_PROBE_DEFAULT_FRAME_US 10000U
#define F2026_PROBE_DEFAULT_RAMP_US 50U
#define F2026_PROBE_MIN_RAMP_US 10U
#define F2026_PROBE_MAX_RAMP_US 1800U
#define F2026_PROBE_FRAME_QUANTUM_US 200U
#define F2026_PROBE_MIN_FRAME_US F2026_PROBE_FRAME_QUANTUM_US
#define F2026_PROBE_MAX_FRAME_US 10000U
#define F2026_FPGA_TICKS_PER_US 50U
#define F2026_PROBE_SWEEP_FRAME_US 10000U
#define F2026_PROBE_SWEEP_FIRST_RAMP_US 10U
#define F2026_PROBE_TABLE_MID_INDEX 16U
#define F2026_VISION_MEASUREMENT_TIMEOUT_MS 12000U
#define F2026_TRACK_CALIBRATION_TIMEOUT_MS 15000U
#define F2026_COMPLETION_BEEP_MS 120U
#define F2026_MAX_FREQUENCY_TRIM_WORD 1000000L
typedef struct {
    F2026_FpgaMode mode;
    bool free_run;
    uint8_t amplitude_index;
    uint8_t amplitude_codes[4];
    uint32_t free_frequency_hz;
    int32_t frequency_trim_word;
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
    uint8_t vision_task_number;
    bool tracking_calibration_pending;
    bool tracking_calibration_valid;
    bool tracking_calibration_failed;
    uint8_t tracking_calibration_question;
    uint32_t tracking_calibration_started_ms;
    bool reference_calibration_pending;
    bool reference_calibration_started;
    bool reference_calibration_start_request;
    bool reference_calibration_valid;
    bool reference_calibration_failed;
    uint32_t reference_calibration_ticks;
    uint32_t reference_calibration_started_ms;
    bool completion_beep_active;
    uint32_t completion_beep_started_ms;
    bool blackout_sweep_active;
    F2026_FpgaMode blackout_restore_mode;
    bool blackout_restore_free_run;
} F2026_State;

static const uint8_t amplitude_divisions[4] = {2U, 4U, 6U, 8U};
static F2026_State state = {
    .mode = F2026_FPGA_MODE_IDLE,
    .free_run = false,
    .amplitude_index = 3U,
    .amplitude_codes = {13U, 26U, 38U, 51U},
    .free_frequency_hz = 1000U,
    .frequency_trim_word = 0,
    .probe_ramp_us = F2026_PROBE_DEFAULT_RAMP_US,
    .probe_frame_us = F2026_PROBE_DEFAULT_FRAME_US,
    .user_phase_word = F2026_DEFAULT_PHASE_WORD,
    .last_control_valid = false,
    .communication_ok = false,
    .output_active = false,
    .vision_frequency_hz = 0U,
    .vision_frequency_valid = false,
    .vision_measurement_pending = false,
    .vision_measurement_started_ms = 0U,
    .vision_task_number = 0U,
    .tracking_calibration_pending = false,
    .tracking_calibration_valid = false,
    .tracking_calibration_failed = false,
    .tracking_calibration_question = 0U,
    .tracking_calibration_started_ms = 0U,
    .reference_calibration_pending = false,
    .reference_calibration_started = false,
    .reference_calibration_start_request = false,
    .reference_calibration_valid = true,
    .reference_calibration_failed = false,
    .reference_calibration_ticks = F2026_REFERENCE_CALIBRATION_SAVED_TICKS,
    .reference_calibration_started_ms = 0U,
    .completion_beep_active = false,
    .completion_beep_started_ms = 0U,
    .blackout_sweep_active = false,
    .blackout_restore_mode = F2026_FPGA_MODE_IDLE,
    .blackout_restore_free_run = false
};

static void F2026_Task(void *argument);
static void F2026_HardwareInit(void);
static void F2026_ApplyControl(void);
static void F2026_ServiceKeys(void);
static void F2026_SelectTrackingMode(F2026_FpgaMode mode);
static void F2026_CycleAmplitude(void);
static void F2026_StartReferenceCalibration(void);
static void F2026_StartTrackingCalibration(uint8_t question_number);
static void F2026_StartVisionTask(uint8_t task_number);
static void F2026_StartBlackoutSweep(void);
static void F2026_StopBlackoutSweep(void);
static void F2026_ServicePi(void);
static void F2026_StartCompletionBeep(void);
static void F2026_ServiceCompletionBeep(void);
static void F2026_DrawStatus(void);
static void F2026_SendPiStatus(void);
static const char *F2026_ModeName(F2026_FpgaMode mode);
static int F2026_AmplitudeIndex(uint32_t divisions);
static bool F2026_IsQ5Frequency(uint32_t frequency_hz);
static uint64_t F2026_OutputPhaseIncrement(uint32_t frequency_hz);

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
    BSP_Keypad_Init();
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

            if (state.reference_calibration_pending &&
                state.reference_calibration_started &&
                latest_status.calibration_done) {
                state.reference_calibration_pending = false;
                state.reference_calibration_ticks = latest_status.calibration_ticks;
                state.reference_calibration_valid =
                    (latest_status.calibration_ticks >= 24000000U) &&
                    (latest_status.calibration_ticks <= 26000000U);
                state.reference_calibration_failed =
                    !state.reference_calibration_valid;
                state.last_control_valid = false;
                F2026_StartCompletionBeep();
            }

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
        F2026_ServiceCompletionBeep();
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
    control.calibration_start = state.reference_calibration_start_request;

    if (state.mode != F2026_FPGA_MODE_IDLE) {
        if (state.mode == F2026_FPGA_MODE_PROBE) {
            control.free_run = true;
            control.phase_increment = state.probe_ramp_us * F2026_FPGA_TICKS_PER_US;
            control.phase_offset = state.probe_frame_us * F2026_FPGA_TICKS_PER_US;
            control.output_enable = true;
        } else if (state.mode == F2026_FPGA_MODE_PROBE_SWEEP) {
            // The FPGA owns all eight ramp widths and switches only at the
            // 10 ms frame boundary. These values merely arm the free-run path.
            control.free_run = true;
            control.phase_increment =
                F2026_PROBE_SWEEP_FIRST_RAMP_US * F2026_FPGA_TICKS_PER_US;
            control.phase_offset =
                F2026_PROBE_SWEEP_FRAME_US * F2026_FPGA_TICKS_PER_US;
            control.output_enable = true;
        } else if (state.mode == F2026_FPGA_MODE_PROBE_TABLE) {
            // phase_increment carries the 0..31 table index. The FPGA latches
            // it on the next 10 ms frame boundary.
            control.free_run = true;
            control.phase_increment = state.probe_ramp_us;
            control.phase_offset =
                F2026_PROBE_SWEEP_FRAME_US * F2026_FPGA_TICKS_PER_US;
            control.output_enable = true;
        } else if (state.free_run) {
            control.phase_increment =
                F2026_OutputPhaseIncrement(state.free_frequency_hz);
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
            if (state.reference_calibration_start_request) {
                state.reference_calibration_start_request = false;
                state.reference_calibration_started = true;
            }
        } else {
            state.communication_ok = false;
            state.output_active = false;
        }
    }
}

static void F2026_ServiceKeys(void)
{
    uint16_t event = BSP_Key_Scan();
    char matrix_key = BSP_Keypad_Scan();
    bool matrix_key_handled = true;

    if (state.vision_measurement_pending &&
        ((uint32_t)(HAL_GetTick() - state.vision_measurement_started_ms) >=
         F2026_VISION_MEASUREMENT_TIMEOUT_MS)) {
        state.vision_measurement_pending = false;
    }
    if (state.tracking_calibration_pending &&
        ((uint32_t)(HAL_GetTick() - state.tracking_calibration_started_ms) >=
         F2026_TRACK_CALIBRATION_TIMEOUT_MS)) {
        state.tracking_calibration_pending = false;
        state.tracking_calibration_failed = true;
    }

    if (state.blackout_sweep_active) {
        if (matrix_key == 'D') {
            F2026_StopBlackoutSweep();
            BSP_Beep_Write(true);
            vTaskDelay(pdMS_TO_TICKS(15U));
            BSP_Beep_Write(false);
        }
        return;
    }

    if ((event & BSP_KEY0_PRESS) != 0U) {
        F2026_SelectTrackingMode(F2026_FPGA_MODE_DIAGONAL);
    }
    if ((event & BSP_KEY1_PRESS) != 0U) {
        F2026_SelectTrackingMode(F2026_FPGA_MODE_CIRCLE);
    }
    if (((event & BSP_KEY2_PRESS) != 0U) && !state.reference_calibration_pending) {
        F2026_StartReferenceCalibration();
    }

    if (((event & BSP_KEY3_PRESS) != 0U) &&
        !state.vision_measurement_pending) {
        F2026_StartVisionTask(1U);
    }

    switch (matrix_key) {
    case '1':
    case '2':
    case '3':
        F2026_StartTrackingCalibration((uint8_t)(matrix_key - '0'));
        break;
    case 'A':
        F2026_CycleAmplitude();
        break;
    case '4':
    case '5':
    case '6':
        F2026_StartVisionTask((uint8_t)(matrix_key - '3'));
        break;
    case 'B':
        F2026_StartReferenceCalibration();
        break;
    case '0':
        F2026_StartBlackoutSweep();
        break;
    default:
        matrix_key_handled = false;
        break;
    }

    if (state.reference_calibration_pending &&
        ((uint32_t)(HAL_GetTick() - state.reference_calibration_started_ms) >=
         F2026_REFERENCE_CALIBRATION_TIMEOUT_MS)) {
        state.reference_calibration_pending = false;
        state.reference_calibration_start_request = false;
        state.reference_calibration_failed = true;
    }

    if (((event & (uint16_t)~BSP_KEY3_PRESS) != BSP_KEY_EVENT_NONE) ||
        matrix_key_handled) {
        BSP_Beep_Write(true);
        vTaskDelay(pdMS_TO_TICKS(15U));
        BSP_Beep_Write(false);
        state.last_control_valid = false;
    }
}

static void F2026_SelectTrackingMode(F2026_FpgaMode mode)
{
    if (state.vision_measurement_pending || state.reference_calibration_pending ||
        state.tracking_calibration_pending) {
        return;
    }

    state.mode = mode;
    state.free_run = false;
    state.vision_frequency_hz = 0U;
    state.vision_frequency_valid = false;
    state.tracking_calibration_valid = false;
    state.tracking_calibration_failed = false;
    state.last_control_valid = false;
}

static void F2026_CycleAmplitude(void)
{
    if (state.vision_measurement_pending || state.reference_calibration_pending ||
        state.tracking_calibration_pending) {
        return;
    }

    if ((state.mode != F2026_FPGA_MODE_DIAGONAL) &&
        (state.mode != F2026_FPGA_MODE_CIRCLE) &&
        (state.mode != F2026_FPGA_MODE_DOUBLE)) {
        state.mode = F2026_FPGA_MODE_DIAGONAL;
    }
    state.free_run = false;
    state.vision_frequency_hz = 0U;
    state.vision_frequency_valid = false;
    state.tracking_calibration_valid = false;
    state.tracking_calibration_failed = false;
    state.amplitude_index = (uint8_t)((state.amplitude_index + 1U) % 4U);
    state.last_control_valid = false;
}

static void F2026_StartReferenceCalibration(void)
{
    if (state.reference_calibration_pending || state.vision_measurement_pending ||
        state.tracking_calibration_pending) {
        return;
    }

    state.vision_frequency_hz = 0U;
    state.vision_frequency_valid = false;
    state.vision_measurement_pending = false;
    state.tracking_calibration_valid = false;
    state.tracking_calibration_failed = false;
    state.mode = F2026_FPGA_MODE_DIAGONAL;
    state.free_run = true;
    state.free_frequency_hz = F2026_REFERENCE_CALIBRATION_HZ;
    state.reference_calibration_pending = true;
    state.reference_calibration_started = false;
    state.reference_calibration_start_request = true;
    state.reference_calibration_valid = false;
    state.reference_calibration_failed = false;
    state.reference_calibration_ticks = 0U;
    state.reference_calibration_started_ms = HAL_GetTick();
    state.last_control_valid = false;
}

static void F2026_StartTrackingCalibration(uint8_t question_number)
{
    if ((question_number < 1U) || (question_number > 3U) ||
        state.tracking_calibration_pending || state.vision_measurement_pending ||
        state.reference_calibration_pending) {
        return;
    }

    state.mode = F2026_FPGA_MODE_DIAGONAL;
    state.free_run = false;
    state.user_phase_word = F2026_DEFAULT_PHASE_WORD;
    state.vision_frequency_hz = 0U;
    state.vision_frequency_valid = false;
    state.tracking_calibration_pending = true;
    state.tracking_calibration_valid = false;
    state.tracking_calibration_failed = false;
    state.tracking_calibration_question = question_number;
    state.tracking_calibration_started_ms = HAL_GetTick();
    state.last_control_valid = false;
    F2026_PiNotifyTrackCalibrationRequest(question_number);
}

static void F2026_StartVisionTask(uint8_t task_number)
{
    if ((task_number < 1U) || (task_number > 3U) ||
        state.vision_measurement_pending ||
        state.reference_calibration_pending ||
        state.tracking_calibration_pending) {
        return;
    }

    state.vision_frequency_hz = 0U;
    state.vision_frequency_valid = false;
    state.frequency_trim_word = 0;
    state.vision_measurement_pending = true;
    state.vision_measurement_started_ms = HAL_GetTick();
    state.vision_task_number = task_number;
    F2026_PiNotifyMeasureRequest(task_number);
}

static void F2026_StartBlackoutSweep(void)
{
    if (state.blackout_sweep_active || state.vision_measurement_pending ||
        state.reference_calibration_pending ||
        state.tracking_calibration_pending) {
        return;
    }

    state.blackout_restore_mode = state.mode;
    state.blackout_restore_free_run = state.free_run;
    state.blackout_sweep_active = true;
    state.mode = F2026_FPGA_MODE_PROBE_SWEEP;
    state.free_run = true;
    state.last_control_valid = false;

    BSP_LCD_Clear(BSP_LCD_BLACK);
    BSP_LCD_SetBacklight(false);
}

static void F2026_StopBlackoutSweep(void)
{
    if (!state.blackout_sweep_active) {
        return;
    }

    state.mode = state.blackout_restore_mode;
    state.free_run = state.blackout_restore_free_run;
    state.blackout_sweep_active = false;
    state.last_control_valid = false;

    BSP_LCD_Clear(BSP_LCD_BLACK);
    BSP_LCD_SetBacklight(true);
}

static void F2026_StartCompletionBeep(void)
{
    state.completion_beep_active = true;
    state.completion_beep_started_ms = HAL_GetTick();
    BSP_Beep_Write(true);
}

static void F2026_ServiceCompletionBeep(void)
{
    if (state.completion_beep_active &&
        ((uint32_t)(HAL_GetTick() - state.completion_beep_started_ms) >=
         F2026_COMPLETION_BEEP_MS)) {
        BSP_Beep_Write(false);
        state.completion_beep_active = false;
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
            // The Pi captures immediately after the command reply.  Apply the
            // phase word before acknowledging it so that reply is a real
            // hardware boundary rather than a queued MCU request.
            state.last_control_valid = false;
            F2026_ApplyControl();
            F2026_PiReply("OK PHASE\r\n");
            break;
        case F2026_PI_COMMAND_PHASE_FINE:
            if (command.value < 360000U) {
                state.user_phase_word = (uint32_t)((((uint64_t)command.value) << 32U) / 360000ULL);
                state.last_control_valid = false;
                F2026_ApplyControl();
                F2026_PiReply("OK PHASEQ\r\n");
            } else {
                F2026_PiReply("ERR PHASEQ\r\n");
            }
            break;
        case F2026_PI_COMMAND_FREQUENCY_TRIM: {
            int32_t trim_word = (int32_t)command.value;
            if ((trim_word >= -F2026_MAX_FREQUENCY_TRIM_WORD) &&
                (trim_word <= F2026_MAX_FREQUENCY_TRIM_WORD) &&
                state.free_run) {
                state.frequency_trim_word = trim_word;
                state.last_control_valid = false;
                F2026_ApplyControl();
                F2026_PiReply("OK TRIMQ\r\n");
            } else {
                F2026_PiReply("ERR TRIMQ\r\n");
            }
            break;
        }
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
            if (F2026_IsQ5Frequency(command.value)) {
                state.vision_frequency_hz = command.value;
                state.vision_frequency_valid = true;
                state.vision_measurement_pending = false;
                state.mode = F2026_FPGA_MODE_DIAGONAL;
                state.free_run = true;
                state.free_frequency_hz = command.value;
                state.last_control_valid = false;
                F2026_ApplyControl();
                F2026_PiReply("OK RESULT\r\n");
            } else if (command.value == 0U) {
                state.vision_frequency_hz = 0U;
                state.vision_frequency_valid = false;
                state.vision_measurement_pending = false;
                state.mode = F2026_FPGA_MODE_IDLE;
                state.free_run = false;
                state.last_control_valid = false;
                F2026_ApplyControl();
                F2026_PiReply("OK RESULT\r\n");
            } else {
                F2026_PiReply("ERR RESULT\r\n");
            }
            break;
        case F2026_PI_COMMAND_TRACK_RESULT:
            if (state.tracking_calibration_pending &&
                (command.value == state.tracking_calibration_question) &&
                (command.value2 <= 1U)) {
                state.tracking_calibration_pending = false;
                state.tracking_calibration_valid = command.value2 == 1U;
                state.tracking_calibration_failed = command.value2 == 0U;
                F2026_PiReply("OK TRACKDONE\r\n");
                F2026_StartCompletionBeep();
            } else {
                F2026_PiReply("ERR TRACKDONE\r\n");
            }
            break;
        case F2026_PI_COMMAND_TASK_RESULT:
            if ((command.value >= 1U) && (command.value <= 3U) &&
                (command.value == state.vision_task_number) &&
                (command.value2 <= 1U)) {
                state.vision_task_number = 0U;
                F2026_PiReply("OK TASKDONE\r\n");
                F2026_StartCompletionBeep();
            } else {
                F2026_PiReply("ERR TASKDONE\r\n");
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
    bool vision_frequency = state.vision_frequency_valid;
    uint16_t status_color = state.communication_ok ? BSP_LCD_GREEN : BSP_LCD_RED;

    if (state.blackout_sweep_active) {
        return;
    }

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

    if (state.tracking_calibration_pending ||
        state.tracking_calibration_valid ||
        state.tracking_calibration_failed) {
        BSP_LCD_ShowString(0U, 84U, "QCAL", BSP_LCD_CYAN, BSP_LCD_BLACK);
        BSP_LCD_ShowU32(40U, 84U, state.tracking_calibration_question,
                        BSP_LCD_WHITE, BSP_LCD_BLACK);
        BSP_LCD_ShowString(56U, 84U,
                           state.tracking_calibration_pending ? "RUN" :
                           state.tracking_calibration_valid ? "OK" : "FAIL",
                           state.tracking_calibration_pending ? BSP_LCD_YELLOW :
                           state.tracking_calibration_valid ? BSP_LCD_GREEN : BSP_LCD_RED,
                           BSP_LCD_BLACK);
    } else if (state.reference_calibration_pending ||
        state.reference_calibration_valid ||
        state.reference_calibration_failed) {
        BSP_LCD_ShowString(0U, 84U, "CAL", BSP_LCD_CYAN, BSP_LCD_BLACK);
        BSP_LCD_ShowString(40U, 84U,
                           state.reference_calibration_pending ? "RUN" :
                           state.reference_calibration_valid ? "OK" : "FAIL",
                           state.reference_calibration_pending ? BSP_LCD_YELLOW :
                           state.reference_calibration_valid ? BSP_LCD_GREEN : BSP_LCD_RED,
                           BSP_LCD_BLACK);
        if (state.reference_calibration_valid) {
            BSP_LCD_ShowString(72U, 84U, "T", BSP_LCD_GRAY, BSP_LCD_BLACK);
            BSP_LCD_ShowU32(80U, 84U, state.reference_calibration_ticks,
                            BSP_LCD_WHITE, BSP_LCD_BLACK);
        }
    } else {
        BSP_LCD_ShowString(0U, 84U, "ADC", BSP_LCD_CYAN, BSP_LCD_BLACK);
        BSP_LCD_ShowU32(40U, 84U, state.fpga_status.sample_min, BSP_LCD_WHITE, BSP_LCD_BLACK);
        BSP_LCD_ShowString(72U, 84U, "-", BSP_LCD_GRAY, BSP_LCD_BLACK);
        BSP_LCD_ShowU32(88U, 84U, state.fpga_status.sample_max, BSP_LCD_WHITE, BSP_LCD_BLACK);
    }

    BSP_LCD_ShowString(0U, 100U, "FPGA", BSP_LCD_CYAN, BSP_LCD_BLACK);
    BSP_LCD_ShowString(48U, 100U,
                       state.communication_ok ? "ONLINE" : "OFFLINE",
                       status_color,
                       BSP_LCD_BLACK);
    BSP_LCD_ShowString(112U, 100U,
                       state.output_active ? "RUN" : "MUTE",
                       state.output_active ? BSP_LCD_GREEN : BSP_LCD_YELLOW,
                       BSP_LCD_BLACK);

    BSP_LCD_ShowString(0U, 116U, "Q:123A T:456 0:S D:X", BSP_LCD_GRAY, BSP_LCD_BLACK);
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
                   "STATUS MODE=%s AUTO=%u AMP=%u FREQ=%lu RAMP_US=%lu FRAME_US=%lu LOCK=%u ADC=%u,%u OTR=%u OUTPUT=%u COMM=%u FOUT=%u CAL=%u CTICKS=%lu VER=%u\r\n",
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
                    state.reference_calibration_valid ? 1U : 0U,
                    (unsigned long)state.reference_calibration_ticks,
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

static bool F2026_IsQ5Frequency(uint32_t frequency_hz)
{
    return (frequency_hz == F2026_Q5_FREQUENCY_1_HZ) ||
           (frequency_hz == F2026_Q5_FREQUENCY_2_HZ) ||
           (frequency_hz == F2026_Q5_FREQUENCY_3_HZ);
}

static uint64_t F2026_OutputPhaseIncrement(uint32_t frequency_hz)
{
    uint64_t phase_increment = F2026_PhaseIncrementFromHz(frequency_hz);
    int64_t adjusted_increment;

    if (state.reference_calibration_valid &&
        (state.reference_calibration_ticks != 0U)) {
        phase_increment = ((phase_increment *
                         F2026_REFERENCE_CALIBRATION_EXPECTED_TICKS) +
                        (state.reference_calibration_ticks / 2U)) /
                       state.reference_calibration_ticks;
    }
    adjusted_increment = (int64_t)phase_increment + state.frequency_trim_word;
    return adjusted_increment > 0 ? (uint64_t)adjusted_increment : 1ULL;
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
