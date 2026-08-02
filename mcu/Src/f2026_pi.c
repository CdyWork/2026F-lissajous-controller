#include "f2026_pi.h"

#include "bsp_uart.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define F2026_PI_LINE_SIZE 80U

typedef struct {
    UART_HandleTypeDef *uart;
    char line_buffer[F2026_PI_LINE_SIZE];
    uint8_t line_length;
} F2026_PiPort;

static F2026_PiPort pi_ports[2] = {
    {.uart = &huart1},
    {.uart = &huart3}
};
static UART_HandleTypeDef *reply_uart = &huart1;

static F2026_FpgaMode parse_mode(const char *text)
{
    if (strcmp(text, "DIAG") == 0) {
        return F2026_FPGA_MODE_DIAGONAL;
    }
    if (strcmp(text, "CIRCLE") == 0) {
        return F2026_FPGA_MODE_CIRCLE;
    }
    if (strcmp(text, "DOUBLE") == 0) {
        return F2026_FPGA_MODE_DOUBLE;
    }
    if (strcmp(text, "PROBE") == 0) {
        return F2026_FPGA_MODE_PROBE;
    }
    if (strcmp(text, "SWEEP") == 0) {
        return F2026_FPGA_MODE_PROBE_SWEEP;
    }
    if (strcmp(text, "TABLE") == 0) {
        return F2026_FPGA_MODE_PROBE_TABLE;
    }
    if (strcmp(text, "IDLE") == 0) {
        return F2026_FPGA_MODE_IDLE;
    }
    return F2026_FPGA_MODE_IDLE;
}

static bool parse_line(F2026_PiCommand *command, char *line_buffer)
{
    char *verb;
    char *arg1;
    char *arg2;

    command->type = F2026_PI_COMMAND_NONE;
    command->mode = F2026_FPGA_MODE_IDLE;
    command->value = 0U;
    command->value2 = 0U;

    verb = strtok(line_buffer, " \t");
    if (verb == 0) {
        return false;
    }
    arg1 = strtok(0, " \t");
    arg2 = strtok(0, " \t");

    if ((strcmp(verb, "IDLE") == 0) || (strcmp(verb, "OFF") == 0)) {
        command->type = F2026_PI_COMMAND_IDLE;
    } else if (strcmp(verb, "STATUS") == 0) {
        command->type = F2026_PI_COMMAND_STATUS;
    } else if (strcmp(verb, "BYPASS") == 0) {
        command->type = F2026_PI_COMMAND_BYPASS;
    } else if ((strcmp(verb, "TRACK") == 0) && (arg1 != 0)) {
        command->mode = parse_mode(arg1);
        if (command->mode != F2026_FPGA_MODE_IDLE) {
            command->type = F2026_PI_COMMAND_TRACK;
        }
    } else if ((strcmp(verb, "AUTO") == 0) && (arg1 != 0)) {
        command->mode = parse_mode(arg1);
        if (command->mode != F2026_FPGA_MODE_IDLE) {
            command->type = F2026_PI_COMMAND_AUTO;
        }
    } else if ((strcmp(verb, "AMP") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_AMPLITUDE;
        command->value = strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "FREQ") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_FREQUENCY;
        command->value = strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "TRIMQ") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_FREQUENCY_TRIM;
        command->value = (uint32_t)strtol(arg1, 0, 10);
    } else if ((strcmp(verb, "PHASE") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_PHASE;
        command->value = strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "PHASEQ") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_PHASE_FINE;
        command->value = strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "CAL") == 0) && (arg1 != 0) && (arg2 != 0)) {
        command->type = F2026_PI_COMMAND_CALIBRATE;
        command->value = strtoul(arg1, 0, 10);
        command->value2 = strtoul(arg2, 0, 10);
    } else if (strcmp(verb, "PROBE") == 0) {
        command->type = F2026_PI_COMMAND_PROBE;
        command->value = (arg1 == 0) ? 1000U : strtoul(arg1, 0, 10);
        command->value2 = (arg2 == 0) ? 0U : strtoul(arg2, 0, 10);
    } else if ((strcmp(verb, "STEP") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_STEP;
        command->value = strtoul(arg1, 0, 10);
    } else if (strcmp(verb, "SWEEP") == 0) {
        command->type = F2026_PI_COMMAND_SWEEP;
    } else if (strcmp(verb, "RESULT") == 0) {
        command->type = F2026_PI_COMMAND_RESULT;
        command->value = (arg1 == 0) ? 0U : strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "TRACKDONE") == 0) &&
               (arg1 != 0) && (arg2 != 0)) {
        command->type = F2026_PI_COMMAND_TRACK_RESULT;
        command->value = strtoul(arg1, 0, 10);
        command->value2 = strtoul(arg2, 0, 10);
    } else if ((strcmp(verb, "TASKDONE") == 0) &&
               (arg1 != 0) && (arg2 != 0)) {
        command->type = F2026_PI_COMMAND_TASK_RESULT;
        command->value = strtoul(arg1, 0, 10);
        command->value2 = strtoul(arg2, 0, 10);
    }

    return command->type != F2026_PI_COMMAND_NONE;
}

void F2026_PiInit(void)
{
    pi_ports[0].line_length = 0U;
    pi_ports[1].line_length = 0U;
    BSP_UART_InitMode(BSP_UART_MODE_TX_RX, BSP_UART_MODE_TX_RX);
    (void)BSP_UART1_StartReceiveDMA();
    (void)BSP_UART3_StartReceiveDMA();
    reply_uart = &huart1;
    F2026_PiReply("F2026 READY U1\r\n");
    reply_uart = &huart3;
    F2026_PiReply("F2026 READY U3\r\n");
    reply_uart = &huart1;
}

bool F2026_PiPoll(F2026_PiCommand *command)
{
    if (command == 0) {
        return false;
    }

    for (uint8_t i = 0U; i < 2U; i++) {
        F2026_PiPort *port = &pi_ports[i];
        uint8_t byte;

        while (BSP_UART_ReadBuffered(port->uart, &byte, 1U) == 1U) {
            char ch = (char)byte;

            if ((ch == '\r') || (ch == '\n')) {
                if (port->line_length == 0U) {
                    continue;
                }
                port->line_buffer[port->line_length] = '\0';
                for (uint8_t j = 0U; j < port->line_length; j++) {
                    port->line_buffer[j] =
                        (char)toupper((unsigned char)port->line_buffer[j]);
                }
                port->line_length = 0U;
                reply_uart = port->uart;
                if (parse_line(command, port->line_buffer)) {
                    return true;
                }
                F2026_PiReply("ERR COMMAND\r\n");
            } else if (port->line_length < (F2026_PI_LINE_SIZE - 1U)) {
                port->line_buffer[port->line_length++] = ch;
            } else {
                port->line_length = 0U;
                reply_uart = port->uart;
                F2026_PiReply("ERR LENGTH\r\n");
            }
        }
    }

    return false;
}

void F2026_PiReply(const char *text)
{
    if (text != 0) {
        (void)BSP_UART_Transmit(reply_uart, (uint8_t *)text,
                                (uint16_t)strlen(text), 100U);
    }
}

void F2026_PiNotifyMeasureRequest(uint8_t task_number)
{
    static const char *const requests[3] = {
        "MEASURE 1\r\n",
        "MEASURE 2\r\n",
        "MEASURE 3\r\n",
    };

    if ((task_number < 1U) || (task_number > 3U)) {
        return;
    }

    /* The Orange Pi is wired to USART3 (PB10/PB11), independently of the
       port that most recently issued a configuration command. */
    (void)BSP_UART_Transmit(&huart3,
                            (const uint8_t *)requests[task_number - 1U],
                            (uint16_t)strlen(requests[task_number - 1U]), 100U);
}

void F2026_PiNotifyTrackCalibrationRequest(uint8_t question_number)
{
    static const char *const requests[3] = {
        "TRACKCAL 1\r\n",
        "TRACKCAL 2\r\n",
        "TRACKCAL 3\r\n",
    };

    if ((question_number < 1U) || (question_number > 3U)) {
        return;
    }

    (void)BSP_UART_Transmit(&huart3,
                            (const uint8_t *)requests[question_number - 1U],
                            (uint16_t)strlen(requests[question_number - 1U]), 100U);
}
