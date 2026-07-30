#include "f2026_pi.h"

#include "bsp_uart.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define F2026_PI_LINE_SIZE 80U

static char line_buffer[F2026_PI_LINE_SIZE];
static uint8_t line_length;

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
    return F2026_FPGA_MODE_IDLE;
}

static bool parse_line(F2026_PiCommand *command)
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

    if (strcmp(verb, "STATUS") == 0) {
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
    } else if ((strcmp(verb, "PHASE") == 0) && (arg1 != 0)) {
        command->type = F2026_PI_COMMAND_PHASE;
        command->value = strtoul(arg1, 0, 10);
    } else if ((strcmp(verb, "CAL") == 0) && (arg1 != 0) && (arg2 != 0)) {
        command->type = F2026_PI_COMMAND_CALIBRATE;
        command->value = strtoul(arg1, 0, 10);
        command->value2 = strtoul(arg2, 0, 10);
    }

    return command->type != F2026_PI_COMMAND_NONE;
}

void F2026_PiInit(void)
{
    line_length = 0U;
    BSP_UART_InitMode(BSP_UART_MODE_TX_RX, BSP_UART_MODE_TX_ONLY);
    (void)BSP_UART1_StartReceiveDMA();
    F2026_PiReply("F2026 READY\r\n");
}

bool F2026_PiPoll(F2026_PiCommand *command)
{
    uint8_t byte;

    if (command == 0) {
        return false;
    }

    while (BSP_UART_ReadBuffered(&huart1, &byte, 1U) == 1U) {
        char ch = (char)byte;

        if ((ch == '\r') || (ch == '\n')) {
            if (line_length == 0U) {
                continue;
            }
            line_buffer[line_length] = '\0';
            for (uint8_t j = 0U; j < line_length; j++) {
                line_buffer[j] = (char)toupper((unsigned char)line_buffer[j]);
            }
            line_length = 0U;
            if (parse_line(command)) {
                return true;
            }
            F2026_PiReply("ERR COMMAND\r\n");
        } else if (line_length < (F2026_PI_LINE_SIZE - 1U)) {
            line_buffer[line_length++] = ch;
        } else {
            line_length = 0U;
            F2026_PiReply("ERR LENGTH\r\n");
        }
    }

    return false;
}

void F2026_PiReply(const char *text)
{
    if (text != 0) {
        (void)BSP_UART1_Print(text);
    }
}
