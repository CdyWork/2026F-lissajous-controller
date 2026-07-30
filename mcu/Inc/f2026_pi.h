#ifndef F2026_PI_H
#define F2026_PI_H

#include <stdbool.h>
#include <stdint.h>

#include "f2026_fpga.h"

typedef enum {
    F2026_PI_COMMAND_NONE = 0,
    F2026_PI_COMMAND_STATUS,
    F2026_PI_COMMAND_BYPASS,
    F2026_PI_COMMAND_TRACK,
    F2026_PI_COMMAND_AUTO,
    F2026_PI_COMMAND_AMPLITUDE,
    F2026_PI_COMMAND_FREQUENCY,
    F2026_PI_COMMAND_PHASE,
    F2026_PI_COMMAND_CALIBRATE
} F2026_PiCommandType;

typedef struct {
    F2026_PiCommandType type;
    F2026_FpgaMode mode;
    uint32_t value;
    uint32_t value2;
} F2026_PiCommand;

void F2026_PiInit(void);
bool F2026_PiPoll(F2026_PiCommand *command);
void F2026_PiReply(const char *text);

#endif
