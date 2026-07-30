#ifndef F2026_FPGA_H
#define F2026_FPGA_H

#include <stdbool.h>
#include <stdint.h>

#define F2026_FPGA_PROTOCOL_VERSION 2U

typedef enum {
    F2026_FPGA_MODE_IDLE = 0,
    F2026_FPGA_MODE_DIAGONAL = 1,
    F2026_FPGA_MODE_CIRCLE = 2,
    F2026_FPGA_MODE_DOUBLE = 3
} F2026_FpgaMode;

typedef struct {
    uint8_t protocol_version;
    bool locked;
    bool otr_seen;
    bool output_enabled;
    bool free_run;
    uint32_t period_ticks;
    uint32_t edge_count;
    uint8_t sample_min;
    uint8_t sample_max;
    F2026_FpgaMode mode;
    uint8_t amplitude_code;
} F2026_FpgaStatus;

typedef struct {
    F2026_FpgaMode mode;
    uint8_t amplitude_code;
    bool output_enable;
    bool free_run;
    uint32_t phase_increment;
    uint32_t phase_offset;
    uint8_t dac_mid;
    uint8_t threshold_hysteresis;
} F2026_FpgaControl;

void F2026_FpgaInterfaceInit(void);
void F2026_FpgaReset(bool release_reset);
bool F2026_FpgaReadStatus(F2026_FpgaStatus *status);
bool F2026_FpgaWriteControl(const F2026_FpgaControl *control);
uint32_t F2026_PhaseIncrementFromHz(uint32_t frequency_hz);
uint32_t F2026_PhaseWordFromDegrees(uint32_t degrees);

#endif
