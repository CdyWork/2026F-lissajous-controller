#ifndef AD9833_H
#define AD9833_H

#include <stdint.h>

/* AD9833 module MCLK is supplied by its onboard 25 MHz oscillator. */
#define AD9833_MCLK_HZ 25000000UL

/* Configure the bit-banged AD9833 interface on PE8/PE9/PE10. */
void AD9833_Init(void);

/* Start continuous sine-wave output using both frequency registers. */
void AD9833_SetFrequency(uint32_t frequency_hz);

/* Re-write the configured frequency register while DDS remains running. */
void AD9833_Refresh(void);

#endif
