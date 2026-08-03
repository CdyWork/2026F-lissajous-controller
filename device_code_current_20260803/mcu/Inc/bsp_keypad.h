#ifndef BSP_KEYPAD_H
#define BSP_KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* 4x4 matrix keypad: PD0..PD3 rows and PD4..PD7 columns. */
void BSP_Keypad_Init(void);

/* Return one debounced key press, or '\0' when no new key was pressed. */
char BSP_Keypad_Scan(void);

#ifdef __cplusplus
}
#endif

#endif
