#include "bsp_system.h"

#include "bsp_board.h"

void Error_Handler(void)
{
    /* 关闭中断，避免错误状态下继续执行其他任务。 */
    __disable_irq();
    while (1) {
        /* 用 LED1 闪烁提示初始化或运行时错误。 */
        BSP_Board_LED_Toggle(BSP_LED1);
        for (volatile uint32_t i = 0; i < 200000U; i++) {
        }
    }
}
