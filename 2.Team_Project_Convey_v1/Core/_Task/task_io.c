/*
 * task_io.c
 *
 *  Created on: Jan 18, 2026
 *      Author: kangs
 */

void TaskIO_Entry(void *argument) {
    for(;;) {
        // PLC 통역사에게 물어보고 결과를 장부에 적음
        g_sys_status.isPlcActive = BSP_PLC_IsActive();

        osDelay(10); // RTOS의 효율적인 폴링
    }
}
