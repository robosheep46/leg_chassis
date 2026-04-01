#include "motor_task.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include "LK9025.h"
// #include "servo_motor.h"
// #include "mi_motor.h"
void MotorControlTask()
{
    LKMotorControl();
}


static TaskHandle_t motorTaskHandle = NULL;
const osThreadAttr_t motor_task_attributes = {
    .name = "MotorTask",
    .stack_size = 256 * 4,  // 256 words = 1024 bytes
    .priority = (osPriority_t) osPriorityNormal,
};

__attribute__((noreturn)) void StartMOTORTASK(void *argument)
{
    static float motor_dt;
    static float motor_start;
    for (;;)
    {
        motor_start = dwt_get_time_ms();
        MotorControlTask();
        motor_dt = dwt_get_time_ms() - motor_start;
        // if (motor_dt > 1)
        //     ;
        osDelay(2);
    }
}

void MotorTaskInit(void)
{
    motorTaskHandle = osThreadNew(StartMOTORTASK, NULL, &motor_task_attributes);
}

